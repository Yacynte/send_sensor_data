// #include <opencv2/opencv.hpp>
// #include <opencv2/features2d.hpp> // For ORB

#include <thread>
#include <iostream>
#include <algorithm> 
#include <chrono>
#include "StereoCamera.h"
#include "LidarScanner.h"
#include <atomic>

#include <queue>
#include <mutex>
#include <condition_variable>
#include <termios.h> // For termios functions


#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <fstream>


#define UDP_IP "192.168.178.28"
#define UDP_PORT 5005
#define PACKET_SIZE 4096

std::atomic<bool> interupt(false);  // Atomic flag to safely stop the process

std::queue<std::pair<std::pair<cv::Mat, int>,std::pair<std::string, std::string>>> imageQueue;
std::mutex imageMutex;
std::condition_variable imageCondVar;
bool stopImageSaving = false;


// Non-blocking keyboard input function for ESC key press
void listen_for_esc() {
    struct termios oldt, newt;
    tcgetattr(STDIN_FILENO, &oldt);  // Get current terminal settings
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO); // Disable canonical mode and echo
    tcsetattr(STDIN_FILENO, TCSANOW, &newt); // Apply new settings

    while (true) {
        char ch = getchar(); // Read a single character
        if (ch == 27 || interupt) { // ESC key
            interupt = true;  // Set interrupt flag
            stopImageSaving = true;
            std::cout << "Esc key pressed. Interrupt signal sent!" << std::endl;
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);  // Restore old terminal settings
}

void sendData() {
    // Create a TCP socket
    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) {
        perror("Socket creation failed");
        return;
    }

    // Setup server address
    struct sockaddr_in server_addr;
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(UDP_PORT);
    server_addr.sin_addr.s_addr = inet_addr(UDP_IP);

    // Connect to the server
    if (connect(sock, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        perror("Connection to server failed");
        close(sock);
        return;
    }

    while (true) {
        std::unique_lock<std::mutex> lock(imageMutex);
        imageCondVar.wait(lock, [] { return !imageQueue.empty() || stopImageSaving; });

        // Exit loop if stop signal is received and queue is empty
        if (stopImageSaving && imageQueue.empty()) break;

        if (!imageQueue.empty()) {
            auto dataPair = imageQueue.front();
            imageQueue.pop();
            lock.unlock();

            auto& [image, socket] = dataPair.first;
            auto& [timestamp, label] = dataPair.second;

            std::vector<uchar> buffer;
            cv::imencode(".jpg", image, buffer);

            // Prepare header with label and timestamp
            std::string header = label + "|" + timestamp + "|";
            size_t header_size = header.size();
            size_t image_size = buffer.size();

            // Send header size and header
            if (send(sock, &header_size, sizeof(header_size), 0) < 0) {
                perror("Failed to send header size");
                break;
            }
            if (send(sock, header.data(), header_size, 0) < 0) {
                perror("Failed to send header");
                break;
            }

            // Send image size and image data
            if (send(sock, &image_size, sizeof(image_size), 0) < 0) {
                perror("Failed to send image size");
                break;
            }
            if (send(sock, buffer.data(), image_size, 0) < 0) {
                perror("Failed to send image data");
                break;
            }
        }
    }

    // Close the socket connection
    close(sock);
}





void camera_record(StereoCamera& stereoCam, int sock) {
    // Initialize time tracking and frame counter
    auto lastCaptureTime = std::chrono::steady_clock::now();
    int frameCounter = 0;
    
    // Matrices to store left and right stereo frames
    cv::Mat leftFrame, rightFrame;
    
    while (!interupt) { // Loop until interrupted
        auto currentTime = std::chrono::steady_clock::now();
        
        // Ensure frame capture at approximately 20 FPS (every 50 ms)
        if (std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - lastCaptureTime).count() >= 50) {
            std::string timestamp;
            
            // Capture stereo frames
            if (stereoCam.captureFrames(leftFrame, rightFrame, timestamp)) {
                lastCaptureTime = std::chrono::steady_clock::now(); // Update last capture time
                
                // Lock the queue and push both frames with their respective metadata
                {
                    std::lock_guard<std::mutex> lock(imageMutex);
                    imageQueue.emplace(std::make_pair(leftFrame, sock), std::make_pair(timestamp, "L")); // Left frame
                    imageQueue.emplace(std::make_pair(rightFrame, sock), std::make_pair(timestamp, "R")); // Right frame
                }
                
                // Notify waiting threads that new images are available
                imageCondVar.notify_all();
            } else {
                // Handle capture failure
                std::cerr << "Failed to obtain camera frames" << std::endl;
                interupt = true; // Set interruption flag to stop loop
                break;
            }
        }
    }
}


    
void lidar_record(LidarScanner& lidarscan, int sock) {
    // Attempt to initialize the LiDAR scanner
    if (!lidarscan.initialize()) {
        std::cerr << "RPLIDAR C1 initialization failed!" << std::endl;
        return;
    }

    auto lastCaptureTime = std::chrono::steady_clock::now();
    cv::Mat scans_cur; // Matrix to store current LiDAR scans

    while (!interupt) {
        auto currentTime = std::chrono::steady_clock::now();
        
        // Capture data at 40 FPS (25 ms per frame)
        if (std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - lastCaptureTime).count() >= 25) {
            std::string timestamp;
            
            if (lidarscan.getScans(scans_cur, timestamp)) {
                lastCaptureTime = std::chrono::steady_clock::now(); // Update last capture time
                
                // Lock the queue and push new LiDAR scan
                {
                    std::lock_guard<std::mutex> lock(imageMutex);
                    imageQueue.push({{scans_cur, sock}, {timestamp, "D"}});
                }
                imageCondVar.notify_one(); // Notify saving thread
            } else {
                std::cerr << "Failed to obtain LiDAR scans" << std::endl;
                interupt = true;
                break;
            }
        }
    }
}


int main(int argc, char** argv) {
    // Ensure correct usage
    if (argc < 4) {
        std::cerr << "Usage: " << argv[0] << " <left_camera_id> <right_camera_id> <lidar_port>" << std::endl;
        return -1;
    }

    // Create UDP socket
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        perror("Socket creation failed");
        return -1;
    }

    // Parse camera IDs and LiDAR port from command-line arguments
    int left_camera_id, right_camera_id;
    std::string lidar_port = argv[3];
    std::istringstream(argv[1]) >> left_camera_id;
    std::istringstream(argv[2]) >> right_camera_id;

    // Initialize camera and LiDAR scanner
    StereoCamera stereoCam(left_camera_id, right_camera_id);
    LidarScanner lidarscan(lidar_port);
    std::cout << "StereoCamera and Lidar Initialized" << std::endl;

    // Launch user input listener in a separate thread
    std::thread inputThread(listen_for_esc);
    
    // Start data transmission thread
    std::thread saveDataThread(sendData);
    
    // Start camera recording thread
    std::thread cameraThread(camera_record, std::ref(stereoCam), sock);
    
    // Start LiDAR recording thread
    std::thread lidarThread(lidar_record, std::ref(lidarscan), sock);
    
    // Wait for recording threads to finish
    lidarThread.join();
    cameraThread.join();
    saveDataThread.join();
    
    // Join user input thread
    inputThread.join();

    std::cout << "Recording complete" << std::endl;
    return 0;
}
