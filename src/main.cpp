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
#include <filesystem>
#include <termios.h> // For termios functions


#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <fstream>

#include <queue>
#include <mutex>
#include <boost/lockfree/queue.hpp>

#define UDP_IP "192.168.178.28"
#define UDP_PORT 5005
#define PACKET_SIZE 4096

std::atomic<bool> interupt(false);  // Atomic flag to safely stop the process

// std::queue<std::pair<std::pair<cv::Mat, int>,std::pair<std::string, std::string>>> imageQueue;

boost::lockfree::queue<std::pair<std::pair<cv::Mat, int>, std::pair<std::string, std::string>>> imageQueue;

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
    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) {
        perror("Socket creation failed");
        return;
    }

    struct sockaddr_in server_addr;
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(UDP_PORT);
    server_addr.sin_addr.s_addr = inet_addr(UDP_IP);

    if (connect(sock, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        perror("Connection to server failed");
        close(sock);
        return;
    }

    while (true) {
        std::unique_lock<std::mutex> lock(imageMutex);
        imageCondVar.wait(lock, [] { return !imageQueue.empty() || stopImageSaving; });

        if (stopImageSaving && imageQueue.empty()) break;

        auto dataPair = std::move(imageQueue.front());
        imageQueue.pop();
        lock.unlock();

        auto& [image, socket] = dataPair.first;
        auto& [timestamp, label] = dataPair.second;

        // Encode image asynchronously
        auto encodeFuture = std::async(std::launch::async, [&]() {
            std::vector<uchar> buffer;
            cv::imencode(".jpg", image, buffer); // Faster than PNG
            return buffer;
        });

        std::vector<uchar> buffer = encodeFuture.get();

        std::string header = label + "|" + timestamp + "|";

        size_t header_size = header.size();
        size_t image_size = buffer.size();

        // Send header size and header
        send(sock, &header_size, sizeof(header_size), 0);
        send(sock, header.data(), header_size, 0);

        // Send image size and image
        send(sock, &image_size, sizeof(image_size), 0);
        send(sock, buffer.data(), image_size, 0);
        
    }

    close(sock);
}





void camera_record(StereoCamera& stereoCam, int sock){
    auto lastFrameTime = std::chrono::steady_clock::now();
    int frameCounter = 0;

    while (!interupt) {
        auto currentTime = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - lastFrameTime).count();
        
        if (elapsed >= 50) { // 20 FPS
            cv::Mat leftFrame, rightFrame;
            std::string timestamp;

            if (stereoCam.captureFrames(leftFrame, rightFrame, timestamp)) {
                std::cout << frameCounter++ << std::endl;

                // Lock once and push both frames together
                {
                    std::lock_guard<std::mutex> lock(imageMutex);
                    imageQueue.emplace(std::make_pair(leftFrame, sock), std::make_pair(timestamp, "L"));
                    imageQueue.emplace(std::make_pair(rightFrame, sock), std::make_pair(timestamp, "R"));
                }
                imageCondVar.notify_all();
            } else {
                std::cerr << "Camera capture failed!" << std::endl;
                interupt = true;
                break;
            }
            lastFrameTime = std::chrono::steady_clock::now(); // Update time after capturing

        }
    }
}



    
void lidar_record(LidarScanner& lidarscan, int sock){
    if (!lidarscan.initialize()) {
        std::cerr << "LIDAR initialization failed!" << std::endl;
        return;
    }

    auto lastScanTime = std::chrono::steady_clock::now();
    cv::Mat scans_cur;

    while (!interupt) {
        auto currentTime = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - lastScanTime).count() >= 25) {
            std::string timestamp;
            if (lidarscan.getScans(scans_cur, timestamp)) {
                std::lock_guard<std::mutex> lock(imageMutex);
                imageQueue.emplace(std::make_pair(scans_cur, sock), std::make_pair(timestamp, "D"));
                imageCondVar.notify_one();
            } else {
                std::cerr << "Lidar capture failed!" << std::endl;
                interupt = true;
                break;
            }
            lastScanTime = std::chrono::steady_clock::now(); // Update time after capture
        }
    }
}




int main(int argc, char** argv) {
    if (argc < 4) {
        std::cerr << "Usage: " << argv[0] << " <left_camera_id> <right_camera_id> <lidar_port>" << std::endl;
        return - 1;
    }

    // Create UDP socket
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        perror("Socket creation failed");
        return -1;
    }


    // Parse camera IDs from command-line arguments
    int left_camera_id, right_camera_id;
    std::string lidar_port = argv[3];
    std::istringstream(argv[1]) >> left_camera_id;
    std::istringstream(argv[2]) >> right_camera_id;
    // std::string(argv[3]) >> lidar_port;

    StereoCamera stereoCam(left_camera_id, right_camera_id);

    LidarScanner lidarscan(lidar_port);

    // camera_record(stereoCam, vo, totalTranslation, totalRotation);
    std::cout << "SteraoCamera and Lidar Initialized " << std::endl;
    // Launching user input in a separate thread
    std::thread inputThread(listen_for_esc);
    std::cout << "input thread started" <<std::endl;
    // Start image-saving thread
    std::thread saveDataThread(sendData);
    // Start the camera recording threads
    std::thread cameraThread(camera_record, std::ref(stereoCam), sock);
    std::cout << "Camera thread started" <<std::endl;
    // Start the pcd recording threads
    std::thread lidarThread(lidar_record, std::ref(lidarscan), sock);
    std::cout << "Lidar thread started" <<std::endl;
    // Wait for the recording threads to finish
    lidarThread.join();
    cameraThread.join();
    saveDataThread.join();
    // Join the user input thread (this thread will wait for "stop" command)
    inputThread.join();

    

    std::cout<< "Recording complete"<<"\n";
    return 0;
}
