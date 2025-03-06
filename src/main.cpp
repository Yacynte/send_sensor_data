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
            std::cout << "Esc key pressed. Interrupt signal sent!" << std::endl;
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);  // Restore old terminal settings
}

void sendData() {

    // Create UDP socket
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        perror("Socket creation failed");
        return ;
    }

    // Setup server address
    struct sockaddr_in server_addr;
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(UDP_PORT);
    server_addr.sin_addr.s_addr = inet_addr(UDP_IP);

    while (!interupt) {
        std::unique_lock<std::mutex> lock(imageMutex);
        imageCondVar.wait(lock, [] { return !imageQueue.empty() || stopImageSaving; });

        if (stopImageSaving && imageQueue.empty()) break;

        if (!imageQueue.empty()) {
            auto dataPair = imageQueue.front();
            imageQueue.pop();
            lock.unlock();

            auto first_pair = dataPair.first;
            auto second_pair = dataPair.second;

            cv::Mat image= first_pair.first;
            // int sock = first_pair.second;
            std::string timestamp = second_pair.first;
            std::string label = second_pair.second;

             // Encode the image matrix to .bin format (as a vector of bytes)
            std::vector<uchar> buffer;
            cv::imencode(".png", image, buffer);

            // Get the size of the Mat
            cv::Size matSize = image.size();
            int rows = matSize.height;
            int cols = matSize.width;

            std::string header = label + "|" + timestamp + "|" + std::to_string(rows) + "x" + std::to_string(cols) + "|";


            // Prepare the full data buffer
            size_t dataSize = header.size() + sizeof(size_t) + buffer.size();
            std::vector<uchar> fullData;
            
            // Reserve space for the total buffer size
            fullData.reserve(dataSize);
            size_t data_size = buffer.size();
            size_t data_size_header = header.size();
            // Append the header
            // fullData.insert(fullData.end(), header.begin(), header.end());

            // Append the data size (convert size_t to bytes)
            // fullData.insert(fullData.end(), reinterpret_cast<uchar*>(&data_size), reinterpret_cast<uchar*>(&data_size) + sizeof(size_t));

            // Append the image data
            // fullData.insert(fullData.end(), matSize.begin(), matSize.end());
            // fullData.insert(fullData.end(), reinterpret_cast<unsigned char*>(&matSize), reinterpret_cast<unsigned char*>(&matSize) + sizeof(cv::Size));
           
            // Send header data size first
            ssize_t sent_header_size = sendto(sock, &data_size_header, sizeof(data_size_header), 0,
                                    (struct sockaddr *)&server_addr, sizeof(server_addr));
            if (sent_header_size < 0) {
                perror("Error sending header size");
                close(sock);
                return ;
            }

            // Send the encoded header in chunks
            size_t sent_bytes_header = 0;
            ssize_t sent_size_header = sendto(sock, header.data(), data_size_header, 0,
                           (struct sockaddr *)&server_addr, sizeof(server_addr));

            if (sent_size_header < 0) {
                perror("Error sending header");
                close(sock);
                return;
            }

            
            // Send total data size first
            ssize_t sent_size = sendto(sock, &data_size, sizeof(data_size), 0,
                                    (struct sockaddr *)&server_addr, sizeof(server_addr));
            if (sent_size < 0) {
                perror("Error sending data size");
                close(sock);
                return ;
            }

            // Send the encoded data in chunks
            size_t sent_bytes = 0;
            while (sent_bytes < data_size) {
                size_t chunk_size = std::min(static_cast<size_t>(PACKET_SIZE), data_size - sent_bytes);
                sent_size = sendto(sock, buffer.data() + sent_bytes, chunk_size, 0,
                                (struct sockaddr *)&server_addr, sizeof(server_addr));
                if (sent_size < 0) {
                    perror("Error sending data chunk");
                    close(sock);
                    return;
                }
                sent_bytes += sent_size;
            }

            std::cout << "Sent " << sent_bytes << " bytes successfully\n";

            // ssize_t bytesSent = 0;
            // size_t totalSize = fullData.size();
            // const uchar* buffer = fullData.data();
            // std::cout << "Sending Data: | Data size: " << totalSize<< " bytes" << std::endl;
            
            // while (bytesSent < totalSize) {
            //     ssize_t sent = send(sock, buffer + bytesSent, totalSize - bytesSent, MSG_NOSIGNAL);
            //     std::cout << "bytes sent: "<< sent << " bytes" <<std::endl;
            //     if (sent == -1) {
            //         std::cerr << "Error sending Data" << std::endl;
            //         break;
            //     }
            //     bytesSent += sent;
            // }

            // if (bytesSent == totalSize) {
            //     std::cout << "Data sent successfully!" << std::endl;
            // } else {
            //     std::cerr << "Error: Only " << bytesSent << " of " << totalSize << " bytes sent!" << std::endl;
            // }

            // // cv::imwrite(imgPair.second, imgPair.first); // Save image
            // // std::cout << "Saved Image: " << imgPair.second << std::endl;

            // lock.lock();
        }
    }
    close(sock);
}


// void sendImageL(int sock, const std::string& timestamp, const cv::Mat& image, const std::string& label) {
//     // Encode the image matrix to .bin format (as a vector of bytes)
//     std::vector<uchar> buffer;
//     cv::imencode(".png", image, buffer);

//     // Get the size of the Mat
//     cv::Size matSize = image.size();
//     int rows = matSize.height;
//     int cols = matSize.width;

//     std::string header = label + "|" + timestamp + "|" + std::to_string(rows) + "x" + std::to_string(cols) + "|";


//     // Prepare the full data buffer
//     size_t dataSize = header.size() + sizeof(size_t) + buffer.size();
//     std::vector<uchar> fullData;
    
//     // Reserve space for the total buffer size
//     fullData.reserve(dataSize);
//     size_t data_size = buffer.size();
//     // Append the header
//     fullData.insert(fullData.end(), header.begin(), header.end());

//     // Append the data size (convert size_t to bytes)
//     fullData.insert(fullData.end(), reinterpret_cast<uchar*>(&data_size), reinterpret_cast<uchar*>(&data_size) + sizeof(size_t));

//     // Append the image data
//     fullData.insert(fullData.end(), data.begin(), data.end());

//     {
//         std::lock_guard<std::mutex> lock(imageMutex);
//         imageQueue.push({fullData, sock});
//     }
//     imageCondVar.notify_one(); // Notify saving thread
//     // std::cout << "notified queue Image L" << std::endl;
//     // Send the full data buffer in one go
//     // if (send(sock, fullData.data(), fullData.size(), MSG_NOSIGNAL) == -1){
//     //     std::cerr << "Error sending complete left Image" << std::endl;
//     //     return;
//     // }

//     // std::cout << "Image sent successfully!" << std::endl;
// }

// void sendImageR(int sock, const std::string& timestamp, const cv::Mat& image) {
//     // Similar to sendImageL
//     std::vector<uchar> data;
//     cv::imencode(".png", image, data);

//     // Get the size of the Mat
//     cv::Size matSize = image.size();
//     int rows = matSize.height;
//     int cols = matSize.width;

//     std::string header = "R|" + timestamp + "|" + std::to_string(rows) + "x" + std::to_string(cols) + "|";

//     size_t dataSize = header.size() + sizeof(size_t) + data.size();
//     std::vector<uchar> fullData;
//     fullData.reserve(dataSize);

//     size_t data_size = data.size();
//     fullData.insert(fullData.end(), header.begin(), header.end());
//     fullData.insert(fullData.end(), reinterpret_cast<uchar*>(&data_size), reinterpret_cast<uchar*>(&data_size) + sizeof(size_t));
//     fullData.insert(fullData.end(), data.begin(), data.end());

//     {
//         std::lock_guard<std::mutex> lock(imageMutex);
//         imageQueue.push({fullData, sock});
//     }
//     imageCondVar.notify_one(); // Notify saving thread
//     // std::cout << "notified queue Image R" << std::endl;
//     // if (send(sock, fullData.data(), fullData.size(), MSG_NOSIGNAL) == -1) {
//     //     std::cerr << "Error sending complete Right Image" << std::endl;
//     //     return;
//     // }

//     // std::cout << "Image sent successfully!" << std::endl;
// }

// void sendLidar(int sock, const std::string& timestamp, const cv::Mat& lidar_matrix) {
//     // Encode the lidar matrix to .bin format (as a vector of bytes)
//     std::vector<uchar> data;
//     cv::imencode(".png", lidar_matrix, data);

//     // Get the size of the Mat
//     cv::Size matSize = lidar_matrix.size();
//     int rows = matSize.height;
//     int cols = matSize.width;

//     std::string header = "D|" + timestamp + "|" + std::to_string(rows) + "x" + std::to_string(cols) + "|";

//     size_t dataSize = header.size() + sizeof(size_t) + data.size();
//     std::vector<uchar> fullData;
//     fullData.reserve(dataSize);
//     size_t data_size = data.size();
//     fullData.insert(fullData.end(), header.begin(), header.end());
//     fullData.insert(fullData.end(), reinterpret_cast<uchar*>(&data_size), reinterpret_cast<uchar*>(&data_size) + sizeof(size_t));
//     fullData.insert(fullData.end(), data.begin(), data.end());

//     {
//         std::lock_guard<std::mutex> lock(imageMutex);
//         imageQueue.push({fullData, sock});
//     }
//     imageCondVar.notify_one(); // Notify saving thread
//     // std::cout << "notified queue lidar" << std::endl;
//     // if (send(sock, fullData.data(), fullData.size(), MSG_NOSIGNAL) == -1) {
//     //     std::cerr << "Error sending complete Lidar data" << std::endl;
//     //     return;
//     // }

//     // std::cout << "Lidar data sent successfully!" << std::endl;
// }


void camera_record(StereoCamera& stereoCam, int sock){
    // StereoCamera stereoCam(0, 2); // Adjust IDs based on your setup

    std::cout << "In camera thread " <<std::endl;
    auto now = std::chrono::steady_clock::now();
    int frameCounter = 0;

    cv::Mat leftFrame, rightFrame;
    // Define a vector to store timestamps
    
    while (!interupt) {
        // cv::Mat leftFrame, rightFrame;
        auto currentTime = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - now).count();
        if (elapsed >= 40) { // 1000 ms / 20 FPS = 50 ms
            // Capture stereo frames
            std::string timestamp;
            if (stereoCam.captureFrames(leftFrame, rightFrame, timestamp)) {
                auto now = std::chrono::steady_clock::now();
                // Get current time in seconds with microsecond precision
                
                {
                    std::lock_guard<std::mutex> lock(imageMutex);
                    imageQueue.push({{leftFrame, sock}, {timestamp, "L"} });
                }
                imageCondVar.notify_one(); // Notify saving thread

                {
                    std::lock_guard<std::mutex> lock(imageMutex);
                    imageQueue.push({{rightFrame, sock}, {timestamp, "R"} });
                }
                imageCondVar.notify_one(); // Notify saving thread

                // std::thread leftThread(sendImageL, sock, timestamp, leftFrame);
                // std::thread rightThread(sendImageR, sock, timestamp, rightFrame);
                           
                // leftThread.join();
                // rightThread.join();
            }        
        }
    }
}


    
void lidar_record(LidarScanner& lidarscan, int sock){

    if (!lidarscan.initialize()) {
        std::cerr << "RPLIDAR C1 initialization failed!" << std::endl;
        return ;
    }
    std::cout << "in lidar thread" <<std::endl;
    auto now = std::chrono::steady_clock::now();
    // auto start = std::chrono::steady_clock::now();

    // Initialize point cloud pointers
    cv::Mat scans_cur;

    while (!interupt) {
        auto currentTime = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - now).count();
        
        if (elapsed >= 50) { // 1000 ms / 20 FPS = 50 ms
            std::string timestamp;
            
            if (lidarscan.getScans(scans_cur, timestamp)) {
                auto now = std::chrono::steady_clock::now();
                
                // sendLidar(sock, timestamp, scans_cur);
                {
                    std::lock_guard<std::mutex> lock(imageMutex);
                    imageQueue.push({{scans_cur, sock}, {timestamp, "D"} });
                }
                imageCondVar.notify_one(); // Notify saving thread

            }
            else {
                std::cerr << "Failed to obtain lidar scans" << std::endl;
                interupt = true;
                break;
            } 

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

    // Setup server address
    // struct sockaddr_in server_addr;
    // server_addr.sin_family = AF_INET;
    // server_addr.sin_port = htons(UDP_PORT);
    // server_addr.sin_addr.s_addr = inet_addr(UDP_IP);


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

    
    

    

    // std::cout<<totalTranslation<<std::endl;
    // std::thread t1(camera_record, stereoCam, vo, &totalTranslation, &totalRotation);
    // std::thread t2(lidar_record, lidar);

    // t1.join();
    // t2.join();
    std::cout<< "Recording complete"<<"\n";
    return 0;
}
