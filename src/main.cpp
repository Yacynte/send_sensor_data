#include <iostream>
#include <thread>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <termios.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <chrono>
#include <opencv2/opencv.hpp>
#include "StereoCamera.h"
#include "LidarScanner.h"
#include <atomic>

namespace data_capture {

constexpr const char* TCP_IP = "192.168.178.28";
constexpr int TCP_PORT = 5005;

std::atomic<bool> interrupt(false);
std::queue<std::pair<std::pair<cv::Mat, int>, std::pair<std::string, std::string>>> imageQueue;
std::mutex imageMutex;
std::condition_variable imageCondVar;
bool stopImageSaving = false;

// Function to set terminal to non-canonical mode
void setTerminalNonCanonical() {
    struct termios oldt, newt;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
}

// Function to restore terminal settings
void restoreTerminalSettings() {
    struct termios oldt;
    tcgetattr(STDIN_FILENO, &oldt);
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
}

void listenForEsc() {
    setTerminalNonCanonical();
    while (!interrupt) {
        char ch = getchar();
        if (ch == 27) {
            interrupt = true;
            stopImageSaving = true;
            std::cout << "Esc key pressed. Interrupt signal sent!" << std::endl;
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    restoreTerminalSettings();
}

int createAndConnectSocket() {
    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) {
        perror("Socket creation failed");
        return -1;
    }

    struct sockaddr_in server_addr;
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(TCP_PORT);
    if (inet_pton(AF_INET, TCP_IP, &server_addr.sin_addr) <= 0) {
        perror("Invalid address / Address not supported");
        close(sock);
        return -1;
    }

    if (connect(sock, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        perror("Connection to server failed");
        close(sock);
        return -1;
    }
    std::cout << "Connection established\n";
    return sock;
}

void sendData() {
    int sock = createAndConnectSocket();
    if (sock == -1) return;

    while (!interrupt) {
        std::unique_lock<std::mutex> lock(imageMutex);
        imageCondVar.wait(lock, [] { return !imageQueue.empty() || stopImageSaving; });
        if (stopImageSaving && imageQueue.empty()) break;
        if (!imageQueue.empty()) {
            auto dataPair = imageQueue.front();
            imageQueue.pop();
            lock.unlock();

            auto& [image, socket] = dataPair.first;
            auto& [timestamp, label] = dataPair.second;

            std::vector<uchar> buffer;
            if (label == "D") {
                buffer.assign(image.data, image.data + image.total() * image.elemSize());
            } else {
                cv::imencode(".jpg", image, buffer);
            }
            size_t image_size = buffer.size();

            std::string header = label + "|" + timestamp + "|" + std::to_string(image.rows) + "x" + std::to_string(image.cols) + "|";
            size_t header_size = header.size();

            if (send(sock, &header_size, sizeof(header_size), 0) < 0) {
                perror("Failed to send header size");
                interrupt = true;
                break;
            }
            if (send(sock, header.data(), header_size, 0) < 0) {
                perror("Failed to send header");
                interrupt = true;
                break;
            }
            if (send(sock, &image_size, sizeof(image_size), 0) < 0) {
                perror("Failed to send image size");
                interrupt = true;
                break;
            }
            if (send(sock, buffer.data(), image_size, 0) < 0) {
                perror("Failed to send image data");
                interrupt = true;
                break;
            }
        }
    }
    close(sock);
}

// ... (listenForEsc, setTerminalNonCanonical, restoreTerminalSettings, createAndConnectSocket, sendData functions) ...

void cameraRecord(StereoCamera& stereoCam, int sock) {
    auto lastCaptureTime = std::chrono::steady_clock::now();
    cv::Mat leftFrame, rightFrame;

    while (!interrupt) {
        auto currentTime = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - lastCaptureTime).count() >= 40) {
            std::string timestamp;

            if (stereoCam.captureFrames(leftFrame, rightFrame, timestamp)) {
                lastCaptureTime = std::chrono::steady_clock::now();
                {
                    std::lock_guard<std::mutex> lock(imageMutex);
                    imageQueue.emplace(std::make_pair(leftFrame, sock), std::make_pair(timestamp, "L"));
                    imageQueue.emplace(std::make_pair(rightFrame, sock), std::make_pair(timestamp, "R"));
                }
                imageCondVar.notify_all();
            } else {
                std::cerr << "Failed to obtain camera frames" << std::endl;
                interrupt = true;
                break;
            }
        }
    }
}

void lidarRecord(LidarScanner& lidarscan, int sock) {
    if (!lidarscan.initialize()) {
        std::cerr << "RPLIDAR C1 initialization failed!" << std::endl;
        return;
    }
    auto lastCaptureTime = std::chrono::steady_clock::now();
    cv::Mat scansCur;

    while (!interrupt) {
        auto currentTime = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - lastCaptureTime).count() >= 25) {
            std::string timestamp;

            if (lidarscan.getScans(scansCur, timestamp)) {
                lastCaptureTime = std::chrono::steady_clock::now();
                {
                    std::lock_guard<std::mutex> lock(imageMutex);
                    imageQueue.push({{scansCur, sock}, {timestamp, "D"}});
                }
                imageCondVar.notify_one();
            } else {
                std::cerr << "Failed to obtain LiDAR scans" << std::endl;
                interrupt = true;
                break;
            }
        }
    }
}

//... camera_record and lidar_record functions here.
} // namespace data_capture

int main(int argc, char** argv) {
    if (argc < 4) {
        std::cerr << "Usage: " << argv[0] << " <left_camera_id> <right_camera_id> <lidar_port>" << std::endl;
        return -1;
    }

    int leftCameraId, rightCameraId;
    std::string lidarPort = argv[3];
    std::istringstream(argv[1]) >> leftCameraId;
    std::istringstream(argv[2]) >> rightCameraId;

    StereoCamera stereoCam(leftCameraId, rightCameraId);
    LidarScanner lidarscan(lidarPort);
    std::cout << "StereoCamera and Lidar Initialized" << std::endl;

    std::thread inputThread(data_capture::listenForEsc);
    std::thread saveDataThread(data_capture::sendData);
    std::thread cameraThread(data_capture::cameraRecord, std::ref(stereoCam), 0);
    std::thread lidarThread(data_capture::lidarRecord, std::ref(lidarscan), 0);

    lidarThread.join();
    cameraThread.join();
    saveDataThread.join();
    inputThread.join();

    std::cout << "Recording complete" << std::endl;
    return 0;
}