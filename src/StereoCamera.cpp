// StereoCamera.cpp
#include "StereoCamera.h"
#include "my_utils.h"
#include <thread>

// Constructor: Open both left and right cameras
StereoCamera::StereoCamera(int leftCamID, int rightCamID) {
    leftCamIDp = leftCamID;
    rightCamIDp = rightCamID;
    leftCam.open(leftCamID, cv::CAP_V4L2);

    if (rightCamID == 100) {
        // Set resolution to 3840x1080 (side-by-side stereo)
        leftCam.set(cv::CAP_PROP_FRAME_WIDTH, 3840);
        leftCam.set(cv::CAP_PROP_FRAME_HEIGHT, 1080);
    } else {
        rightCam.open(rightCamID, cv::CAP_V4L2);
    }

    if (!checkCameras()) {
        std::cerr << "Failed to open one or both cameras." << std::endl;
    }
}

// Destructor: Release camera resources
StereoCamera::~StereoCamera() {
    leftCam.release();
    if (rightCamIDp != 100) {
        rightCam.release();
    }
}

std::string StereoCamera::splitStereoImage(cv::Mat &leftFrame, cv::Mat &rightFrame) {
    cv::Mat frame;
    leftCam.read(frame);  // Capture a frame
    std::string timestamp = getTimestamp();
    
    if (!frame.empty()) {
        int width = frame.cols / 2, height = frame.rows;
        leftFrame.create(480, 640, frame.type());
        rightFrame.create(480, 640, frame.type());
        
        cv::resize(frame(cv::Rect(0, 0, width, height)), leftFrame, leftFrame.size());
        cv::resize(frame(cv::Rect(width, 0, width, height)), rightFrame, rightFrame.size());
    } else {
        std::cerr << "Error: Captured frame is empty!" << std::endl;
        return "Error";
    }
    return timestamp;
}

bool StereoCamera::captureFrames(cv::Mat& leftFrame, cv::Mat& rightFrame, std::string& timestamp) {
    if (rightCamIDp == 100) {
        timestamp = splitStereoImage(leftFrame, rightFrame);
        return true;
    }
    
    if (!checkCameras()) {
        return false;
    }
    
    std::thread leftThread([&] { leftCam.read(leftFrame); });
    std::thread rightThread([&] { rightCam.read(rightFrame); });
    
    leftThread.join();
    rightThread.join();
    
    if (leftFrame.empty() || rightFrame.empty()) {
        std::cerr << "Error: Failed to capture frames." << std::endl;
        return false;
    }
    
    timestamp = getTimestamp();
    RectifyImage(leftFrame, rightFrame);
    return true;
}

bool StereoCamera::checkCameras() {
    return leftCam.isOpened() && (rightCamIDp == 100 || rightCam.isOpened());
}

void StereoCamera::RectifyImage(cv::Mat& leftImage, cv::Mat& rightImage) {
    cv::Mat R1, R2, P1, P2, Q;
    cv::stereoRectify(K1, D1, K2, D2, leftImage.size(), R, T, R1, R2, P1, P2, Q);

    cv::Mat map1x, map1y, map2x, map2y;
    cv::initUndistortRectifyMap(K1, D1, R1, P1, leftImage.size(), CV_32FC1, map1x, map1y);
    cv::initUndistortRectifyMap(K2, D2, R2, P2, rightImage.size(), CV_32FC1, map2x, map2y);

    cv::remap(leftImage, leftImage, map1x, map1y, cv::INTER_LINEAR);
    cv::remap(rightImage, rightImage, map2x, map2y, cv::INTER_LINEAR);
}
