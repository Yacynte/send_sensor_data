// LiDAR.cpp
#include "LidarScanner.h"
#include "my_utils.h"

#include <unistd.h>
static inline void delay(sl_word_size_t ms){
    while (ms>=1000){
        usleep(1000*1000);
        ms-=1000;
    };
    if (ms!=0)
        usleep(ms*1000);
}

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

using namespace rp::standalone::rplidar;



LidarScanner::LidarScanner(const std::string& port)
    : port_(port), baudrate_(460800), lidar_(nullptr) {}

LidarScanner::~LidarScanner() {
    if (lidar_) {
        // lidar_->stopMotor();
        lidar_->stop();
        delay(200);
        // lidar_->setMotorSpeed(0);
        lidar_->setMotorPWM(0);
        lidar_->disconnect();
        // delete lidar_;
        // lidar_ = NULL;
        RPlidarDriver::DisposeDriver(lidar_);
        lidar_ = nullptr;
    }
}


bool LidarScanner::initialize() {
    lidar_ = RPlidarDriver::CreateDriver();
    // create the driver instance
	// sl::ILidarDriver * lidar_ = *createLidarDriver();
    if (!lidar_) {
        std::cerr << "Failed to create RPLidar driver instance." << std::endl;
        return false;
    }
    // _channel = (*createSerialPortChannel(port_, baudrate_));
    if (IS_FAIL(lidar_->connect(port_.c_str(), baudrate_))) {
    // if (SL_IS_FAIL(lidar_->connect(_channel))){
        std::cerr << "Failed to connect to RPLidar on " << port_ << std::endl;
        RPlidarDriver::DisposeDriver(lidar_);
        lidar_ = nullptr;
        // delete lidar_;
        // lidar_ = NULL;
        return false;
    }
    // std::cout << "Attempting to start motor!" << std::endl;
    // lidar_->startMotor();
    lidar_->setMotorPWM(600);
    // lidar_->setMotorSpeed(); // Set to 500 RPM (example)
    // std::cout << "Motor started successfully!" << std::endl;
    // std::cout << "Attempting to start scan!" << std::endl;
    // lidar_->startScanExpress(false);
    lidar_->startScan(false, true);
    // std::cout << "Scan started successfully!" << std::endl;

    return true;
}

bool LidarScanner::getScans(cv::Mat& lidar_matrix, std::string& timestamp) {
    // std::cout << "In\n";
    // if (!lidar_) return false;
    if (!lidar_) {
        fprintf(stderr, "insufficent memory, exit\n");
        return false;
    }
    // std::cout << "In and init\n";
    // lidar_->startScan(false, true);
    // delay(3000);
    rplidar_response_measurement_node_hq_t nodes[8192];
    // size_t count = sizeof(nodes) / sizeof(nodes[0]);
    size_t   count = _countof(nodes);

    if (IS_FAIL(lidar_->grabScanDataHq(nodes, count))) {
        std::cerr << "Failed to grab scan data." << std::endl;
        lidar_->stop();
        // delay(200);
        lidar_->setMotorPWM(0);
        // lidar_->stopMotor();
        return false;
    }
    timestamp = getTimestamp();
    // lidar_->ascendScanData(nodes, count);
    // Preallocate the matrix with max possible rows (count) and 2 columns (x, y)
    lidar_matrix.create(count, 2, CV_32F);
    int valid_points = 0;  // Track valid rows

    for (int i = 0; i < (int) count; ++i) {
        float angle = nodes[i].angle_z_q14 * 90.f / 16384.f;
        float distance = nodes[i].dist_mm_q2 / 4.0f;

        if (distance > 0) {
            float x = distance * cos(angle * M_PI / 180.0f);
            float y = distance * sin(angle * M_PI / 180.0f);

            lidar_matrix.at<float>(valid_points, 0) = x;
            lidar_matrix.at<float>(valid_points, 1) = y;
            // std::cout << "x: " << x << " y: " << y <<std::endl;
            valid_points++;  // Increment only for valid points
        }
    }
    // Trim unused rows
    lidar_matrix = lidar_matrix.rowRange(0, valid_points);
    // std::cout << " pcd size: " << lidar_matrix.size() << std::endl;
    if(lidar_matrix.empty()) return false;
    // std::cout << "Cloud size using .size(): " << cloud_->points.size() << std::endl;
    return true;
}

