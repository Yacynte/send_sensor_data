// LiDAR.cpp
#include "LidarScanner.h"
#include "my_utils.h"


using namespace rp::standalone::rplidar;



LidarScanner::LidarScanner(const std::string& port)
    : port_(port), baudrate_(460800), lidar_(nullptr) {}

LidarScanner::~LidarScanner() {
    if (lidar_) {
        lidar_->stopMotor();
        lidar_->stop();
        lidar_->disconnect();
        RPlidarDriver::DisposeDriver(lidar_);
        lidar_ = nullptr;
    }
}


bool LidarScanner::initialize() {
    lidar_ = RPlidarDriver::CreateDriver();
    if (!lidar_) {
        std::cerr << "Failed to create RPLidar driver instance." << std::endl;
        return false;
    }

    if (IS_FAIL(lidar_->connect(port_.c_str(), baudrate_))) {
        std::cerr << "Failed to connect to RPLidar on " << port_ << std::endl;
        RPlidarDriver::DisposeDriver(lidar_);
        lidar_ = nullptr;
        return false;
    }
    // std::cout << "Attempting to start motor!" << std::endl;
    lidar_->startMotor();
    // std::cout << "Motor started successfully!" << std::endl;
    // std::cout << "Attempting to start scan!" << std::endl;
    lidar_->startScan(false, true);
    // std::cout << "Scan started successfully!" << std::endl;

    return true;
}

bool LidarScanner::getScans(cv::Mat& lidar_matrix, std::string& timestamp) {
    
    if (!lidar_) return false;

    rplidar_response_measurement_node_hq_t nodes[8192];
    size_t count = sizeof(nodes) / sizeof(nodes[0]);

    if (IS_FAIL(lidar_->grabScanDataHq(nodes, count))) {
        std::cerr << "Failed to grab scan data." << std::endl;
        lidar_->stop();
        lidar_->stopMotor();
        return false;
    }
    timestamp = getTimestamp();

    // Preallocate the matrix with max possible rows (count) and 2 columns (x, y)
    lidar_matrix.create(count, 2, CV_32F);
    int valid_points = 0;  // Track valid rows

    for (size_t i = 0; i < count; i++) {
        float angle = nodes[i].angle_z_q14 * 90.f / 16384.f;
        float distance = nodes[i].dist_mm_q2 / 4.0f;

        if (distance > 0) {
            float x = distance * cos(angle * M_PI / 180.0f);
            float y = distance * sin(angle * M_PI / 180.0f);

            lidar_matrix.at<float>(valid_points, 0) = x;
            lidar_matrix.at<float>(valid_points, 1) = y;
            valid_points++;  // Increment only for valid points
        }
    }
    // Trim unused rows
    lidar_matrix = lidar_matrix.rowRange(0, valid_points);

    if(lidar_matrix.empty()) return false;
    // std::cout << "Cloud size using .size(): " << cloud_->points.size() << std::endl;
    return true;
}

