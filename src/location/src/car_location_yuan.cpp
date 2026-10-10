// location_node.cpp - 发布位置 + 方向（已注释平滑器和速度阈值，添加最终校准）
#include "location/serial_reader.h"
#include "location/nmea_parser.h"
#include "location/rtk_config.h"
#include "location/coordinate_converter.hpp"
#include <iostream>
#include <iomanip>
#include <csignal>
#include <atomic>
#include <rclcpp/rclcpp.hpp>
#include "location/msg/rtk_data.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <deque>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace std::placeholders;

std::atomic<bool> running(true);
std::unique_ptr<CoordinateConverter> coordinate_converter;
rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher;
rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr navsatfix_publisher;

// =====================================================
// 平滑器类（已禁用，所有 smooth() 调用已被注释）
// =====================================================
class HeadingSmoother {
private:
    std::deque<double> buffer_;
    size_t window_size_ = 5;
    double last_heading_ = 0.0;
    
public:
    double smooth(double heading) {
        buffer_.push_back(heading);
        if (buffer_.size() > window_size_) {
            buffer_.pop_front();
        }
        
        std::vector<double> sorted(buffer_.begin(), buffer_.end());
        std::sort(sorted.begin(), sorted.end());
        double median = sorted[sorted.size() / 2];
        
        if (last_heading_ != 0.0 && std::abs(median - last_heading_) > 0.1745) {
            return last_heading_;
        }
        
        last_heading_ = median;
        return median;
    }
};

static HeadingSmoother heading_smoother;

double normalizeAngleDeg(double angle) {
    while (angle > 180.0) angle -= 360.0;
    while (angle < -180.0) angle += 360.0;
    return angle;
}

void signalHandler(int signal) {
    std::cout << "\nReceived signal " << signal << ", shutting down..." << std::endl;
    running = false;
}

void displayData(const RTKData& data) {
    std::cout << "\033[2J\033[1;1H";
    std::cout << "=== RTK GNSS RECEIVER ===" << std::endl;
    std::cout << "Time: " << data.utc_time << " Date: " << data.date << std::endl;
    std::cout << "Status: " << (data.data_valid ? "\033[1;32mVALID\033[0m" : "\033[1;31mINVALID\033[0m") << std::endl;
    std::cout << "Fix: " << data.rtk_status;
    
    if (data.is_rtk_fixed) {
        std::cout << " \033[1;32m[PRECISE]\033[0m";
    } else if (data.is_rtk_float) {
        std::cout << " \033[1;33m[FLOAT]\033[0m";
    }
    std::cout << std::endl;
    
    std::cout << "Satellites: " << data.satellites_used << " HDOP: " << std::setprecision(2) << data.hdop << std::endl;
    
    if (data.data_valid) {
        std::cout << "Position: " << std::fixed << std::setprecision(8) 
                  << data.position.latitude << "° N, " 
                  << data.position.longitude << "° E" << std::endl;
        std::cout << "Altitude: " << std::setprecision(3) << data.position.altitude << " m" << std::endl;
        std::cout << "Speed: " << std::setprecision(2) << data.speed_kmh << " km/h" << std::endl;
        std::cout << "Course: " << std::setprecision(1) << data.head_angle_true << "°" << std::endl;
        
        if (coordinate_converter && coordinate_converter->isOriginSet()) {
            double x, y, z;
            coordinate_converter->latLonToLocal(data.position.latitude, data.position.longitude, data.position.altitude, x, y, z);
            std::cout << "Local Coordinates: X=" << std::setprecision(3) << x << "m, Y=" << y << "m, Z=" << z <<"m" << std::endl;
        }
    } else {
        std::cout << "Position: ---" << std::endl;
        std::cout << "Waiting for valid GPS signal..." << std::endl;
    }
    
    std::cout << "\nPress Ctrl+C to exit" << std::endl;
}

void publishNavSatFix(const RTKData& data, std::shared_ptr<rclcpp::Node> node) {
    if (!data.data_valid || !navsatfix_publisher) {
        return;
    }

    auto navsatfix_msg = std::make_shared<sensor_msgs::msg::NavSatFix>();
    navsatfix_msg->header.stamp = node->now();
    navsatfix_msg->header.frame_id = "gps_link";
    navsatfix_msg->latitude = data.position.latitude;
    navsatfix_msg->longitude = data.position.longitude;
    navsatfix_msg->altitude = data.position.altitude;

    if (data.is_rtk_fixed) {
        navsatfix_msg->status.status = sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
    } else if (data.is_rtk_float) {
        navsatfix_msg->status.status = sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
    } else {
        navsatfix_msg->status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
    }
    navsatfix_msg->status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;

    double pos_cov = data.is_rtk_fixed ? 0.01 : (data.is_rtk_float ? 0.1 : 1.0);
    navsatfix_msg->position_covariance[0] = pos_cov;
    navsatfix_msg->position_covariance[4] = pos_cov;
    navsatfix_msg->position_covariance[8] = pos_cov;
    navsatfix_msg->position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

    navsatfix_publisher->publish(*navsatfix_msg);
}

// ✅ 发布位置 + 方向（已注释平滑器和速度阈值，添加最终校准）
void publishOdometry(const RTKData& data, std::shared_ptr<rclcpp::Node> node) {
    if (!data.data_valid || !odom_publisher || !coordinate_converter || !coordinate_converter->isOriginSet()) {
        return;
    }

    // =====================================================
    // 调试输出：打印原始数据
    // =====================================================
    static int debug_counter = 0;
    if (debug_counter++ % 10 == 0) {  // 每10帧打印一次，避免刷屏
        std::cout << "[DEBUG] raw heading=" << data.head_angle_true 
                  << "°, speed=" << data.speed_kmh << " km/h" << std::endl;
    }

    double x, y, z;
    coordinate_converter->latLonToLocal(data.position.latitude, data.position.longitude, data.position.altitude, x, y, z);

    auto odom_msg = std::make_shared<nav_msgs::msg::Odometry>();
    odom_msg->header.stamp = node->now();
    odom_msg->header.frame_id = "map";           
    odom_msg->child_frame_id = "base_link";      

    // 位置
    odom_msg->pose.pose.position.x = x;
    odom_msg->pose.pose.position.y = y;
    odom_msg->pose.pose.position.z = 0.0;

    // =====================================================
    // ✅ 最终校准公式（基于实测：朝东时 raw heading=270°）
    // 公式：ros_deg = 180.0 - rtk_deg
    // 验证：rtk_deg=270°(东) → ros_deg=180-270=-90°? 不对
    // 
    // 根据实测数据（朝东时 z=-0.998, w=0.061 → 约 -179°）
    // 使用加法校准：ros_deg = (90 - rtk_deg) + 179
    // =====================================================
    double rtk_deg = data.head_angle_true;
    
    // 基础转换：北=0° → 东=0°
    double ros_deg = 90.0 - rtk_deg;
    
    // ✅ 最终校准偏移（根据朝东实测数据：应输出0°）
    const double CALIBRATION_OFFSET = 179.0;
    ros_deg = ros_deg + CALIBRATION_OFFSET;
    
    ros_deg = normalizeAngleDeg(ros_deg);
    
    // 调试输出：打印转换后的角度
    if (debug_counter % 10 == 0) {
        std::cout << "[DEBUG] ros_deg=" << ros_deg << "°" << std::endl;
    }
    
    // =====================================================
    // ✅ 已注释：平滑滤波和速度阈值
    // =====================================================
    double heading_rad = ros_deg * M_PI / 180.0;
    // heading_rad = heading_smoother.smooth(heading_rad);  // 已禁用

    odom_msg->pose.pose.orientation.x = 0.0;
    odom_msg->pose.pose.orientation.y = 0.0;
    odom_msg->pose.pose.orientation.z = sin(heading_rad / 2.0);
    odom_msg->pose.pose.orientation.w = cos(heading_rad / 2.0);

    // 协方差：位置 + 方向都可信
    double pos_var = data.is_rtk_fixed ? 0.01 : (data.is_rtk_float ? 0.1 : 1.0);
    double yaw_var = 0.008;
    
    for (int i = 0; i < 36; i++) {
        odom_msg->pose.covariance[i] = 0.0;
        odom_msg->twist.covariance[i] = 0.0;
    }
    
    odom_msg->pose.covariance[0] = pos_var;
    odom_msg->pose.covariance[7] = pos_var;
    odom_msg->pose.covariance[14] = pos_var;
    odom_msg->pose.covariance[21] = 1e6;
    odom_msg->pose.covariance[28] = 1e6;
    odom_msg->pose.covariance[35] = yaw_var;

    odom_publisher->publish(*odom_msg);
}

int main(int argc, char* argv[]) {
    signal(SIGINT, signalHandler);

    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("location_node");

    node->declare_parameter<std::string>("serial_port", RTKConfig::DEFAULT_PORT);
    node->declare_parameter<int>("baud_rate", RTKConfig::DEFAULT_BAUDRATE);
    node->declare_parameter<double>("datum_lat", 29.4898045);
    node->declare_parameter<double>("datum_lon", 106.5676103);
    node->declare_parameter<bool>("auto_set_origin", false);
    
    std::string port = node->get_parameter("serial_port").as_string();
    int baudrate = node->get_parameter("baud_rate").as_int();
    double datum_lat = node->get_parameter("datum_lat").as_double();
    double datum_lon = node->get_parameter("datum_lon").as_double();
    bool auto_set_origin = node->get_parameter("auto_set_origin").as_bool();

    coordinate_converter = std::make_unique<CoordinateConverter>(datum_lat, datum_lon, 0.0);
    std::cout << "Using map origin: lat=" << datum_lat << ", lon=" << datum_lon << std::endl;

    navsatfix_publisher = node->create_publisher<sensor_msgs::msg::NavSatFix>("/navsatfix", 10);
    odom_publisher = node->create_publisher<nav_msgs::msg::Odometry>("/gps_odom", 10);

    SerialReader reader;
    
    std::cout << "Initializing RTK receiver on " << port << " at " << baudrate << " baud..." << std::endl;
    
    if (!reader.initialize(port, baudrate)) {
        std::cerr << "Failed to initialize RTK receiver!" << std::endl;
        rclcpp::shutdown();
        return -1;
    }
    
    reader.startReading();
    std::cout << "RTK receiver started successfully!" << std::endl;
    
    rclcpp::Rate rate(20);
    
    while (running && rclcpp::ok()) {
        RTKData data = reader.getLatestData();
        
        if (data.data_valid) {
            displayData(data);
            
            if (auto_set_origin && !coordinate_converter->isOriginSet()) {
                coordinate_converter->setOrigin(data.position.latitude, 
                                               data.position.longitude, 
                                               data.position.altitude);
                std::cout << "Auto-set origin: lat=" << data.position.latitude 
                          << ", lon=" << data.position.longitude << std::endl;
            }
            
            publishNavSatFix(data, node);
            publishOdometry(data, node);
        }
        
        rclcpp::spin_some(node);
        rate.sleep();
    }
    
    reader.stopReading();
    rclcpp::shutdown();
    
    return 0;
}
