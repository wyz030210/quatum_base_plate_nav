// location_node.cpp - 融合绝对航向（HDT） + 天线偏移补偿
//头文件引入
#include "location/serial_reader.h"            //串口读取
#include "location/nmea_parser.h"              //NMEA 解析
#include "location/rtk_config.h"               //RTK 配置模块
#include "location/coordinate_converter.hpp"   //经纬度 ↔ 局部直角坐标转换
#include <iostream>
#include <iomanip>
#include <csignal>
#include <atomic>
#include <rclcpp/rclcpp.hpp>                   //ROS2 C++ 客户端库
#include "location/msg/rtk_data.hpp"
#include <nav_msgs/msg/odometry.hpp>           //里程计消息类型（发布位置 + 姿态）
#include <sensor_msgs/msg/nav_sat_fix.hpp>     //GPS 原始定位消息类型
#include <deque>
#include <algorithm>
#include <sstream>
#include <unistd.h>
#include <std_msgs/msg/string.hpp>

//宏定义(确保 M_PI 有定义。不同平台/编译器的 <cmath> 中 M_PI 不一定存在，这里手动兜底。)
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace std::placeholders;

//全局变量
std::atomic<bool> running(true);                                                 //原子布尔标志，控制主循环是否继续。被信号处理函数设置为 false 来优雅退出
std::unique_ptr<CoordinateConverter> coordinate_converter;                       //坐标转换器（GPS 经纬度 → 局部 XY 坐标）
rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher;            //发布 /odometry/gps 话题
rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr navsatfix_publisher;   //发布 /gps/fix 话题

// Independent raw measurement provenance. Legacy fix/pose transport is retained.
std::string sourceJson(const RtkSourceState & state, const std::string & instance,
                       uint64_t status_seq, const RtkSourceState & used,
                       double datum_lat, double datum_lon) {
    std::ostringstream out;
    out << std::boolalpha << std::setprecision(15)
        << "{\"schema_version\":1,\"driver_instance_id\":\"" << instance
        << "\",\"status_seq\":" << status_seq
        << ",\"connection_generation\":" << state.connection_generation
        << ",\"discontinuity_sequence\":" << state.discontinuity_sequence
        << ",\"connected\":" << state.connected << ",\"ready\":" << state.ready
        << ",\"frame_id\":\"map\",\"base_frame_id\":\"base_link\""
        << ",\"datum_lat\":" << datum_lat << ",\"datum_lon\":" << datum_lon
        << ",\"calibration_id\":\"east_clockwise_ant0.319\""
        << ",\"published_position_seq\":" << used.position.sequence
        << ",\"published_quality_seq\":" << used.quality.sequence
        << ",\"published_heading_seq\":" << used.heading.sequence
        << ",\"published_generation\":" << used.connection_generation;
    const auto source = [&out](const char * name, const SourceMeasurement & value) {
        out << ",\"" << name << "\":{\"sequence\":" << value.sequence
            << ",\"epoch_us\":" << value.epoch_us << ",\"valid\":" << value.valid
            << ",\"age_sec\":";
        if (value.age_sec < 0) out << "null"; else out << value.age_sec;
        out << ",\"reason\":\"" << value.reason << "\"}";
    };
    for (const auto & entry : {std::make_pair("position", &used.position),
            std::make_pair("quality", &used.quality), std::make_pair("heading", &used.heading)}) {
        out << ",\"published_" << entry.first << "_age_sec\":";
        if (!entry.second->sequence) out << "null";
        else out << std::chrono::duration<double>(
            std::chrono::steady_clock::now() - entry.second->received).count();
    }
    source("position", state.position);
    source("quality", state.quality);
    source("heading", state.heading);
    out << "}";
    return out.str();
}

//将角度归一化到 [-180°, 180°] 范围内。
double normalizeAngleDeg(double angle) {
    while (angle > 180.0) angle -= 360.0;
    while (angle < -180.0) angle += 360.0;
    return angle;
}

//将角度归一化到 [0°, 360°) 范围内
double normalizeAngleDeg360(double angle) {
    while (angle >= 360.0) angle -= 360.0;
    while (angle < 0.0) angle += 360.0;
    return angle;
}

//SIGNAT（Ctrl+C）信号处理函数。将 running 设为 false，让主循环优雅退出。
void signalHandler(int signal) {
    std::cout << "\nReceived signal " << signal << ", shutting down..." << std::endl;
    running = false;
}

//终端实时显示 RTK 数据。
//用 ANSI 转义码清屏重绘，实现"仪表盘"效果
//显示：时间、定位状态（绿色 VALID / 红色 INVALID）、RTK 解算类型（PRECISE/FLOAT）、卫星数、HDOP、经纬度、速度、航向
//如果坐标转换器已设置原点，还会显示局部坐标系下的 XY 坐标
void displayData(const RTKData& data) {
    std::cout << "\033[2J\033[1;1H";
    std::cout << "=== RTK GNSS RECEIVER ===" << std::endl;
    std::cout << "Time: " << data.utc_time << " Date: " << data.date << std::endl;
    std::cout << "Status: " << (data.data_valid ? "\033[1;32mVALID\033[0m" : "\033[1;31mINVALID\033[0m") << std::endl;
    std::cout << "Fix: " << data.rtk_status;
    std::cout << " (GGA quality=" << data.fix_quality << ")";
    
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
        
        // 显示设备原始HDT
        std::cout << "Device HDT: " << std::setprecision(1) << data.head_angle_true << "°" << std::endl;
        
        // 计算并显示标准HDT和ROS Yaw
        // 设备定义: 0°=东, 顺时针增加
        // 标准定义: 0°=北, 顺时针增加
        double standard_hdt = normalizeAngleDeg360(data.head_angle_true + 90.0);
        double ros_yaw_deg = 90.0 - standard_hdt;
        ros_yaw_deg = normalizeAngleDeg(ros_yaw_deg);
        
        std::cout << "Standard HDT: " << std::setprecision(1) << standard_hdt << "°" << std::endl;
        std::cout << "ROS Yaw: " << std::setprecision(1) << ros_yaw_deg << "°" << std::endl;
        
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

//发布 RTK 原始经纬度到 /gps/fix 话题。
void publishNavSatFix(const RTKData& data, std::shared_ptr<rclcpp::Node> node) {
    if (!data.data_valid || !navsatfix_publisher) {
        return;
    }

    auto navsatfix_msg = std::make_shared<sensor_msgs::msg::NavSatFix>();
    navsatfix_msg->header.stamp = node->now();
    navsatfix_msg->header.frame_id = "gps_link";                                            //这里是天线的坐标系，不是车体
    navsatfix_msg->latitude = data.position.latitude;
    navsatfix_msg->longitude = data.position.longitude;
    navsatfix_msg->altitude = data.position.altitude;

    if (data.is_rtk_fixed) {
        navsatfix_msg->status.status = sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;      //RTK FIXED/FLOAT 都映射为 STATUS_GBAS_FIX（高精度）
    } else if (data.is_rtk_float) {
        navsatfix_msg->status.status = sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
    } else {                                                                                 
        navsatfix_msg->status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;           //普通 GPS 为 STATUS_FIX
    }
    navsatfix_msg->status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;

    double pos_cov = data.is_rtk_fixed ? 0.16 : (data.is_rtk_float ? 0.5 : 1.0);
    navsatfix_msg->position_covariance[0] = pos_cov;
    navsatfix_msg->position_covariance[4] = pos_cov;
    navsatfix_msg->position_covariance[8] = pos_cov;
    navsatfix_msg->position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;     //协方差类型设为 COVARIANCE_TYPE_DIAGONAL_KNOWN（只有对角线有值）

    navsatfix_publisher->publish(*navsatfix_msg);
}

// ====== 绝对航向 + 天线偏移补偿 ======
void publishOdometry(const RTKData& data, std::shared_ptr<rclcpp::Node> node) {
    if (!data.data_valid || !odom_publisher || !coordinate_converter || !coordinate_converter->isOriginSet()) {
        return;
    }

    // 获取天线的绝对位置(将 RTK 报告的经纬度转换成局部直角坐标 (ant_x, ant_y)。注意：这是天线的位置，不是车体中心。)
    double ant_x, ant_y, z;
    coordinate_converter->latLonToLocal(data.position.latitude, data.position.longitude, data.position.altitude, ant_x, ant_y, z);

    // ====== 航向转换（修正版 - 适配设备定义）======
    // 设备HDT定义: 0°=东, 顺时针增加
    // 标准HDT定义: 0°=北, 顺时针增加
    // 转换: standard_hdt = (device_hdt + 90) % 360
    
    double device_hdt = data.head_angle_true;
    double standard_hdt = normalizeAngleDeg360(device_hdt + 90.0);
    
    // 标准HDT → ROS Yaw: ROS Yaw = 90° - 标准HDT
    // ROS坐标系: 0°=东, 逆时针增加 (ENU)
    double yaw_deg = 90.0 - standard_hdt;
    yaw_deg = normalizeAngleDeg(yaw_deg);  // 归一化到 [-180°, 180°]
    
    // 转换为弧度
    double yaw_rad = yaw_deg * M_PI / 180.0;

    // ====== 天线偏移补偿 ======
    const double antenna_x = 0.319;   //天线相对于 base_link 前后偏移 (正=前方)
    const double antenna_y = 0.0;     //天线相对于 base_link 左右偏移 (正=左侧)

    // 将天线偏移量旋转到当前航向方向，然后从天线位置中减去，得到车体中心 (base_x, base_y)
    double cos_yaw = cos(yaw_rad);
    double sin_yaw = sin(yaw_rad);
    
    double base_x = ant_x - (antenna_x * cos_yaw - antenna_y * sin_yaw);
    double base_y = ant_y - (antenna_x * sin_yaw + antenna_y * cos_yaw);

    // ====== 构造 Odometry 消息 ======
    auto odom_msg = std::make_shared<nav_msgs::msg::Odometry>();
    odom_msg->header.stamp = node->now();
    odom_msg->header.frame_id = "map";           //全局坐标系(定位结果直接发布在 map 坐标系下（不经过 odom → 没有漂移）)
    odom_msg->child_frame_id = "base_link";      //子坐标系(位置已经是车体中心坐标)

    // 位置 (base_link 中心)
    odom_msg->pose.pose.position.x = base_x;
    odom_msg->pose.pose.position.y = base_y;
    odom_msg->pose.pose.position.z = 0.0;        //2D 平面

    // 姿态 (仅 yaw)
    odom_msg->pose.pose.orientation.x = 0.0;
    odom_msg->pose.pose.orientation.y = 0.0;
    odom_msg->pose.pose.orientation.z = sin(yaw_rad * 0.5);
    odom_msg->pose.pose.orientation.w = cos(yaw_rad * 0.5);

    // ====== 协方差矩阵 ======
    for (int i = 0; i < 36; i++) {
        odom_msg->pose.covariance[i] = 0.0;
        odom_msg->twist.covariance[i] = 0.0;
    }

    // 位置协方差
    double pos_var = data.is_rtk_fixed ? 0.16 : (data.is_rtk_float ? 0.5 : 1.0);
    odom_msg->pose.covariance[0]  = pos_var;   // x
    odom_msg->pose.covariance[7]  = pos_var;   // y
    odom_msg->pose.covariance[14] = pos_var;   // z

    // 姿态协方差
    odom_msg->pose.covariance[21] = 1e6;       // roll 不可信
    odom_msg->pose.covariance[28] = 1e6;       // pitch 不可信
    odom_msg->pose.covariance[35] = 0.0001;    // yaw 可信 (约 0.57° 标准差)

    // ====== 发布消息 ======
    odom_publisher->publish(*odom_msg);
}

int main(int argc, char* argv[]) {
    // ====== 初始化 ======
    signal(SIGINT, signalHandler);
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("location_node");

    // ====== 参数声明与读取 ======
    node->declare_parameter<std::string>("serial_port", RTKConfig::DEFAULT_PORT);   //RTK 串口设备路径
    node->declare_parameter<int>("baud_rate", RTKConfig::DEFAULT_BAUDRATE);         //串口波特率
    node->declare_parameter<double>("datum_lat", 29.4898045);                       //地图原点纬度（重庆某位置）
    node->declare_parameter<double>("datum_lon", 106.5676103);                      //地图原点经度
    node->declare_parameter<bool>("auto_set_origin", false);                        //是否自动用第一帧有效数据设原点
    
    // 添加航向偏移参数（用于微调）
    node->declare_parameter<double>("heading_offset", 0.0);                         //航向偏移量（度）
    
    std::string port = node->get_parameter("serial_port").as_string();
    int baudrate = node->get_parameter("baud_rate").as_int();
    double datum_lat = node->get_parameter("datum_lat").as_double();
    double datum_lon = node->get_parameter("datum_lon").as_double();
    bool auto_set_origin = node->get_parameter("auto_set_origin").as_bool();
    double heading_offset = node->get_parameter("heading_offset").as_double();
    
    // ====== 初始化坐标转换器 ======
    coordinate_converter = std::make_unique<CoordinateConverter>(datum_lat, datum_lon, 0.0);
    std::cout << "Using map origin: lat=" << datum_lat << ", lon=" << datum_lon << std::endl;
    std::cout << "Heading offset: " << heading_offset << "°" << std::endl;

    // ====== 创建发布器 ======
    navsatfix_publisher = node->create_publisher<sensor_msgs::msg::NavSatFix>("/gps/fix", 10);
    odom_publisher = node->create_publisher<nav_msgs::msg::Odometry>("/odometry/gps", 10);

    auto source_publisher = node->create_publisher<std_msgs::msg::String>(
        "/localization/rtk_source_state", 10);
    const auto instance = std::to_string(getpid()) + "-" + std::to_string(
        std::chrono::steady_clock::now().time_since_epoch().count());
    uint64_t status_seq = 0;
    RtkSourceState published_evidence;

    // ====== 串口初始化与启动 ======
    SerialReader reader;
    
    std::cout << "Initializing RTK receiver on " << port << " at " << baudrate << " baud..." << std::endl;
    
    if (!reader.initialize(port, baudrate)) {
        std::cerr << "RTK unavailable; waiting for the serial device to reconnect." << std::endl;
    }
    
    reader.startReading();
    std::cout << "RTK reader started; waiting for fresh position data." << std::endl;
    
    // ====== 主循环 ======
    rclcpp::Rate rate(20);   //20Hz 循环
    std::chrono::system_clock::time_point last_published{};
    
    while (running && rclcpp::ok()) {
        RtkSourceState used;
        RTKData data = reader.getLatestData(&used);      //获取最新 RTK 数据
        displayData(data);
        
        if (data.data_valid) {
            if (auto_set_origin && !coordinate_converter->isOriginSet()) {
                coordinate_converter->setOrigin(data.position.latitude,    //自动设原点
                                               data.position.longitude, 
                                               data.position.altitude);
                std::cout << "Auto-set origin: lat=" << data.position.latitude 
                          << ", lon=" << data.position.longitude << std::endl;
            }
            
            if (data.timestamp != last_published) {
                publishNavSatFix(data, node);
                publishOdometry(data, node);
                last_published = data.timestamp;
                published_evidence = used;
            }
        }
        
        std_msgs::msg::String source_status;
        source_status.data = sourceJson(reader.getSourceState(), instance, ++status_seq,
            published_evidence, datum_lat, datum_lon);
        source_publisher->publish(source_status);
        rclcpp::spin_some(node);                                           //处理 ROS 回调和异步事件
        rate.sleep();                                                      //保持 20Hz 帧率
    }
    
    // ====== 清理退出 ======
    reader.stopReading();                           //停止串口读取线程
    odom_publisher.reset();
    navsatfix_publisher.reset();
    node.reset();
    rclcpp::shutdown();                             //关闭 ROS2
    
    return 0;
}
