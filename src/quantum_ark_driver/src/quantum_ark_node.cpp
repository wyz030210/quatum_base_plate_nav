// ============================================================================
//ROS2 的头文件:话题发布/订阅和 TF
// ============================================================================
#include <rclcpp/rclcpp.hpp>                      //ROS2 C++ 核心库
#include <geometry_msgs/msg/twist.hpp>            //Twist 消息类型（cmd_vel）
#include <geometry_msgs/msg/transform_stamped.hpp>//TF 变换消息
#include <nav_msgs/msg/odometry.hpp>              //里程计消息
#include <std_msgs/msg/float32.hpp>               //浮点数消息（舵机角度）
#include <tf2_ros/transform_broadcaster.h>        //TF 广播器
#include <tf2/LinearMath/Quaternion.h>            //四元数运算

// ============================================================================
//Linux 系统头文件:操作串口（USB-CAN 模块在 Linux 下表现为一个串口设备 /dev/ttyUSB0）。
// ============================================================================
#include <fcntl.h>                                //串口文件操作 (open, O_RDWR等)
#include <unistd.h>                               //POSIX API (write, close, usleep)
#include <termios.h>                              //串口终端参数配置
#include <cstring>                                //C 字符串操作 (strerror)
#include <vector>                                  //std::vector
#include <cmath>                                  //数学函数 (cos, sin, M_PI)
#include <algorithm>                              //std::min, std::max

// ============================================================================
// CAN 协议常量
// ============================================================================
#define CAN_ID_ROS_CONTROL    0x06                //ROS→STM32 控制指令的 CAN ID
#define CAN_ID_SERVO_FEEDBACK 0x05                //STM32→ROS 舵机角度反馈的 CAN ID
#define CMD_TYPE_COMBINED     0x01                //组合控制指令码（线速度+角速度）
#define CAN_FRAME_HEADER      0xAA                //串口帧头
#define CAN_FRAME_TAIL        0x55                //串口帧尾

// ============================================================================
// 车辆物理参数
// ============================================================================
static constexpr double WHEEL_RADIUS     = 0.1125;  // m 轮子半径
static constexpr int    MOTOR_POLE_PAIRS = 15;      //电机磁极对数
static constexpr double GEAR_RATIO       = 1.0;     //减速比（直驱=1）
static constexpr double WHEELBASE        = 0.30;    // m 轴距
static constexpr double TRACK_WIDTH      = 0.25;    // m 轮距

// ============================================================================
//ERPM(电机电转速) 换算常量
// ============================================================================
static constexpr double MS_TO_ERPM = 60.0 * MOTOR_POLE_PAIRS / (2.0 * M_PI * WHEEL_RADIUS);
// = 15 * 60 / (2 * PI * 0.1275) = 1124.6

// ============================================================================
//阿克曼转向参数
// ============================================================================
static constexpr double SERVO_CENTER    = 90.0;      //舵机中位（直行）
static constexpr double SERVO_LEFT_MAX  = 40.0;      //最大左转角度
static constexpr double SERVO_RIGHT_MAX = 190.0;     //最大右转角度
static constexpr double STEERING_RATIO  = 15.0;      //转向传动比
//舵机角度 90° = 直行，小于 90° 左转，大于 90° 右转。范围 40°~190° 对应机械限位。

// ============================================================================
//打包串口帧（把 CAN 帧数据封装成 USB-CAN 模块认识的串口协议格式。）
// ============================================================================
static void pack_can_frame(uint8_t can_id, const uint8_t* data, uint8_t dlc,
                           uint8_t* out, size_t* out_len)
{
    size_t idx = 0;
    out[idx++] = CAN_FRAME_HEADER;
    out[idx++] = 0xC0 | (dlc & 0x0F);
    out[idx++] = can_id & 0xFF;
    out[idx++] = (can_id >> 8) & 0xFF;
    for (uint8_t i = 0; i < dlc; ++i) {
        out[idx++] = data[i];
    }
    out[idx++] = CAN_FRAME_TAIL;
    *out_len = idx;
}

// ============================================================================
//构建8字节控制数据（把 ROS 的线速度（m/s）和角速度（rad/s）转换成 STM32 能解析的 8 字节格式。）
// ============================================================================
static void build_combined_control(double linear_x, double angular_z,
                                   uint8_t* data_out)
{
    // 线速度 -> ERPM
    double clamped_linear = std::max(-2.0, std::min(2.0, linear_x));   //限幅 ±2m/s
    int32_t erpm = static_cast<int32_t>(clamped_linear * MS_TO_ERPM);  //乘以 1124.6
    erpm = std::max(-1000, std::min(1000, erpm));                      //限幅 ±2250 ERPM
    //限幅很重要：底盘最高 2m/s，驱动器最 大ERPM ±2250，超限会出问题
    
    //角速度 → 舵机角度：
    double servo_angle;
    if (angular_z > 0.0) {
        servo_angle = 90.0 - angular_z * 30.0;                         //左转：90→40°
    } else if (angular_z < 0.0) {
        servo_angle = 90.0 + std::abs(angular_z) * 30.0;              //右转：90→190°
    } else {
        servo_angle = 90.0;                                            //直行
    }
    servo_angle = std::max(0.0, std::min(180.0, servo_angle));        //限幅 40~190°
    uint16_t angle_int = static_cast<uint16_t>(servo_angle);
    
    //打包成大端序 8 字节：
    data_out[0] = CMD_TYPE_COMBINED;                                   //0x01 = 组合控制指令
    data_out[1] = (erpm >> 24) & 0xFF;                                 //ERPM 最高字节
    data_out[2] = (erpm >> 16) & 0xFF;
    data_out[3] = (erpm >> 8) & 0xFF;
    data_out[4] = erpm & 0xFF;                                         //ERPM 最低字节
    data_out[5] = (angle_int >> 8) & 0xFF;                             //角度高字节
    data_out[6] = angle_int & 0xFF;                                    //角度低字节
    data_out[7] = 0x00;                                                //保留字节
}

// ============================================================================
// QuantumArkNode类
// ============================================================================
class QuantumArkNode : public rclcpp::Node
{
public:
    QuantumArkNode()
    : Node("quantum_ark_node"), serial_fd_(-1)                         //构造函数
    {
        //声明参数
        this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
        this->declare_parameter<int>("serial_baudrate", 2000000);
        this->declare_parameter<std::string>("odom_frame_id", "odom");
        this->declare_parameter<std::string>("base_frame_id", "base_link");
        this->declare_parameter<double>("wheelbase", 0.30);
        this->declare_parameter<double>("publish_rate", 50.0);
        this->declare_parameter<double>("cmd_vel_timeout", 1.0);
        //读取参数
        port_          = this->get_parameter("serial_port").as_string();
        baudrate_      = this->get_parameter("serial_baudrate").as_int();
        odom_frame_    = this->get_parameter("odom_frame_id").as_string();
        base_frame_    = this->get_parameter("base_frame_id").as_string();
        wheelbase_     = this->get_parameter("wheelbase").as_double();
        publish_rate_  = this->get_parameter("publish_rate").as_double();
        cmd_timeout_   = this->get_parameter("cmd_vel_timeout").as_double();

        //创建发布者
        odom_pub_      = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);               //nav_msgs/Odometry 类型，发布里程计
        servo_pub_     = this->create_publisher<std_msgs::msg::Float32>("/vehicle/servo_angle", 10); //Float32 类型，发布当前舵机角度

        //创建订阅者
        cmd_vel_sub_   = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&QuantumArkNode::cmdVelCallback, this, std::placeholders::_1));                 //geometry_msgs/Twist 类型，接收速度指令

        //创建 TF 广播器
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        //创建定时器
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate_)),
            std::bind(&QuantumArkNode::timerCallback, this));                                          //默认 50Hz（1000/50 = 20ms），定时执行 timerCallback()

        //打开串口
        openSerial();

        //初始化里程计状态
        x_ = 0.0;
        y_ = 0.0;
        theta_ = 0.0;
        vx_ = 0.0;
        wz_ = 0.0;
        last_cmd_time_ = this->now();

        RCLCPP_INFO(this->get_logger(),
            "Quantum Ark driver started. Port=%s, odom_frame=%s, base_frame=%s",
            port_.c_str(), odom_frame_.c_str(), base_frame_.c_str());
    }

    //析构函数(节点退出时确保底盘安全停车。)
    ~QuantumArkNode()
    {
        //发 5 次停止指令
        sendStopCommand();
        if (serial_fd_ >= 0) {
            close(serial_fd_);              //关闭串口
        }
    }

private:
    // ========================================================================
    //打开并配置串口
    // ========================================================================
    void openSerial()
    {
        serial_fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (serial_fd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port %s: %s",
                         port_.c_str(), strerror(errno));
            return;
        }

        // Configure baud rate
        struct termios tty{};
        if (tcgetattr(serial_fd_, &tty) != 0) {
            RCLCPP_ERROR(this->get_logger(), "tcgetattr failed: %s", strerror(errno));
            ::close(serial_fd_);
            serial_fd_ = -1;
            return;
        }

        speed_t speed = B2000000;  //波特率 2,000,000 bps
        cfsetispeed(&tty, speed);
        cfsetospeed(&tty, speed);

        tty.c_cflag &= ~PARENB;    //无校验
        tty.c_cflag &= ~CSTOPB;    //1 停止位
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;        //8 数据位
        tty.c_cflag &= ~CRTSCTS;
        tty.c_cflag |= CLOCAL | CREAD;
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        tty.c_iflag &= ~(ICRNL | IGNCR);
        tty.c_oflag &= ~OPOST;
        tty.c_cc[VMIN]  = 0;
        tty.c_cc[VTIME] = 1;      //非阻塞读取，100ms 超时

        tcsetattr(serial_fd_, TCSANOW, &tty);
        tcflush(serial_fd_, TCIOFLUSH);

        RCLCPP_INFO(this->get_logger(), "Serial port %s opened at %d baud",
                     port_.c_str(), baudrate_);
        // CAN 总线波特率配置码对照表
        // 0x01=125k, 0x02=250k, 0x03=500k, 0x04=1M
        static constexpr uint8_t CAN_BAUDRATE_CODE = 0x03;  // 500kbps

        // 校验和计算（从第 2 字节到倒数第 1 字节累加）
        auto calculate_checksum = [](const std::vector<uint8_t>& data) -> uint8_t {
            uint8_t sum = 0;
            for (size_t i = 2; i < data.size(); ++i) {
                sum += data[i];
            }
            return sum;
        };

        // 构建配置命令：AA 55 12 [波特率码] [帧类型] [过滤] [掩码] [保留] [校验和]
        std::vector<uint8_t> cfg = {
            0xAA,                                      // 帧头
            0x55,                                      // 帧头
            0x12,                                      // 命令类型（CAN 配置）
            CAN_BAUDRATE_CODE,                         // CAN 波特率：500kbps
            0x01,                                      // 帧类型：标准帧
            0x00, 0x00, 0x00, 0x00,                   // 过滤器 ID（接收所有 ID）
            0x00, 0x00, 0x00, 0x00,                   // 掩码（接收所有 ID）
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00        // 保留字节
        };
        cfg.push_back(calculate_checksum(cfg));        // 追加校验和

        serialWrite(cfg.data(), cfg.size());
        RCLCPP_INFO(this->get_logger(),
            "CAN config sent: baudrate=500kbps, standard frames, accept all IDs");

        // 等待模块配置生效
        usleep(500000);  // 500ms

        // 读取配置响应（如果有的话）
        uint8_t resp[128];
        ssize_t nread = ::read(serial_fd_, resp, sizeof(resp));
        if (nread > 0) {
            RCLCPP_INFO(this->get_logger(), "CAN module responded (%ld bytes)", nread);
        } else {
            RCLCPP_DEBUG(this->get_logger(), "No response from CAN module (normal)");
        }
    }

    //写串口(往 USB-CAN 模块写数据。检查返回值确保所有字节都发送成功。)
    bool serialWrite(const uint8_t* data, size_t len)
    {
        if (serial_fd_ < 0) {
            RCLCPP_WARN(this->get_logger(), "serialWrite: fd is invalid!");
            return false;
        }
        ssize_t written = ::write(serial_fd_, data, len);
        if (written < 0) {
            RCLCPP_ERROR(this->get_logger(), "serialWrite FAILED: %s", strerror(errno));
            return false;
        }
        return written == static_cast<ssize_t>(len);                     //检查是否写完
    }

    // ========================================================================
    //核心控制函数(调用时机：每次收到 /cmd_vel 消息时触发)
    // ========================================================================
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        last_cmd_time_ = this->now();                                    //记录收到指令的时间（用于超时检测）

        vx_ = std::max(-2.0, std::min(2.0, msg->linear.x));
        wz_ = std::max(-2.0, std::min(2.0, msg->angular.z));             //保存限幅后的vx_ wz_（用于里程计积分）

        uint8_t data[8];
        build_combined_control(msg->linear.x, msg->angular.z, data);     //调用build_combined_control生成 8 字节数据

        uint8_t frame[16];
        size_t frame_len = 0;
        pack_can_frame(CAN_ID_ROS_CONTROL, data, 8, frame, &frame_len);  //调用pack_can_frame封装成串口帧

        // 打印将要发送的串口帧（十六进制），方便调试
        static int tx_count = 0;
        if (tx_count < 3) {
            std::string hex_str = "TX [" + std::to_string(frame_len) + "]: ";
            for (size_t i = 0; i < frame_len; ++i) {
                char buf[4];
                snprintf(buf, sizeof(buf), "%02X ", frame[i]);
                hex_str += buf;
            }
            RCLCPP_INFO(this->get_logger(), "%s (vx=%.2f, wz=%.2f)", hex_str.c_str(), vx_, wz_);
            tx_count++;
        }

        bool ok = serialWrite(frame, frame_len);                         //调用serialWrite发送
        if (!ok) {
            RCLCPP_ERROR(this->get_logger(), "serialWrite FAILED for cmd_vel frame!");
        }
    }
    ///cmd_vel → Twist → ERPM+角度 → 8字节 → 0xAA..0x55封装 → USB-CAN → CAN总线 → STM32

    // ========================================================================
    // 里程计算+TF(调用时机：默认每 20ms（50Hz）执行一次。)
    // ========================================================================
    void timerCallback()
    {
        auto now = this->now();
        double dt = (now - last_timer_time_).seconds();                  //时间计算(计算距离上次调用过去了多少秒，用于积分。)
        last_timer_time_ = now;

        if (dt <= 0.0 || dt > 1.0) {
            // First call or large jump, skip integration
            return;
        }

        double cmd_age = (now - last_cmd_time_).seconds();               //超时保护
        if (cmd_age > cmd_timeout_) {
            vx_ = 0.0;                                                   //超 1 秒没有收到 cmd_vel → 强制停车
            wz_ = 0.0;
        }

        //航位推算
        double dx = vx_ * std::cos(theta_) * dt;                         //X 方向位移             
        double dy = vx_ * std::sin(theta_) * dt;                         //Y 方向位移
        double dtheta = wz_ * dt;                                        //角度变化

        x_ += dx;                                                     
        y_ += dy;
        theta_ += dtheta;                                                //累加位置+角度

        while (theta_ > M_PI)  theta_ -= 2.0 * M_PI;
        while (theta_ < -M_PI) theta_ += 2.0 * M_PI;                     // 角度归一化到 [-π, π]

        //发布里程计
        nav_msgs::msg::Odometry odom;                                    //构造nav_msgs::msg::Odometry消息并发布到/odom
        odom.header.stamp = now;
        odom.header.frame_id = odom_frame_;
        odom.child_frame_id = base_frame_;

        // 位姿(x,y,theta)用四元数表示
        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;
        odom.pose.pose.position.z = 0.0;
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, theta_);
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.pose.pose.orientation.w = q.w();

        //速度(vx,0,wz)从 cmd_vel 直接拿
        odom.twist.twist.linear.x  = vx_;
        odom.twist.twist.linear.y  = 0.0;
        odom.twist.twist.angular.z = wz_;

        //协方差：设定合理的置信度
        odom.pose.covariance[0]  = 0.05;  // x
        odom.pose.covariance[7]  = 0.05;  // y
        odom.pose.covariance[35] = 0.1;   // yaw
        odom.twist.covariance[0]  = 0.01;  // vx
        odom.twist.covariance[35] = 0.02;  // wz

        odom_pub_->publish(odom);

        //广播 TF(让 ROS 导航栈（Nav2）知道底盘在世界坐标系中的位置。)
        // geometry_msgs::msg::TransformStamped tf_msg;
        // tf_msg.header.stamp = now;
        // tf_msg.header.frame_id = odom_frame_;                               //父坐标
        // tf_msg.child_frame_id = base_frame_;                                //子坐标（底盘中心）

        // tf_msg.transform.translation.x = x_;                                //位置
        // tf_msg.transform.translation.y = y_;
        // tf_msg.transform.translation.z = 0.0;
        // tf_msg.transform.rotation.x = q.x();                                //姿态（四元数）
        // tf_msg.transform.rotation.y = q.y();
        // tf_msg.transform.rotation.z = q.z();
        // tf_msg.transform.rotation.w = q.w();

        // tf_broadcaster_->sendTransform(tf_msg);
    }

    // ========================================================================
    //安全停车(发送 ERPM=0、角度=90° 的指令，连续发 5 次（每次间隔 20ms），确保 STM32 收到。)
    // ========================================================================
    void sendStopCommand()
    {
        uint8_t data[8];
        build_combined_control(0.0, 0.0, data);

        uint8_t frame[16];
        size_t frame_len = 0;
        pack_can_frame(CAN_ID_ROS_CONTROL, data, 8, frame, &frame_len);

        // Send multiple times to ensure delivery
        for (int i = 0; i < 5; ++i) {
            serialWrite(frame, frame_len);
            usleep(20000);  // 20ms
        }
    }

    // ---- ROS interfaces ----
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr servo_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;

    // ---- Parameters ----
    std::string port_;
    int baudrate_;
    std::string odom_frame_;
    std::string base_frame_;
    double wheelbase_;
    double publish_rate_;
    double cmd_timeout_;

    // ---- Serial ----
    int serial_fd_;

    // ---- Odometry state ----
    double x_, y_, theta_;
    double vx_, wz_;
    rclcpp::Time last_cmd_time_;
    rclcpp::Time last_timer_time_{this->now()};
};

// ============================================================================
// Main
// ============================================================================
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);                       //初始化 ROS2
    auto node = std::make_shared<QuantumArkNode>(); //创建节点（自动调构造函数）
    rclcpp::spin(node);                             //循环等待回调
    rclcpp::shutdown();                             //关闭（自动调析构函数）
    return 0;

}
