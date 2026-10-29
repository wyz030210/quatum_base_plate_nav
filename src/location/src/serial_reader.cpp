#include <location/serial_reader.h>         //自身声明
#include "location/nmea_parser.h"           //NMEA解析
#include "location/rtk_config.h"            //配置常量
#include <fcntl.h>                          //open/flags
#include <termios.h>                        //串口参数结构体
#include <unistd.h>                         //read/close
#include <cstring>                          //memset/strerror
#include "iostream"                         //cout/cerr
#include <system_error>                     //错误码
#include <poll.h>

namespace {
constexpr std::chrono::milliseconds position_timeout(1000);
}

//构造/析构
SerialReader::SerialReader() 
    : serial_fd_(-1), connected_(false), running_(false),   //串口文件描述符，-1 表示未打开;未连接;未运行
      total_messages_(0), valid_messages_(0) {              //消息计数器清零
}            //初始化成员变量

SerialReader::~SerialReader() {
    stopReading();
}            //自动调用 stopReading() → 停止线程 + 关闭串口，确保资源释放

//保存串口路径和波特率，调用 connectSerial() 连接 失败返回 false
bool SerialReader::initialize(const std::string& port, int baudrate) {
    port_ = port;
    baudrate_ = baudrate;
    return connectSerial();
}

bool SerialReader::connectSerial() {
    if (connected_) {
        return true;                                                //已连接则直接返回
    }
    
    serial_fd_ = open(port_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);   //可读可写;不让此串口成为控制终端（重要，否则 Ctrl+C 会发给串口）;同步写入（write 阻塞直到完成）
    if (serial_fd_ < 0) {                                           //open 失败
        std::cerr << "Error opening serial port " << port_ << ": " 
                  << strerror(errno) << std::endl;
        return false;
    }
    
    if (!configureSerialPort(baudrate_)) {                          //配置失败
        close(serial_fd_);
        serial_fd_ = -1;
        return false;
    }
    
    remaining_data_.clear();
    connected_ = true;
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        evidence_.connect();
    }
    std::cout << "Connected to " << port_ << " at " << baudrate_ << " baud" << std::endl;
    return true;
}

//关闭串口 fd 重置状态 等——多次调用安全
void SerialReader::disconnectSerial() {
    connected_ = false;
    if (serial_fd_ >= 0) {
        close(serial_fd_);
        serial_fd_ = -1;
        std::cout << "Disconnected from " << port_ << std::endl;
    }
    remaining_data_.clear();
    std::lock_guard<std::mutex> lock(data_mutex_);
    current_data_.data_valid = false;
    evidence_.disconnect();
}

//配置 Linux 串口的全部参数
bool SerialReader::configureSerialPort(int baudrate) {
    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    
    if (tcgetattr(serial_fd_, &tty) != 0) {                         //获取当前属性
        std::cerr << "Error getting termios attributes: " << strerror(errno) << std::endl;
        return false;
    }
    
    // 设置波特率
    speed_t speed;
    switch (baudrate) {
        case 9600: speed = B9600; break;
        case 19200: speed = B19200; break;
        case 38400: speed = B38400; break;
        case 57600: speed = B57600; break;
        case 115200: speed = B115200; break;
        case 230400: speed = B230400; break;
        default: speed = B115200; break;
    }
    cfsetospeed(&tty, speed);                                        //输出波特率
    cfsetispeed(&tty, speed);                                        //输入波特率
    
    // 配置串口参数 (8N1):数据位/校验/停止位
    tty.c_cflag &= ~PARENB;    // 无奇偶校验(8N1 的 N)
    tty.c_cflag &= ~CSTOPB;    // 1位停止位(8N1 的 1)
    tty.c_cflag &= ~CSIZE;     // 清除数据位掩码
    tty.c_cflag |= CS8;        // 8位数据位(8N1 的 8)
    tty.c_cflag &= ~CRTSCTS;   // 无硬件流控
    tty.c_cflag |= (CREAD | CLOCAL); // 启用接收，忽略调制解调器状态
    
    // 输入模式
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // 无软件流控:关闭 XON/XOFF 软件流控，防止数据中出现控制字符被误处理
    tty.c_iflag &= ~(INLCR | ICRNL | IGNCR); // 不转换字符:不转换换行符 CR/LF，NMEA 语句是纯文本，原样保留
    
    // 输出模式
    tty.c_oflag &= ~OPOST; // 原始输出:~OPOST 是最关键的一行——关闭所有输出处理（不把 \n 映射成 \r\n 等），否则数据会被破坏。
    tty.c_oflag &= ~ONLCR; // 不转换换行符
    
    // 本地模式
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // 非规范模式(不按行读取，而是原始字节流);不回显数据;不回显擦除字符;不把 INTR/QUIT/SUSP 信号转给进程（否则串口来的 Ctrl+C 会终止程序）
    
    // 超时设置( 非阻塞模式。read() 不管有没有数据都立即返回，有数据就返回字节数，没数据返回 0。)
    tty.c_cc[VMIN] = 0;    //不要求最小字节数
    tty.c_cc[VTIME] = 0;   //不等待，立即返回
      
    //应用配置并刷新
    if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {                 //TCSANOW — 立即生效，不等数据发送完。tcflush 清掉可能残留的旧数据。
        std::cerr << "Error setting termios attributes: " << strerror(errno) << std::endl;
        return false;
    }
    
    tcflush(serial_fd_, TCIOFLUSH);                                  //清空输入输出缓冲区，扔掉之前残留的数据
    return true;
}

//启动后台读取线程，&SerialReader::readingThread 是指向成员函数的指针，this 是调用对象。
void SerialReader::startReading() {
    if (running_) {                                                  //已在运行则跳过
        return;
    }
    
    if (!connected_ && !connectSerial()) {
        std::cerr << "Serial port unavailable; retrying in the background" << std::endl;
    }
    
    running_ = true;
    read_thread_ = std::thread(&SerialReader::readingThread, this);
    std::cout << "Started reading thread" << std::endl;
}

//有序关闭：先置 running_=false → 线程检测到后退出循环 → join() 等待线程结束 → 关闭串口。
void SerialReader::stopReading() {
    running_ = false;                                                //通知线程退出
    if (read_thread_.joinable()) {
        read_thread_.join();                                         //等待线程结束
    }
    disconnectSerial();                                              //关闭串口
    std::cout << "Stopped reading thread" << std::endl;
}

// 读取线程主循环
void SerialReader::readingThread() {
    auto next_reconnect = std::chrono::steady_clock::now();
    
    while (running_) {
        if (!connected_) {
            const auto now = std::chrono::steady_clock::now();
            if (now >= next_reconnect) {
                connectSerial();
                next_reconnect = now +
                    std::chrono::milliseconds(RTKConfig::RECONNECT_INTERVAL_MS);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }                                                            //如果串口断开了，先等 5 秒（RECONNECT_INTERVAL_MS）再重连
        
        auto sentences = readSerialData();                           //如果已连接，调用 readSerialData() 读取完整 NMEA 语句
        for (const auto& sentence : sentences) {                     //逐条处理
            if (!sentence.empty()) {
                processNMEASentence(sentence);
            }
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(10));  //每次循环 sleep 10ms（约 100Hz），不让线程空转吃满 CPU
    }
}

//读取原始数据并分割语句
std::vector<std::string> SerialReader::readSerialData() {
    std::vector<std::string> sentences;
    if (!connected_) return sentences;

    pollfd descriptor{serial_fd_, POLLIN, 0};
    const int ready = poll(&descriptor, 1, 0);
    if (ready < 0) {
        if (errno != EINTR) disconnectSerial();
        return sentences;
    }
    if (descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) {
        disconnectSerial();
        return sentences;
    }
    if (!(descriptor.revents & POLLIN)) return sentences;
    
    char buffer[256];
    int bytes_read = read(serial_fd_, buffer, sizeof(buffer) - 1);
    
    if (bytes_read > 0) {
        buffer[bytes_read] = '\0';
        std::string data = remaining_data_ + std::string(buffer, bytes_read);
        remaining_data_.clear();
        
        size_t start = 0;
        while (true) {
            size_t end = data.find('\n', start);                    //按换行符分割
            if (end == std::string::npos) {                         //没找到→剩下的是不完整行
                remaining_data_ = data.substr(start);
                if (remaining_data_.size() > 4096) remaining_data_.clear();
                break;
            }
            
            std::string sentence = data.substr(start, end - start);
            // 移除回车符
            if (!sentence.empty() && sentence.back() == '\r') {
                sentence.pop_back();                                //去掉 \r（Windows 风格换行）
            }
            
            if (!sentence.empty() && sentence[0] == '$') {
                sentences.push_back(sentence);                      //以 $ 开头的才是 NMEA 语句
            }
            
            start = end + 1;
        }
    } else if (bytes_read < 0) {
        if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
            std::cerr << "Serial read error: " << strerror(errno) << std::endl;
            disconnectSerial();
        }
    }
    
    return sentences;
}

//拼接机制
void SerialReader::processNMEASentence(const std::string& sentence) {
    total_messages_++;
    
    std::lock_guard<std::mutex> lock(data_mutex_);
    evidence_.observe(sentence);
    RTKData new_data = current_data_;
    
    if (NMEAParser::parseSentence(sentence, new_data)) {           //调用 parseSentence，只更新 new_data 中当前语句涉及的字段
        valid_messages_++;
        
        current_data_ = new_data;
        // Heading traffic must not make an old position look current.
        if (sentence.compare(3, 3, "GGA") == 0 || sentence.compare(3, 3, "RMC") == 0) {
            current_data_.timestamp = std::chrono::system_clock::now();
            last_position_update_ = std::chrono::steady_clock::now();
        }
    }
}

//加锁返回最新数据的副本。car_location.cpp 主循环每次调用都拿一个完整快照。返回的是副本（不是引用），安全
RTKData SerialReader::getLatestData(RtkSourceState * evidence) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (evidence) *evidence = evidence_.snapshot();
    RTKData snapshot = current_data_;
    if (!connected_ || last_position_update_ == std::chrono::steady_clock::time_point{} ||
        std::chrono::steady_clock::now() - last_position_update_ > position_timeout) {
        snapshot.data_valid = false;
        snapshot.is_rtk_fixed = false;
        snapshot.is_rtk_float = false;
        snapshot.rtk_status = connected_ ? "NO DATA" : "DISCONNECTED";
    }
    return snapshot;
}

bool SerialReader::isConnected() const {
    return connected_;
}

std::string SerialReader::getStatus() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (!connected_ || last_position_update_ == std::chrono::steady_clock::time_point{} ||
        std::chrono::steady_clock::now() - last_position_update_ > position_timeout) {
        return "NO DATA";
    }
    if (!current_data_.data_valid) {
        return "INVALID";                                          //有数据，但 fix_quality=0                  
    }
    
    return current_data_.rtk_status;                               //"RTK FIXED" / "RTK FLOAT" / "GPS" 等
}

RtkSourceState SerialReader::getSourceState() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return evidence_.snapshot();
}
