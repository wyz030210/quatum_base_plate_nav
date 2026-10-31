// include/serial_reader.h
#ifndef SERIAL_READER_H
#define SERIAL_READER_H

#include <string>
#include <vector>                    // 一次可能读到多条完整 NMEA 语句
#include <atomic>                    // 跨线程安全读写，一个线程写入（后台线程），一个线程读取（主循环）
#include <mutex>                     //保护 current_data_ 的并发访问
#include <thread>                    //后台串口读取线程
#include "source_evidence.h"
#include "rtk_types.h"               //定义了 RTKData 结构体

class SerialReader {                 //没有继承任何基类。这是一个独立的类，不过很多序列化/反序列化的类名。
public:
    //构造函数 / 析构函数              构造时初始化所有成员到默认状态（fd=-1, connected=false, running=false, 计数器=0）。
    //析构时自动调用 stopReading() → 停止线程 + 关闭串口，确保资源释放。
    SerialReader();
    ~SerialReader();
    
    //打开并配置串口
    bool initialize(const std::string& port, int baudrate);
    void startReading();             //启动后台读取线程 readingThread()。如果串口还没打开，先打开再启动线程
    void stopReading();              //设置 running_ = false → join() 等待线程结束 → 关闭串口
    
    RtkSourceState getSourceState() const;
    RTKData getLatestData(RtkSourceState * evidence = nullptr);         //获取最新的 RTK 数据快照
    bool isConnected() const;        // 返回 connected_（串口是否处于打开状态）
    std::string getStatus() const;   // "NO DATA" after one second without position updates.
    
private:
    void readingThread();            //后台线程的主函数。无限循环
    // 串口操作
    bool connectSerial();            //open() 串口 → configureSerialPort() → 设置成功标志
    void disconnectSerial();         //close(fd) → 重置 fd 和连接标志
    bool configureSerialPort(int baudrate);                  //配置 8N1、非阻塞、无流控等串口参数
    //数据读取与处理
    std::vector<std::string> readSerialData();               //从串口读取原始字节流，按 \n 分割成完整 NMEA 语句，处理粘包/半包
    void processNMEASentence(const std::string& sentence);   //解析单条 NMEA 语句，更新 current_data_
    
    // 串口相关
    int serial_fd_;                  //串口文件描述符
    std::string port_;               //串口设备路径
    int baudrate_;                   //波特率
    std::atomic<bool> connected_;    //是否已连接
    std::atomic<bool> running_;      //是否正在运行
    
    // 数据相关
    SourceEvidence evidence_;
    RTKData current_data_;           //最新的 RTK 数据
    mutable std::mutex data_mutex_;  //保护 current_data_ 的互斥锁
    
    // 线程相关
    std::thread read_thread_;        //后台读取线程
    
    // 统计信息
    uint64_t total_messages_;        //收到的 NMEA 语句总数
    uint64_t valid_messages_;        //校验通过的有效语句数
    std::chrono::steady_clock::time_point last_position_update_{};
    std::string remaining_data_;
};

#endif
