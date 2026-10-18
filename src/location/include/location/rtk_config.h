//注释与头文件保护
// config/rtk_config.h
#ifndef RTK_CONFIG_H
#define RTK_CONFIG_H

//包含的依赖
#include <string>

//命名空间
namespace RTKConfig {
    // 串口配置(编译时确定默认值，运行时可以通过 launch 参数覆盖，不需要改代码重新编译)
    const std::string DEFAULT_PORT =
        "/dev/serial/by-id/"
        "usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0";
    const int DEFAULT_BAUDRATE = 115200;
    
    // 超时设置（毫秒）
    const int READ_TIMEOUT_MS = 1000;        //这个常量定义了"读超时"，但实际 serial_reader.cpp 中的串口配置用的是非阻塞模式（VMIN=0, VTIME=0，立即返回），读超时实际上未生效
    const int RECONNECT_INTERVAL_MS = 5000;  //当串口断开后，readingThread() 会 sleep 5000ms
    
    // NMEA消息类型枚举
    enum NMEAMessageType {
        GGA = 0,                             //定位信息语句
        RMC,                                 //推荐最小定位信息
        UNKNOWN                              //未知/不支持的类型
    };
    
    // RTK状态枚举
    enum RTKStatus {
        NO_FIX = 0,
        GPS_FIX,
        DGPS_FIX,
        RTK_FLOAT,
        RTK_FIXED
    };
}

#endif
