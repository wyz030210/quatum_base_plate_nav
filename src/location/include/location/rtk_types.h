//注释与头文件保护
// include/rtk_types.h
#ifndef RTK_TYPES_H
#define RTK_TYPES_H

//包含的依赖
#include <string>
#include <chrono>       //timestamp 是 std::chrono::system_clock::time_point 类型

//结构体
struct Position {
    double latitude;    // 纬度（度）
    double longitude;   // 经度（度）
    double altitude;    // 海拔（米）
    
    Position() : latitude(0.0), longitude(0.0), altitude(0.0) {}
};

struct RTKData {
    // 基本信息
    Position position;  //经纬度 + 海拔，从 GGA 语句解析
    std::string utc_time;
    std::string date;
    
    // 质量指标
    int fix_quality;
    int satellites_used;
    double hdop;        //水平精度因子，越小精度越高
    double pdop;        //位置精度因子（3D），当前没有被解析，保持默认
    double vdop;        //垂直精度因子，当前没有被解析，保持默认
    
    // 运动信息
    double speed_knots;
    double speed_kmh;
    double head_angle_true;   //HDT 双天线 RTK 的真北航向
    double course_magnetic;   //磁北航向（原始保留字段）
    double heading; 
    
    // RTK特定信息
    bool is_rtk_fixed;
    bool is_rtk_float;
    std::string rtk_status;
    int age_of_differential;
    std::string base_station_id;
    
    // 系统状态
    bool data_valid;
    std::string fix_mode;
    std::chrono::system_clock::time_point timestamp;
    
    //构造函数 + reset
    RTKData() {
        reset();
    }
    
    void reset() {
        position = Position();
        utc_time = "";
        date = "";
        fix_quality = 0;
        satellites_used = 0;
        hdop = 99.9;
        pdop = 99.9;
        vdop = 99.9;
        speed_knots = 0.0;
        speed_kmh = 0.0;
        head_angle_true = 0.0;
        course_magnetic = 0.0;
        heading = 0.0;
        is_rtk_fixed = false;
        is_rtk_float = false;
        rtk_status = "NO FIX";
        age_of_differential = 0;
        base_station_id = "";
        data_valid = false;
        fix_mode = "NONE";
        timestamp = std::chrono::system_clock::now();
    }                            //99.9 明确表示"这个值还没被设置
};

#endif