#include "location/nmea_parser.h"         //自己的头文件，声明了 NMEAParser 类（全是静态方法）
#include <iostream>                       // 打印调试信息、错误信息
#include <sstream>                        //用于 stringstream 做字符串分割和校验和解析
#include <algorithm>
#include <rclcpp/rclcpp.hpp>              //ROS2 基础库

//NMEA 语句分发器
bool NMEAParser::parseSentence(const std::string& sentence, RTKData& data) {
    if (!checkChecksum(sentence)) {
        return false;
    }                                     //先校验和检查 → 校验失败直接返回 false
    
    if (sentence.find("GGA") != std::string::npos) {
        return parseGGA(sentence, data);  //定位信息（经纬度、海拔、卫星数等）
    } else if (sentence.find("RMC") != std::string::npos) {
        return parseRMC(sentence, data);  //推荐最小定位信息（速度、日期等）
    } else if (sentence.find("HDT") != std::string::npos) {
        return parseHDT(sentence, data);  //真北航向（双天线 RTK 必备）
    }
    return false;                         //都不是 → 返回 false（忽略该语句）
}

//GGA 语句解析
bool NMEAParser::parseGGA(const std::string& sentence, RTKData& data) {
    auto fields = split(sentence, ',');
    if (fields.size() < 15) return false;
    try {
        // UTC时间
        if (isFieldValid(fields[1])) {
            data.utc_time = fields[1];
        }
        
        // 纬度
        if (isFieldValid(fields[2]) && isFieldValid(fields[3])) {
            double lat = std::stod(fields[2]);
            data.position.latitude = nmeaToDecimal(lat, fields[3][0]);
        }
        
        // 经度
        if (isFieldValid(fields[4]) && isFieldValid(fields[5])) {
            double lon = std::stod(fields[4]);
            data.position.longitude = nmeaToDecimal(lon, fields[5][0]);
        }
        
        // 定位质量
        if (isFieldValid(fields[6])) {
            data.fix_quality = std::stoi(fields[6]);
            data.data_valid = (data.fix_quality > 0) &&
                isFieldValid(fields[2]) && isFieldValid(fields[3]) &&
                isFieldValid(fields[4]) && isFieldValid(fields[5]);
        } else {
            data.fix_quality = 0;
            data.data_valid = false;
        }                                              //1=GPS, 2=DGPS, 4=RTK固定, 5=RTK浮点
        
        // 卫星数量
        if (isFieldValid(fields[7])) {
            data.satellites_used = std::stoi(fields[7]);
        }
        
        // HDOP
        if (isFieldValid(fields[8])) {
            data.hdop = std::stod(fields[8]);
        }
        
        // 海拔高度
        if (isFieldValid(fields[9])) {
            data.position.altitude = std::stod(fields[9]);
        }
        
        updateRTKStatus(data);
        return true;
    } catch (...) {
        return false;
    }
}

//RMC 语句解析
bool NMEAParser::parseRMC(const std::string& sentence, RTKData& data) {
    auto fields = split(sentence, ',');
    if (fields.size() < 12) return false;
    
    try {
        // UTC时间
        if (isFieldValid(fields[1])) {
            data.utc_time = fields[1];
        }
        
        // 状态
        if (isFieldValid(fields[2])) {
            data.data_valid = (fields[2] == "A") &&
                isFieldValid(fields[3]) && isFieldValid(fields[4]) &&
                isFieldValid(fields[5]) && isFieldValid(fields[6]);
        } else {
            data.data_valid = false;
        }
        
        // 纬度
        if (isFieldValid(fields[3]) && isFieldValid(fields[4])) {
            double lat = std::stod(fields[3]);
            data.position.latitude = nmeaToDecimal(lat, fields[4][0]);
        }
        
        // 经度
        if (isFieldValid(fields[5]) && isFieldValid(fields[6])) {
            double lon = std::stod(fields[5]);
            data.position.longitude = nmeaToDecimal(lon, fields[6][0]);
        }
        
        // 速度
        if (isFieldValid(fields[7])) {
            data.speed_knots = std::stod(fields[7]);
            data.speed_kmh = data.speed_knots * 1.852;
        }
        
        // 航向
        //if (isFieldValid(fields[8])) {
          //  data.head_angle_true = std::stod(fields[8]);
        //}
        
        // 日期
        if (isFieldValid(fields[9])) {
            data.date = fields[9];
        }
        
        return true;
    } catch (...) {
        return false;
    }
}

//HDT 语句解析
bool NMEAParser::parseHDT(const std::string& sentence, RTKData& data) {
    auto fields = split(sentence, ',');
    if (fields.size() < 3) return false;
    
    try {
        // HDT格式: $GNHDT,164.9197,T*1E
        // fields[1] = 航向角度, fields[2] = T(真北)或M(磁北)
        if (isFieldValid(fields[1])) {
            data.head_angle_true = std::stod(fields[1]);
            data.heading = data.head_angle_true;  // 复制到heading字段
        }
        return true;
    } catch (...) {
        return false;
    }
}

// 更新 RTK 状态文本
void NMEAParser::updateRTKStatus(RTKData& data) {
    data.is_rtk_fixed = false;
    data.is_rtk_float = false;
    switch (data.fix_quality) {
        case 4:
            data.is_rtk_fixed = true;
            data.rtk_status = "RTK FIXED";
            break;
        case 5:
            data.is_rtk_float = true;
            data.rtk_status = "RTK FLOAT";
            break;
        case 2:
            data.rtk_status = "DGPS";
            break;
        case 1:
            data.rtk_status = "GPS";
            break;
        default:
            data.rtk_status = "NO FIX";
            break;
    }
}

//校验和检查
bool NMEAParser::checkChecksum(const std::string& sentence) {
    // 查找校验和分隔符'*'
    size_t asterisk_pos = sentence.find('*');
    if (asterisk_pos == std::string::npos || asterisk_pos + 3 > sentence.length()) {
        std::cerr << "Invalid NMEA sentence format: " << sentence << std::endl;
        return false;
    }
    
    // 计算校验和（从'$'后开始到'*'前结束）
    uint8_t calculated_checksum = 0;
    for (size_t i = 1; i < asterisk_pos; i++) {
        calculated_checksum ^= sentence[i];
    }
    
    // 提取接收到的校验和
    std::string checksum_str = sentence.substr(asterisk_pos + 1, 2);
    
    // 验证校验和字符串是有效的十六进制
    if (checksum_str.length() != 2 || 
        !std::isxdigit(checksum_str[0]) || 
        !std::isxdigit(checksum_str[1])) {
        std::cerr << "Invalid checksum format: " << checksum_str << std::endl;
        return false;
    }
    
    // 转换校验和
    int received_checksum;
    std::stringstream ss;
    ss << std::hex << checksum_str;
    ss >> received_checksum;
    
    bool result = (calculated_checksum == static_cast<uint8_t>(received_checksum));
    
    if (!result) {
        std::cerr << "Checksum mismatch: calculated=0x" << std::hex << (int)calculated_checksum
                  << ", received=0x" << received_checksum << std::endl;
    }
    
    return result;
}

//NMEA 格式转十进制
double NMEAParser::nmeaToDecimal(double degrees, char direction) {
    //简单调试输出
    std::cout << "DEBUG - 输入：" << degrees << " " << direction << std::endl;

    double deg = static_cast<int>(degrees / 100);     //取整百 = 度
    double min = degrees - deg * 100;                 //剩余 = 分
    double decimal = deg + min / 60.0;                //度 + 分/60

    //调试输出转换过程
    std::cout << "DEBUG - 转换：" << degrees << "° " << min << "' -> " << decimal << "° " << std::endl;
    
    if (direction == 'S' || direction == 'W') {
        decimal = -decimal;
    }
    
    std::cout << "DEBUG - 最终结果：" << decimal << "°" << std::endl;
    return decimal;
}

//字符串分割
std::vector<std::string> NMEAParser::split(const std::string& str, char delimiter) {
    std::vector<std::string> tokens;
    std::stringstream ss(str);
    std::string token;
    
    while (std::getline(ss, token, delimiter)) {
        tokens.push_back(token);
    }
    
    return tokens;
}

//字段有效性检查
bool NMEAParser::isFieldValid(const std::string& field) {
    return !field.empty();
}

