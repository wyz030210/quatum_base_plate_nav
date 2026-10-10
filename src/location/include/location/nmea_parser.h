//注释与头文件保护
// include/nmea_parser.h
#ifndef NMEA_PARSER_H
#define NMEA_PARSER_H

//包含的依赖
#include <string>               //C++ 标准库(NMEA 语句是字符串，解析函数接收 const std::string&)
#include "rtk_types.h"          //自己的头文件(定义了 RTKData 结构体，所有解析结果都存入它)
#include <vector>               //C++ 标准库(split() 函数返回 std::vector<std::string>std::string)

//类声明(全静态类)
class NMEAParser {
public:
    static bool parseSentence(const std::string& sentence, RTKData& data); 
                                //parseSentence() 是唯一的对外入口，是一个分发器：
private:
    static bool parseGGA(const std::string& sentence, RTKData& data);
    static bool parseRMC(const std::string& sentence, RTKData& data);
    static bool parseHDT(const std::string& sentence, RTKData& data);

    static bool checkChecksum(const std::string& sentence);
                                //校验 NMEA 语句的 XOR 校验和，防止传输错误的数据被使用
    static double nmeaToDecimal(double nmea_coord, char direction);
                                //NMEA 格式 ddmm.mmmm → 十进制 dd.dddd，如 4807.038 → 48.1173
    static std::vector<std::string> split(const std::string& str, char delimiter);
                                //按逗号分割 NMEA 语句为字段数组
    static bool isFieldValid(const std::string& field);
                                //判断字段是否为空（,, 连续逗号会产生空字段）
    static void updateRTKStatus(RTKData& data);
                                //根据 fix_quality（0-6）更新 RTK 状态文本和标志位
};

#endif