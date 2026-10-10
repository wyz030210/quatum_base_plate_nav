//头文件保护
//坐标转换类(标准头文件保护宏 防止同一个 .hpp 被多个 .cpp 包含时重复定义)
#ifndef LOCATION_COORDINATE_CONVERTER_HPP
#define LOCATION_COORDINATE_CONVERTER_HPP

//包含的依赖
#include <GeographicLib/LocalCartesian.hpp>    //第三方库 GeographicLib(提供 GPS 经纬度 ↔ 局部 ENU 坐标的数学转换)
#include <memory>                              //C++ 标准库(提供 std::unique_ptr 智能指针)

//类声明
class CoordinateConverter {
public:
    CoordinateConverter();                     //默认构造函数(创建一个无原点的转换器。local_cartesian_ 为 nullptr，origin_set_ = false。调用 latLonToLocal() 时会自动调用 setOrigin() 以当前传入坐标设原点。)
    //带参构造函数
    CoordinateConverter(double latitude, double longitude, double altitude = 0.0);
    //重新设置原点：销毁旧的 local_cartesian_，创建新的;更新存储的原点坐标值
    void setOrigin(double latitude, double longitude, double altitude = 0.0);
    //将 WGS84 经纬度 → 局部直角坐标 (x, y, z)
    void latLonToLocal(double latitude, double longitude, double altitude, double& x, double& y, double&z);
    //局部 XY → WGS84 经纬度
    void localToLatLon(double x, double y, double z, double& latitude, double& longitude, double& altitude);
    //查询原点是否已设置。const 保证不修改成员变量。调用方用这个判断转换器是否可用：
    bool isOriginSet() const;
    //将角度归一化到特定范围内
    double normalizeAngle(double angle_rad);

//私有成员变量
private:
    std::unique_ptr<GeographicLib::LocalCartesian> local_cartesian_;
    double origin_lat_, origin_lon_, origin_altitude_;
    bool origin_set_;
};

//头文件保护结束
#endif