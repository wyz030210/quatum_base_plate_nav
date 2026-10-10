//只包含自己的头文件，依赖关系由 .hpp 管理。CoordinateConverter 类声明、GeographicLib/LocalCartesian.hpp 都在头文件里引入
#include "location/coordinate_converter.hpp"

//默认构造函数(创建一个没有原点的转换器)
CoordinateConverter::CoordinateConverter()
    :origin_lat_(0.0), origin_lon_(0.0), origin_altitude_(0.0), origin_set_(false) {
}                                         //所有经纬度成员初始化为 0
                                          //origin_set_ = false — 原点未设置
                                          //local_cartesian_（std::unique_ptr）默认为 nullptr，不创建 GeographicLib 对象

// 带参构造函数(创建时直接指定原点经纬度，转换器立即可用)
CoordinateConverter::CoordinateConverter(double latitude, double longitude, double altitude)
    :origin_lat_(latitude), origin_lon_(longitude), origin_altitude_(altitude), origin_set_(true) {
    local_cartesian_ = std::make_unique<GeographicLib::LocalCartesian>(
        latitude, longitude, altitude);
}                                         //将 (latitude, longitude, altitude) 存为原点
                                          //origin_set_ = true
                                          //立即创建 GeographicLib::LocalCartesian 对象

//重新设置原点或延迟初始化。
void CoordinateConverter::setOrigin(double latitude, double longitude, double altitude) {
    origin_lat_ = latitude;
    origin_lon_ = longitude;
    origin_altitude_ = altitude;
    local_cartesian_ = std::make_unique<GeographicLib::LocalCartesian>(
        latitude, longitude, altitude);
    origin_set_ = true;
}                                         //更新存储的原点坐标
                                          //销毁旧的 local_cartesian_，创建新的（make_unique 会先释放旧对象）
                                          //标记 origin_set_ = true

//纬度/经度/海拔 → 局部直角坐标（XYZ）。
void CoordinateConverter::latLonToLocal(double latitude, double longitude, double altitude, double& x, double& y, double& z) {
    if(!origin_set_) {                    //如果还没有原点，自动以当前传入的 (lat, lon, alt) 为原点
        setOrigin(latitude, longitude, altitude);
        x = y = z = 0.0;                  //返回 (0, 0, 0) — 第一帧位置就是原点本身
        return;
    }

    local_cartesian_->Forward(latitude, longitude, altitude, x, y, z);
}                                         //调用 GeographicLib::LocalCartesian::Forward(lat, lon, alt, x, y, z)
                                          //以当前原点为基准，将经纬度投影到局部 ENU 坐标系
 
//局部直角坐标 → 纬度/经度/海拔（latLonToLocal 的逆运算）。
void CoordinateConverter::localToLatLon(double x, double y, double z, double& latitude, double& longitude, double& altitude) {
    if(!origin_set_) {                    //直接返回 (0, 0, 0) 因为没有原点，无法从 XY 反算回经纬度
        latitude = longitude = altitude = 0.0;
        return; 
    }

    local_cartesian_->Reverse(x, y, z, latitude, longitude, altitude);
}                                         //调用 GeographicLib::LocalCartesian::Reverse(x, y, z, lat, lon, alt)
                                          //将局部坐标 (x, y, z) 反算回 WGS84 经纬度

//查询原点是否已设置
bool CoordinateConverter::isOriginSet() const {
    return origin_set_;
}