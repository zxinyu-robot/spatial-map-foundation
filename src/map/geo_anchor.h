// 局部坐标 → 地理经纬度的锚定。
//
// LIO 前端（如 Super-LIO）输出局部坐标系（map / odom）下的增量位姿，
// 要嵌入城市级地图瓦片，需要一个锚定：地图原点经纬度 + 局部坐标轴航向。
// 本层用 ENU 平面近似（小范围地图误差可忽略），把局部 (x, y) 换算为 WGS84。
#pragma once

#include <cmath>

#include "amap_tile.h"  // LatLon

namespace spatial_map {

// 地图锚定：局部坐标系原点对应的 WGS84 经纬度，以及局部 x 轴相对正北的航向（弧度）。
struct GeoAnchor {
    double origin_lat{0.0};   // 原点纬度（度）
    double origin_lon{0.0};   // 原点经度（度）
    double yaw_north{0.0};    // 局部 x 轴相对正北的偏航（弧度，逆时针为正）
};

// 局部 ENU (x 向东, y 向北) 米 → WGS84 经纬度（度）。
// yaw_north 非零时，先按航向旋转局部坐标，再换算。
[[nodiscard]] inline LatLon local_to_wgs84(const GeoAnchor& anchor,
                                           double x_m, double y_m) noexcept {
    // 1. 航向旋转：局部系 → 北东系
    const double c = std::cos(anchor.yaw_north);
    const double s = std::sin(anchor.yaw_north);
    const double east = x_m * c - y_m * s;
    const double north = x_m * s + y_m * c;

    // 2. 平面近似：北向 → 纬度，东向 → 经度（按当地纬度修正）
    constexpr double kMetersPerDegLat = 111320.0;
    const double meters_per_deg_lon =
        kMetersPerDegLat * std::cos(anchor.origin_lat * M_PI / 180.0);

    const double dlat = north / kMetersPerDegLat;
    const double dlon = east / meters_per_deg_lon;

    return LatLon{anchor.origin_lat + dlat, anchor.origin_lon + dlon};
}

}  // namespace spatial_map
