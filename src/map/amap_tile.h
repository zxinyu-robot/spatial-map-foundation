// 高德地图瓦片对接：GCJ-02 坐标对齐 + 瓦片 URL 构造。
//
// 高德底图使用 GCJ-02 加密坐标系，与 WGS84 存在约百米级偏移。
// 我们的哈希瓦片按 WGS84 / Web Mercator 编码（tile_math.h），
// 要与高德底图对齐，需先把 WGS84 转为 GCJ-02，再取瓦片编号。
#pragma once

#include <cmath>
#include <cstdint>
#include <string>

#include "tile_math.h"

namespace spatial_map {

// ---------- WGS84 ↔ GCJ-02 ----------

// 中国境外不偏移。
[[nodiscard]] inline bool out_of_china(double lat, double lon) noexcept {
    return lon < 72.004 || lon > 137.8347 || lat < 0.8293 || lat > 55.8271;
}

namespace detail {
inline double transform_lat(double x, double y) noexcept {
    double ret = -100.0 + 2.0 * x + 3.0 * y + 0.2 * y * y +
                 0.1 * x * y + 0.2 * std::sqrt(std::abs(x));
    ret += (20.0 * std::sin(6.0 * x * M_PI) + 20.0 * std::sin(2.0 * x * M_PI)) * 2.0 / 3.0;
    ret += (20.0 * std::sin(y * M_PI) + 40.0 * std::sin(y / 3.0 * M_PI)) * 2.0 / 3.0;
    ret += (160.0 * std::sin(y / 12.0 * M_PI) + 320.0 * std::sin(y * M_PI / 30.0)) * 2.0 / 3.0;
    return ret;
}

inline double transform_lon(double x, double y) noexcept {
    double ret = 300.0 + x + 2.0 * y + 0.1 * x * x +
                 0.1 * x * y + 0.1 * std::sqrt(std::abs(x));
    ret += (20.0 * std::sin(6.0 * x * M_PI) + 20.0 * std::sin(2.0 * x * M_PI)) * 2.0 / 3.0;
    ret += (20.0 * std::sin(x * M_PI) + 40.0 * std::sin(x / 3.0 * M_PI)) * 2.0 / 3.0;
    ret += (150.0 * std::sin(x / 12.0 * M_PI) + 300.0 * std::sin(x / 30.0 * M_PI)) * 2.0 / 3.0;
    return ret;
}
}  // namespace detail

struct LatLon {
    double lat{0.0};
    double lon{0.0};
};

// WGS84 → GCJ-02（标准算法）。
[[nodiscard]] inline LatLon wgs84_to_gcj02(double lat, double lon) noexcept {
    if (out_of_china(lat, lon)) return LatLon{lat, lon};
    const double a = 6378245.0;
    const double ee = 0.00669342162296594323;
    const double dlat = detail::transform_lat(lon - 105.0, lat - 35.0);
    const double dlon = detail::transform_lon(lon - 105.0, lat - 35.0);
    const double radlat = lat / 180.0 * M_PI;
    double magic = std::sin(radlat);
    magic = 1.0 - ee * magic * magic;
    const double sqrtmagic = std::sqrt(magic);
    const double dlat2 = (dlat * 180.0) /
        ((a * (1.0 - ee)) / (magic * sqrtmagic) * M_PI);
    const double dlon2 = (dlon * 180.0) /
        (a / sqrtmagic * std::cos(radlat) * M_PI);
    return LatLon{lat + dlat2, lon + dlon2};
}

// GCJ-02 → WGS84（3 次迭代，误差 < 0.01m）。
[[nodiscard]] inline LatLon gcj02_to_wgs84(double lat, double lon) noexcept {
    if (out_of_china(lat, lon)) return LatLon{lat, lon};
    double wlat = lat;
    double wlon = lon;
    for (int i = 0; i < 3; ++i) {
        const LatLon g = wgs84_to_gcj02(wlat, wlon);
        wlat += lat - g.lat;
        wlon += lon - g.lon;
    }
    return LatLon{wlat, wlon};
}

// ---------- 高德瓦片 URL ----------

// 高德瓦片 URL（子域名 1~4 轮询以规避单域限流）。
[[nodiscard]] inline std::string amap_tile_url(const TileKey& k,
                                               std::uint32_t subdomain = 1) noexcept {
    const std::uint32_t s = (subdomain < 1 || subdomain > 4) ? 1 : subdomain;
    return "https://webrd0" + std::to_string(s) +
           ".is.autonavi.com/appmaptile?lang=zh_cn&size=1&scale=1&style=8"
           "&x=" + std::to_string(k.x) +
           "&y=" + std::to_string(k.y) +
           "&z=" + std::to_string(k.z);
}

// WGS84 经纬度 → 高德瓦片（先转 GCJ-02 再取 Web Mercator 瓦片）。
[[nodiscard]] inline TileKey wgs84_to_amap_tile(double lat, double lon,
                                                std::uint32_t z) noexcept {
    const LatLon gcj = wgs84_to_gcj02(lat, lon);
    return latlon_to_tile(gcj.lat, gcj.lon, z);
}

}  // namespace spatial_map
