// 高德瓦片对接测试：GCJ-02 偏移、往返、URL 构造。
#include <cassert>
#include <cmath>
#include <iostream>
#include <string>

#include "map/amap_tile.h"
#include "map/tile_math.h"

namespace {

double dist_deg(double a_lat, double a_lon, double b_lat, double b_lon) {
    const double dlat = a_lat - b_lat;
    const double dlon = a_lon - b_lon;
    return std::sqrt(dlat * dlat + dlon * dlon);
}

void test_gcj02_offset() {
    using namespace spatial_map;
    // 杭州：中国境内，应有数百米偏移（约 0.002~0.005 度）
    const LatLon g = wgs84_to_gcj02(30.2740, 120.1550);
    const double d = dist_deg(30.2740, 120.1550, g.lat, g.lon);
    assert(d > 0.002 && d < 0.01);
    std::cout << "  [ok] gcj02 offset=" << d << " deg\n";
}

void test_out_of_china_no_offset() {
    using namespace spatial_map;
    // 境外：不偏移
    const LatLon g = wgs84_to_gcj02(40.7128, -74.0060);  // 纽约
    assert(std::abs(g.lat - 40.7128) < 1e-9);
    assert(std::abs(g.lon - (-74.0060)) < 1e-9);
    std::cout << "  [ok] out-of-china no offset\n";
}

void test_gcj02_roundtrip() {
    using namespace spatial_map;
    const double lat = 30.2740, lon = 120.1550;
    const LatLon g = wgs84_to_gcj02(lat, lon);
    const LatLon back = gcj02_to_wgs84(g.lat, g.lon);
    const double d = dist_deg(lat, lon, back.lat, back.lon);
    assert(d < 1e-5);  // 往返误差 < 1 米
    std::cout << "  [ok] gcj02 roundtrip err=" << d << " deg\n";
}

void test_amap_tile_and_url() {
    using namespace spatial_map;
    const TileKey k = wgs84_to_amap_tile(30.2740, 120.1550, 18);
    const std::string url = amap_tile_url(k, 3);
    assert(url.find("webrd03.is.autonavi.com") != std::string::npos);
    assert(url.find("x=" + std::to_string(k.x)) != std::string::npos);
    assert(url.find("y=" + std::to_string(k.y)) != std::string::npos);
    assert(url.find("z=" + std::to_string(k.z)) != std::string::npos);
    std::cout << "  [ok] amap tile url: z" << k.z << "/" << k.x << "/" << k.y << "\n";
}

void test_wgs84_vs_amap_tile_shift() {
    using namespace spatial_map;
    // 同一 WGS84 坐标：高德瓦片与纯 WGS84 瓦片应相差几个瓦片（GCJ-02 偏移）
    const TileKey wgs = latlon_to_tile(30.2740, 120.1550, 18);
    const TileKey amap = wgs84_to_amap_tile(30.2740, 120.1550, 18);
    const int dx = static_cast<int>(amap.x) - static_cast<int>(wgs.x);
    const int dy = static_cast<int>(amap.y) - static_cast<int>(wgs.y);
    assert(std::abs(dx) <= 8 && std::abs(dy) <= 8);  // 偏移不应超过几十米
    std::cout << "  [ok] tile shift dx=" << dx << " dy=" << dy << "\n";
}

}  // namespace

int main() {
    std::cout << "== amap tile tests ==\n";
    test_gcj02_offset();
    test_out_of_china_no_offset();
    test_gcj02_roundtrip();
    test_amap_tile_and_url();
    test_wgs84_vs_amap_tile_shift();
    std::cout << "== all passed ==\n";
    return 0;
}
