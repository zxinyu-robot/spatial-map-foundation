// 命令行工具：WGS84 经纬度 → GCJ-02 → 高德瓦片 x/y/z → 瓦片 URL。
// 用法：
//   amap_tile_tool <lat> <lon> <zoom> [subdomain]
// 输出多行，供脚本抓取：
//   wgs84=<lat>,<lon>
//   gcj02=<lat>,<lon>
//   tile=z<x>/<y>
//   url=<...>
#include <cstdlib>
#include <iostream>
#include <string>

#include "map/amap_tile.h"
#include "map/tile_math.h"

int main(int argc, char** argv) {
    if (argc < 4) {
        std::cerr << "用法: " << argv[0] << " <lat> <lon> <zoom> [subdomain]\n";
        return 2;
    }
    const double lat = std::atof(argv[1]);
    const double lon = std::atof(argv[2]);
    const std::uint32_t z = static_cast<std::uint32_t>(std::atoi(argv[3]));
    const std::uint32_t sub =
        (argc >= 5) ? static_cast<std::uint32_t>(std::atoi(argv[4])) : 1;

    const spatial_map::LatLon gcj = spatial_map::wgs84_to_gcj02(lat, lon);
    const spatial_map::TileKey k = spatial_map::wgs84_to_amap_tile(lat, lon, z);
    const std::string url = spatial_map::amap_tile_url(k, sub);

    std::cout.precision(7);
    std::cout << "wgs84=" << lat << "," << lon << "\n"
              << "gcj02=" << gcj.lat << "," << gcj.lon << "\n"
              << "tile=z" << k.z << "/" << k.x << "/" << k.y << "\n"
              << "url=" << url << "\n";
    return 0;
}
