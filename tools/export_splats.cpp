// 瓦片 → 高斯泼溅导出工具。
//
// 把已登记的瓦片转成 3D Gaussian Splatting 的 splat 集合，
// 导出标准 3DGS PLY，供 gsplat / SuperSplat 等渲染。
// 第一版用"每瓦片一个代理 splat"（位置=瓦片中心，尺度=瓦片米制大小，
// 颜色按瓦片哈希伪彩色）；真实点云 → splat 的生成留给渲染引擎接入。
//
// 用法：export_splats <anchor_lat> <anchor_lon> <zoom> <out.ply>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

#include "map/tile_math.h"
#include "map/hash_tile_store.h"
#include "map/amap_tile.h"
#include "map/geo_anchor.h"
#include "render/gaussian_splat.h"

namespace {

// 与 demo 一致的局部位姿（ENU 米），模拟 Super-LIO 增量输出。
struct LioPose {
    double x, y, z;
};

// 按瓦片哈希生成伪彩色（0..1）。
void hash_color(std::uint64_t key, float& r, float& g, float& b) noexcept {
    const auto h = [key](std::uint64_t seed) {
        std::uint64_t v = key * 11400714819323198485ULL + seed;
        v ^= v >> 33U;
        return static_cast<float>((v % 1000U)) / 1000.0f;
    };
    r = h(1);
    g = h(2);
    b = h(3);
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 5) {
        std::cerr << "用法: " << argv[0]
                  << " <anchor_lat> <anchor_lon> <zoom> <out.ply>\n";
        return 2;
    }
    const double alat = std::atof(argv[1]);
    const double alon = std::atof(argv[2]);
    const std::uint32_t zoom = static_cast<std::uint32_t>(std::atoi(argv[3]));
    const std::string out = argv[4];

    const spatial_map::GeoAnchor anchor{alat, alon, 0.0};

    // 模拟 Super-LIO 增量位姿（与 map_state_demo 相同）
    const std::vector<LioPose> poses{
        {0.0, 0.0, 0.0}, {0.0, 500.0, 0.0}, {700.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};

    spatial_map::HashTileStore<64> tiles;
    std::vector<spatial_map::GaussianSplat> splats;

    for (const LioPose& p : poses) {
        const spatial_map::LatLon wgs = spatial_map::local_to_wgs84(anchor, p.x, p.y);
        const spatial_map::TileKey key =
            spatial_map::wgs84_to_amap_tile(wgs.lat, wgs.lon, zoom);
        if (!tiles.insert(key)) continue;  // 重复瓦片跳过

        // 瓦片中心（经纬度）
        double nlat = 0.0, wlon = 0.0;
        double slat = 0.0, elon = 0.0;
        spatial_map::tile_to_latlon(key.z, key.x, key.y, nlat, wlon);
        spatial_map::tile_to_latlon(key.z, key.x + 1, key.y + 1, slat, elon);
        const double clat = (nlat + slat) / 2.0;
        const double clon = (wlon + elon) / 2.0;

        // 瓦片米制边长（近似）
        const double dlat_deg = nlat - slat;
        const double dlon_deg = elon - wlon;
        const double size_y = dlat_deg * 111320.0;
        const double size_x = dlon_deg * 111320.0 * std::cos(clat * M_PI / 180.0);

        // 代理 splat：位置=瓦片中心（ENU 相对锚点），尺度=瓦片 1/3，伪彩色
        // 中心点相对锚点的 ENU 偏移（反向用平面近似）
        const double meters_per_deg_lat = 111320.0;
        const double meters_per_deg_lon =
            meters_per_deg_lat * std::cos(alat * M_PI / 180.0);
        const double en = (clat - alat) * meters_per_deg_lat;   // 北向
        const double ee = (clon - alon) * meters_per_deg_lon;   // 东向

        float r, g, b;
        hash_color(spatial_map::encode_tile_key(key), r, g, b);

        splats.push_back(spatial_map::GaussianSplat{
            ee, en, 0.0, r, g, b, 0.8f,
            static_cast<float>(size_x / 3.0),
            static_cast<float>(size_y / 3.0),
            5.0f, 1.0f, 0.0f, 0.0f, 0.0f});
    }

    const std::int64_t n = spatial_map::write_splat_ply(splats, out);
    if (n < 0) {
        std::cerr << "PLY 写入失败: " << out << "\n";
        return 1;
    }
    std::cout << "导出 " << n << " 个 splat → " << out << "\n";
    return 0;
}
