// 哈希瓦片编码：经纬度 ↔ 高德/XYZ 瓦片 z/x/y 互转。
// 采用 Web Mercator（EPSG:3857）与 Slippy Map / XYZ 瓦片标准，
// 瓦片编号与高德地图、OpenStreetMap 的 x/y/z 一致，
// 使局部 SLAM 状态可通过瓦片 ID 对齐到城市级地图数据。
#pragma once

#include <cstdint>
#include <cmath>

namespace spatial_map {

// 瓦片身份：z 为缩放级别，x 为列，y 为行（原点在西北角）。
struct TileKey {
    std::uint32_t z{0};
    std::uint32_t x{0};
    std::uint32_t y{0};

    constexpr bool operator==(const TileKey& o) const noexcept {
        return z == o.z && x == o.x && y == o.y;
    }
};

// 把 z/x/y 压成单个 64 位哈希键（z 高 8 位、y 中 28 位、x 低 28 位）。
// 足够覆盖 zoom 0..255，x/y 0..2^28-1。
[[nodiscard]] constexpr std::uint64_t encode_tile_key(const TileKey& k) noexcept {
    return (static_cast<std::uint64_t>(k.z) << 56U) |
           (static_cast<std::uint64_t>(k.y) << 28U) |
           static_cast<std::uint64_t>(k.x);
}

[[nodiscard]] constexpr TileKey decode_tile_key(std::uint64_t key) noexcept {
    return TileKey{
        static_cast<std::uint32_t>((key >> 56U) & 0xFFU),
        static_cast<std::uint32_t>(key & 0x0FFFFFFFU),
        static_cast<std::uint32_t>((key >> 28U) & 0x0FFFFFFFU),
    };
}

// 经纬度（度）→ Web Mercator 瓦片 x/y/z。
// 结果裁剪到 [0, 2^z - 1]。
[[nodiscard]] inline TileKey latlon_to_tile(double lat, double lon, std::uint32_t z) noexcept {
    const double n = std::ldexp(1.0, static_cast<int>(z));
    const double lat_rad = lat * M_PI / 180.0;

    double xtile = (lon + 180.0) / 360.0 * n;
    const double ytile =
        (1.0 - std::asinh(std::tan(lat_rad)) / M_PI) / 2.0 * n;

    if (xtile < 0.0) xtile = 0.0;
    if (xtile >= n) xtile = n - 1.0;

    const double ny = (ytile < 0.0) ? 0.0 : ((ytile >= n) ? n - 1.0 : ytile);

    return TileKey{
        z,
        static_cast<std::uint32_t>(xtile),
        static_cast<std::uint32_t>(ny),
    };
}

// Web Mercator 瓦片 x/y/z → 瓦片西北角经纬度（度）。
inline void tile_to_latlon(std::uint32_t z, std::uint32_t x, std::uint32_t y,
                           double& north_lat, double& west_lon) noexcept {
    const double n = std::ldexp(1.0, static_cast<int>(z));
    west_lon = static_cast<double>(x) / n * 360.0 - 180.0;
    const double lat_rad = std::atan(std::sinh(M_PI * (1.0 - 2.0 * static_cast<double>(y) / n)));
    north_lat = lat_rad * 180.0 / M_PI;
}

}  // namespace spatial_map
