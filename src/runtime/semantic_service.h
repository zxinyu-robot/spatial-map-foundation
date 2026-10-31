// 语义瓦片查询服务（只读）：把语义地图暴露给边侧 / 端侧消费。
//
// 为什么需要它：语义瓦片存在 C++ 内部结构里（SemanticTileStore），
// 而消费方是 Python（edge-semantic-gateway 的 qwen_vl）。没有跨语言
// 查询接口，老师（VLM）的语义理解就无法落回地图、更无法传导给端侧学生。
// 这是「老师能力 → 语义瓦片 → 端侧策略」链路的唯一通道。
//
// 协议：TCP 行分隔 JSON，一问一答。
//   {"cmd":"info"}
//     → {"ok":true,"map_version":7,"zoom":18,"tile_count":3,
//        "anchor":{"lat":30.2741,"lon":120.1551}}
//   {"cmd":"tiles","categories":["kitchen"],"min_confidence":60}
//     → {"ok":true,"map_version":7,"count":1,
//        "tiles":[{"x":..,"y":..,"z":18,"tags":[{"cat":"kitchen","conf":90}]}]}
//   {"cmd":"query","lat":30.274,"lon":120.155,"radius_m":500,
//    "categories":["kitchen"],"min_confidence":60}
//     → 同 tiles，但每条多带 "dist_m"，且按距离升序
//     **lat/lon 为 WGS84**（机器人位姿空间）；内部会换算到瓦片的 GCJ-02 空间。
//
// 设计口径：
// - **只读**：服务不改瓦片。老师标注回写走文件，由上层显式触发，
//   避免「查询服务」意外变成「写服务」而破坏可回滚性。
// - **map_version 每次响应必带**：消费方必须自行校验版本一致，
//   否则会拿 v45 的坐标去配 v42 的语义。
// - **有界结果**：kMaxQueryResults 上限，超出部分丢弃并置 truncated。
// - 类别用字符串而非枚举序号：Python 侧不需要同步枚举定义。
#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "map/amap_tile.h"
#include "map/semantic_tile.h"
#include "map/tile_math.h"

namespace spatial_map {

inline constexpr std::size_t kMaxQueryResults = 256;
inline constexpr std::size_t kServiceReplyMax = 16384;

// 查询命中项：瓦片 + 到查询点的距离（米）。
struct SemanticHit {
    TileKey key{};
    const SemanticTile* tile{nullptr};
    double dist_m{0.0};
};

// 类别过滤：空表示不过滤。
// filter 用名字字符串，跨语言友好；nullptr 或 count==0 表示全部。
struct CategoryFilter {
    const char* const* names{nullptr};
    std::size_t count{0};

    [[nodiscard]] bool allows(SemanticCategory c) const noexcept {
        if (names == nullptr || count == 0) return true;
        const char* target = semantic_name(c);
        if (target == nullptr) return false;
        for (std::size_t i = 0; i < count; ++i) {
            const char* n = names[i];
            if (n == nullptr) continue;
            const char* a = n;
            const char* b = target;
            while (*a && *b && *a == *b) { ++a; ++b; }
            if (*a == '\0' && *b == '\0') return true;
        }
        return false;
    }
};

// 球面距离（米）。半径几百米量级，haversine 足够。
[[nodiscard]] inline double haversine_m(double lat1, double lon1,
                                        double lat2, double lon2) noexcept {
    constexpr double kEarthR = 6371000.0;
    constexpr double kDeg2Rad = 3.14159265358979323846 / 180.0;
    const double p1 = lat1 * kDeg2Rad;
    const double p2 = lat2 * kDeg2Rad;
    const double dp = (lat2 - lat1) * kDeg2Rad;
    const double dl = (lon2 - lon1) * kDeg2Rad;
    const double a = std::sin(dp / 2.0) * std::sin(dp / 2.0) +
                     std::cos(p1) * std::cos(p2) *
                         std::sin(dl / 2.0) * std::sin(dl / 2.0);
    return 2.0 * kEarthR * std::atan2(std::sqrt(a), std::sqrt(1.0 - a));
}

// 瓦片中心经纬度，**GCJ-02 空间**（与高德瓦片索引一致）。
//
// 注意：瓦片是用 wgs84_to_amap_tile() 生成的，索引空间是 GCJ-02。
// 直接把它当 WGS84 用会产生约 500m 的系统性偏移 —— 这是本层最容易踩的坑。
inline void tile_center_gcj02(const TileKey& k, double& lat, double& lon) noexcept {
    double north = 0.0, west = 0.0;
    tile_to_latlon(k.z, k.x, k.y, north, west);
    double south = 0.0, east = 0.0;
    tile_to_latlon(k.z, k.x + 1U, k.y + 1U, south, east);
    lat = (north + south) / 2.0;
    lon = (west + east) / 2.0;
}

// 瓦片中心经纬度，**WGS84 空间**（机器人位姿所在空间）。
// 查询距离必须用这个，否则 GCJ-02 偏移会伪装成"目标远了 500 米"。
inline void tile_center_wgs84(const TileKey& k, double& lat, double& lon) noexcept {
    double g_lat = 0.0, g_lon = 0.0;
    tile_center_gcj02(k, g_lat, g_lon);
    const LatLon w = gcj02_to_wgs84(g_lat, g_lon);
    lat = w.lat;
    lon = w.lon;
}

// 兼容旧名（GCJ-02）。
inline void tile_center_latlon(const TileKey& k, double& lat, double& lon) noexcept {
    tile_center_gcj02(k, lat, lon);
}

// 半径查询：结果按距离升序。
// 容量取 min(M, kMaxQueryResults)：调用方可给更小的缓冲，但不会被放大。
// 返回命中数；truncated 置 true 表示还有更多未返回。
template <std::size_t N, std::size_t M>
[[nodiscard]] inline std::size_t query_radius(
    const SemanticTileStore<N>& store,
    double lat, double lon, double radius_m,
    const CategoryFilter& filter, std::uint8_t min_confidence,
    SemanticHit (&hits)[M],
    bool& truncated) noexcept {
    truncated = false;
    std::size_t found = 0;
    constexpr std::size_t kCapacity = (M < kMaxQueryResults) ? M : kMaxQueryResults;

    store.for_each([&](const SemanticTile& t) {
        // 类别 + 置信度过滤
        bool any = false;
        for (std::uint8_t i = 0; i < t.tag_count; ++i) {
            if (t.tags[i].confidence < min_confidence) continue;
            if (!filter.allows(t.tags[i].category)) continue;
            any = true;
            break;
        }
        if (!any) return;

        double c_lat = 0.0, c_lon = 0.0;
        tile_center_wgs84(t.key, c_lat, c_lon);   // 查询点是 WGS84，必须同空间比
        const double d = haversine_m(lat, lon, c_lat, c_lon);
        if (d > radius_m) return;

        if (found >= kCapacity) {
            truncated = true;
            return;
        }

        // 插入排序（有界、无动态分配）
        std::size_t pos = found;
        while (pos > 0 && hits[pos - 1].dist_m > d) {
            hits[pos] = hits[pos - 1];
            --pos;
        }
        hits[pos] = SemanticHit{t.key, &t, d};
        ++found;
    });

    return found;
}

namespace detail {

// 有界 JSON 追加器：写满即置 overflow，不越界。
// 各 append 不标 [[nodiscard]] —— 溢出由 overflow 标记统一上报，
// 调用点无需逐次检查返回值。
struct JsonBuf {
    char* data;
    std::size_t cap;
    std::size_t len{0};
    bool overflow{false};

    bool append(const char* s) noexcept {
        if (overflow) return false;
        const std::size_t n = std::strlen(s);
        if (len + n + 1 > cap) { overflow = true; return false; }
        std::memcpy(data + len, s, n);
        len += n;
        data[len] = '\0';
        return true;
    }

    bool append_uint(std::uint64_t v) noexcept {
        char tmp[32];
        std::snprintf(tmp, sizeof(tmp), "%llu", static_cast<unsigned long long>(v));
        return append(tmp);
    }

    bool append_int(std::int64_t v) noexcept {
        char tmp[32];
        std::snprintf(tmp, sizeof(tmp), "%lld", static_cast<long long>(v));
        return append(tmp);
    }

    bool append_double(double v, int prec = 6) noexcept {
        char tmp[48];
        std::snprintf(tmp, sizeof(tmp), "%.*f", prec, v);
        return append(tmp);
    }
};

inline void append_tile_tags(JsonBuf& b, const SemanticHit& h) noexcept {
    b.append("\"tags\":[");
    for (std::uint8_t i = 0; i < h.tile->tag_count; ++i) {
        if (i > 0) b.append(",");
        b.append("{\"cat\":\"");
        b.append(semantic_name(h.tile->tags[i].category));
        b.append("\",\"conf\":");
        b.append_uint(h.tile->tags[i].confidence);
        b.append("}");
    }
    b.append("]");
}

}  // namespace detail

// info 响应。
[[nodiscard]] inline bool build_info_json(
    char (&out)[kServiceReplyMax],
    std::uint32_t map_version, std::uint32_t zoom, std::size_t tile_count,
    double anchor_lat, double anchor_lon) noexcept {
    detail::JsonBuf b{out, sizeof(out)};
    b.append("{\"ok\":true,\"map_version\":");
    b.append_uint(map_version);
    b.append(",\"zoom\":");
    b.append_uint(zoom);
    b.append(",\"tile_count\":");
    b.append_uint(tile_count);
    b.append(",\"anchor\":{\"lat\":");
    b.append_double(anchor_lat, 9);
    b.append(",\"lon\":");
    b.append_double(anchor_lon, 9);
    b.append("}}");
    return !b.overflow;
}

// 命中列表响应。with_dist=true 时每条带 dist_m。
[[nodiscard]] inline bool build_hits_json(
    char (&out)[kServiceReplyMax],
    std::uint32_t map_version,
    const SemanticHit* hits, std::size_t count, bool truncated,
    bool with_dist) noexcept {
    detail::JsonBuf b{out, sizeof(out)};
    b.append("{\"ok\":true,\"map_version\":");
    b.append_uint(map_version);
    b.append(",\"count\":");
    b.append_uint(count);
    b.append(",\"truncated\":");
    b.append(truncated ? "true" : "false");
    b.append(",\"tiles\":[");
    for (std::size_t i = 0; i < count; ++i) {
        if (i > 0) b.append(",");
        b.append("{\"z\":");
        b.append_uint(hits[i].key.z);
        b.append(",\"x\":");
        b.append_uint(hits[i].key.x);
        b.append(",\"y\":");
        b.append_uint(hits[i].key.y);
        if (with_dist) {
            b.append(",\"dist_m\":");
            b.append_double(hits[i].dist_m, 2);
        }
        b.append(",");
        detail::append_tile_tags(b, hits[i]);
        b.append("}");
    }
    b.append("]}");
    return !b.overflow;
}

[[nodiscard]] inline bool build_error_json(char (&out)[kServiceReplyMax],
                                           const char* message) noexcept {
    detail::JsonBuf b{out, sizeof(out)};
    b.append("{\"ok\":false,\"error\":\"");
    b.append(message != nullptr ? message : "unknown");
    b.append("\"}");
    return !b.overflow;
}

// ---------- 极简 JSON 取值（够用即可，不引第三方库） ----------

namespace detail {

// 取字符串字段值到定长缓冲；找不到返回 false。
[[nodiscard]] inline bool json_get_string(const char* json, const char* key,
                                          char* out, std::size_t cap) noexcept {
    char pat[64];
    std::snprintf(pat, sizeof(pat), "\"%s\"", key);
    const char* p = std::strstr(json, pat);
    if (p == nullptr) return false;
    p += std::strlen(pat);
    while (*p == ' ' || *p == ':') ++p;
    if (*p != '"') return false;
    ++p;
    std::size_t i = 0;
    while (*p && *p != '"' && i + 1 < cap) out[i++] = *p++;
    out[i] = '\0';
    return true;
}

// 取数字字段；找不到返回 false。
[[nodiscard]] inline bool json_get_double(const char* json, const char* key,
                                          double& out) noexcept {
    char pat[64];
    std::snprintf(pat, sizeof(pat), "\"%s\"", key);
    const char* p = std::strstr(json, pat);
    if (p == nullptr) return false;
    p += std::strlen(pat);
    while (*p == ' ' || *p == ':') ++p;
    if (*p == '\0') return false;
    out = std::strtod(p, nullptr);
    return true;
}

// 取字符串数组字段（如 categories:["kitchen","door"]）。
// 返回元素数；每个元素拷到 items[i]（每项 cap 字节）。
[[nodiscard]] inline std::size_t json_get_string_array(
    const char* json, const char* key,
    char (*items)[32], std::size_t max_items) noexcept {
    char pat[64];
    std::snprintf(pat, sizeof(pat), "\"%s\"", key);
    const char* p = std::strstr(json, pat);
    if (p == nullptr) return 0;
    p += std::strlen(pat);
    while (*p == ' ' || *p == ':') ++p;
    if (*p != '[') return 0;
    ++p;

    std::size_t n = 0;
    while (*p && *p != ']' && n < max_items) {
        while (*p == ' ' || *p == ',') ++p;
        if (*p != '"') break;
        ++p;
        std::size_t i = 0;
        while (*p && *p != '"' && i + 1 < 32) items[n][i++] = *p++;
        items[n][i] = '\0';
        if (*p == '"') ++p;
        ++n;
    }
    return n;
}

}  // namespace detail

}  // namespace spatial_map
