// 语义标注 overlay 的解析与应用（可单测的核心逻辑）.
//
// 从 tools/semantic_label_merge.cpp 抽出，目的是让「版本校验」这条
// 安全属性可被单元测试覆盖 —— 它是整个回写链路唯一能防止
// 「旧图标注贴到新图」的闸门，不能只靠端到端手测。
//
// 一条 overlay 的字段：
//   map_version  必须与 base 相等，否则丢弃（宁可少标，不可错标）
//   lat / lon    WGS84（机器人位姿空间），落瓦片时换到 GCJ-02
//   category     semantic_name() 可识别的类别名
//   confidence   0..100，缺省 70（低于 SLAM 实测，合并不压真实测）
#pragma once

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "map/amap_tile.h"
#include "map/semantic_tile.h"

namespace spatial_map {

// 老师标注的默认置信度：可信但低于传感器实测。
inline constexpr std::uint8_t kTeacherDefaultConfidence = 70;

struct OverlayStats {
    std::int64_t applied{0};
    std::int64_t skipped_version{0};   // map_version 不匹配
    std::int64_t skipped_bad{0};       // 字段缺失 / 类别未知 / 坐标越界
    std::int64_t skipped_capacity{0};  // 标签或瓦片容量已满
};

namespace detail {

[[nodiscard]] inline bool ov_get_string(const char* json, const char* key,
                                        char* out, std::size_t cap) noexcept {
    char pat[48];
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

[[nodiscard]] inline bool ov_get_double(const char* json, const char* key,
                                        double& out) noexcept {
    char pat[48];
    std::snprintf(pat, sizeof(pat), "\"%s\"", key);
    const char* p = std::strstr(json, pat);
    if (p == nullptr) return false;
    p += std::strlen(pat);
    while (*p == ' ' || *p == ':') ++p;
    if (*p == '\0') return false;
    out = std::strtod(p, nullptr);
    return true;
}

}  // namespace detail

// 应用一条 overlay 到 store。返回该条是否被应用。
template <std::size_t N>
[[nodiscard]] inline bool apply_overlay_entry(
    const char* line,
    SemanticTileStore<N>& store,
    std::uint32_t base_version,
    std::uint32_t zoom,
    OverlayStats& stats) noexcept {
    if (line == nullptr) {
        ++stats.skipped_bad;
        return false;
    }

    // --- 版本闸门：第一道，失败即丢 ---
    double mv_d = 0.0;
    if (!detail::ov_get_double(line, "map_version", mv_d)) {
        ++stats.skipped_bad;
        return false;
    }
    const auto mv = static_cast<std::uint32_t>(mv_d);
    if (base_version == 0 || mv != base_version) {
        ++stats.skipped_version;
        return false;
    }

    // --- 字段解析 ---
    double lat = 0.0, lon = 0.0;
    char category[48] = {};
    if (!detail::ov_get_double(line, "lat", lat) ||
        !detail::ov_get_double(line, "lon", lon) ||
        !detail::ov_get_string(line, "category", category, sizeof(category))) {
        ++stats.skipped_bad;
        return false;
    }
    if (lat < -90.0 || lat > 90.0 || lon < -180.0 || lon > 180.0) {
        ++stats.skipped_bad;
        return false;
    }

    const SemanticCategory cat = semantic_from_name(category);
    if (cat == SemanticCategory::Unknown) {
        ++stats.skipped_bad;
        return false;
    }

    double conf = static_cast<double>(kTeacherDefaultConfidence);
    double conf_raw = 0.0;
    if (detail::ov_get_double(line, "confidence", conf_raw)) conf = conf_raw;
    if (conf < 0.0) conf = 0.0;
    if (conf > 100.0) conf = 100.0;

    // --- 落瓦片：WGS84 → GCJ-02，换算只有这一份实现 ---
    const TileKey key = wgs84_to_amap_tile(lat, lon, zoom);

    SemanticTile t;
    t.key = key;
    if (!t.add_tag(cat, static_cast<std::uint8_t>(conf))) {
        ++stats.skipped_capacity;
        return false;
    }
    if (!store.insert(t)) {
        ++stats.skipped_capacity;
        return false;
    }
    ++stats.applied;
    return true;
}

}  // namespace spatial_map
