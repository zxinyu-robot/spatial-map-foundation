// 语义瓦片序列化：对接真机拿数据 + 供上游消费。
//
// 格式 v2（每行一个语义瓦片，首行带地图版本）：
//   semantic-tile 2 <zoom> <map_version>
//   tile <x> <y> <category> <conf> [<category> <conf> ...]
//
// 兼容读取 v1（无语义版本）：
//   semantic-tile 1 <zoom>
//   → map_version 读为 0（未知），调用方自行决定是否接受。
//
// map_version 与 ScenePack manifest / PoseGraph 版本同源，
// 是「图 — 语义 — 蒸馏样本」三者对齐的唯一依据。
#pragma once

#include <cstdint>
#include <cstdio>
#include <string>

#include "semantic_tile.h"
#include "tile_math.h"

namespace spatial_map {

inline constexpr std::uint32_t kSemanticTileFormatVersion = 2;

template <std::size_t N>
[[nodiscard]] inline std::int64_t save_semantic_tiles(
    const SemanticTileStore<N>& store, std::uint32_t zoom,
    const std::string& path,
    std::uint32_t map_version = 0) noexcept {
    std::FILE* f = std::fopen(path.c_str(), "wb");
    if (f == nullptr) return -1;

    if (std::fprintf(f, "semantic-tile %u %u %u\n", kSemanticTileFormatVersion,
                     zoom, map_version) < 0) {
        std::fclose(f);
        return -1;
    }

    std::int64_t count = 0;
    store.for_each([&](const SemanticTile& t) {
        std::fprintf(f, "tile %u %u", t.key.x, t.key.y);
        for (std::uint8_t i = 0; i < t.tag_count; ++i) {
            std::fprintf(f, " %s %u", semantic_name(t.tags[i].category),
                         t.tags[i].confidence);
        }
        std::fprintf(f, "\n");
        ++count;
    });

    std::fclose(f);
    return count;
}

// map_version 可为 nullptr；读取 v1 时写 0。
template <std::size_t N>
[[nodiscard]] inline std::int64_t load_semantic_tiles(
    SemanticTileStore<N>& store, std::uint32_t& zoom,
    const std::string& path,
    std::uint32_t* map_version = nullptr) noexcept {
    std::FILE* f = std::fopen(path.c_str(), "rb");
    if (f == nullptr) return -1;

    char line[512];
    std::int64_t count = 0;
    zoom = 0;
    if (map_version != nullptr) *map_version = 0;

    while (std::fgets(line, sizeof(line), f) != nullptr) {
        unsigned int ver = 0, z = 0, mv = 0;
        const int n = std::sscanf(line, "semantic-tile %u %u %u", &ver, &z, &mv);
        if (n >= 2) {
            if (ver != 1 && ver != 2) continue;  // 未知格式版本，跳过
            zoom = z;
            if (map_version != nullptr) *map_version = (ver >= 2 && n == 3) ? mv : 0U;
            continue;
        }
        std::uint32_t x = 0, y = 0;
        int off = 0;
        if (std::sscanf(line, "tile %u %u%n", &x, &y, &off) == 2) {
            SemanticTile t;
            t.key = TileKey{zoom, x, y};
            // 解析后续 "category conf" 对
            char cat[64];
            unsigned int conf = 0;
            int consumed = 0;
            const char* p = line + off;
            while (std::sscanf(p, " %63s %u%n", cat, &conf, &consumed) == 2) {
                const SemanticCategory c = semantic_from_name(cat);
                if (c != SemanticCategory::Unknown) {
                    t.add_tag(c, static_cast<std::uint8_t>(conf > 100 ? 100 : conf));
                }
                p += consumed;
                if (*p == '\n' || *p == '\0') break;
            }
            if (store.insert(t)) ++count;
        }
    }

    std::fclose(f);
    return count;
}

}  // namespace spatial_map
