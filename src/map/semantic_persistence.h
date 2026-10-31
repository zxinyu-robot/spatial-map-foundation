// 语义瓦片序列化：对接真机拿数据 + 供上游消费。
//
// 文本格式（每行一个语义瓦片）：
//   semantic-tile 1 <zoom>
//   tile <x> <y> <category> <conf> [<category> <conf> ...]
//
// 真机语义 SLAM 输出（每个对象一行）可直接转换成该格式，
// 再由本层加载为 SemanticTileStore，供边侧世界模型 / 端侧策略查询。
#pragma once

#include <cstdint>
#include <cstdio>
#include <string>

#include "semantic_tile.h"
#include "tile_math.h"

namespace spatial_map {

template <std::size_t N>
[[nodiscard]] inline std::int64_t save_semantic_tiles(
    const SemanticTileStore<N>& store, std::uint32_t zoom,
    const std::string& path) noexcept {
    std::FILE* f = std::fopen(path.c_str(), "wb");
    if (f == nullptr) return -1;

    if (std::fprintf(f, "semantic-tile 1 %u\n", zoom) < 0) {
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

template <std::size_t N>
[[nodiscard]] inline std::int64_t load_semantic_tiles(
    SemanticTileStore<N>& store, std::uint32_t& zoom,
    const std::string& path) noexcept {
    std::FILE* f = std::fopen(path.c_str(), "rb");
    if (f == nullptr) return -1;

    char line[512];
    std::int64_t count = 0;
    zoom = 0;

    while (std::fgets(line, sizeof(line), f) != nullptr) {
        std::uint32_t z = 0;
        if (std::sscanf(line, "semantic-tile %*d %u", &z) == 1) {
            zoom = z;
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
