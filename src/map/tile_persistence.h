// 瓦片版本链持久化：把已登记瓦片 + 图版本序列化为可恢复的文本格式。
//
// 这是"元宇宙持久空间地图"索引层的最小雏形：
//   - 头部记录 graph_version 与缩放级别；
//   - 每行一个瓦片 z/x/y；
//   - 加载时逐行幂等恢复（重复行拒绝），保证与内存状态一致。
//
// 格式（第一行版本头，后续为瓦片）：
//   spatial-map 1
//   version <graph_version>
//   zoom <z>
//   tile <x> <y>
#pragma once

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>

#include "hash_tile_store.h"
#include "tile_math.h"

namespace spatial_map {

// 把瓦片集合 + 版本号写入文件。返回写入的瓦片数；失败返回 -1。
template <std::size_t N>
[[nodiscard]] inline std::int64_t save_tiles(const HashTileStore<N>& store,
                                             std::uint64_t graph_version,
                                             const std::string& path) noexcept {
    std::FILE* f = std::fopen(path.c_str(), "wb");
    if (f == nullptr) return -1;

    std::uint32_t zoom = 0;
    store.for_each([&zoom](const TileKey& k) { zoom = k.z; });

    if (std::fprintf(f, "spatial-map 1\nversion %llu\nzoom %u\n",
                     static_cast<unsigned long long>(graph_version), zoom) < 0) {
        std::fclose(f);
        return -1;
    }

    std::int64_t count = 0;
    store.for_each([&](const TileKey& k) {
        if (std::fprintf(f, "tile %u %u\n", k.x, k.y) >= 0) ++count;
    });

    std::fclose(f);
    return count;
}

// 从文件恢复瓦片集合 + 版本号。返回恢复的瓦片数；失败返回 -1。
template <std::size_t N>
[[nodiscard]] inline std::int64_t load_tiles(HashTileStore<N>& store,
                                             std::uint64_t& graph_version,
                                             const std::string& path) noexcept {
    std::FILE* f = std::fopen(path.c_str(), "rb");
    if (f == nullptr) return -1;

    char line[256];
    std::uint32_t zoom = 0;
    std::int64_t count = 0;

    while (std::fgets(line, sizeof(line), f) != nullptr) {
        if (std::sscanf(line, "version %llu",
                        reinterpret_cast<unsigned long long*>(&graph_version)) == 1) {
            continue;
        }
        if (std::sscanf(line, "zoom %u", &zoom) == 1) continue;
        std::uint32_t x = 0, y = 0;
        if (std::sscanf(line, "tile %u %u", &x, &y) == 2) {
            if (store.insert(TileKey{zoom, x, y})) ++count;
        }
    }

    std::fclose(f);
    return count;
}

}  // namespace spatial_map
