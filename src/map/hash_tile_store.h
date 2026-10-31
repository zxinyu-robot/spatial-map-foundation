// 固定容量开放寻址哈希瓦片存储。
//
// 延续 MISRA 精神：有界资源、无动态分配、值语义、输入验证。
// 用 64 位瓦片哈希键做开放寻址（线性探测），把瓦片查询从 O(n) 降为 O(1) 均摊。
#pragma once

#include <cstdint>
#include <cstddef>
#include <optional>

#include "tile_math.h"

namespace spatial_map {

// 瓦片槽：value 为空表示空闲；墓碑槽在删除后参与探测但不命中。
struct TileSlot {
    std::uint64_t key{0};
    bool occupied{false};
    bool tombstone{false};
};

template <std::size_t N>
class HashTileStore {
    static_assert(N > 0, "capacity must be positive");

public:
    static constexpr std::size_t kCapacity = N;

    HashTileStore() noexcept = default;

    [[nodiscard]] std::size_t size() const noexcept { return size_; }
    [[nodiscard]] std::size_t capacity() const noexcept { return N; }
    [[nodiscard]] bool empty() const noexcept { return size_ == 0; }
    [[nodiscard]] bool full() const noexcept { return size_ == N; }

    // 插入瓦片；已存在返回 false（幂等拒绝，不覆盖）。
    bool insert(const TileKey& tile) noexcept {
        if (full()) return false;
        const std::uint64_t key = encode_tile_key(tile);
        const std::size_t idx = find_slot(key);
        if (slots_[idx].occupied) return false;  // 已存在，拒绝重复
        slots_[idx] = TileSlot{key, true, false};
        ++size_;
        return true;
    }

    // 查询瓦片是否已登记。
    [[nodiscard]] bool contains(const TileKey& tile) const noexcept {
        const std::size_t idx = find_slot(encode_tile_key(tile));
        return slots_[idx].occupied;
    }

    // 删除瓦片；用墓碑标记保证探测链不断。
    bool erase(const TileKey& tile) noexcept {
        const std::size_t idx = find_slot(encode_tile_key(tile));
        if (!slots_[idx].occupied) return false;
        slots_[idx].occupied = false;
        slots_[idx].tombstone = true;
        --size_;
        return true;
    }

    // 遍历所有已登记瓦片（按槽顺序，不保证有序）。
    template <typename Fn>
    void for_each(Fn&& fn) const noexcept {
        for (std::size_t i = 0; i < N; ++i) {
            if (slots_[i].occupied) {
                fn(decode_tile_key(slots_[i].key));
            }
        }
    }

private:
    [[nodiscard]] std::size_t hash(std::uint64_t key) const noexcept {
        // 乘法散列（Knuth 常数），再映射到容量。
        constexpr std::uint64_t kMul = 11400714819323198485ULL;
        std::uint64_t h = key * kMul;
        h ^= h >> 33U;
        return static_cast<std::size_t>(h % N);
    }

    [[nodiscard]] std::size_t find_slot(std::uint64_t key) const noexcept {
        std::size_t idx = hash(key);
        std::size_t first_tomb = N;  // 记录第一个墓碑，供插入复用
        for (std::size_t probe = 0; probe < N; ++probe) {
            const TileSlot& s = slots_[idx];
            if (s.occupied && s.key == key) return idx;      // 命中
            if (!s.occupied) {
                if (!s.tombstone) return idx;                 // 空槽，插入位
                if (first_tomb == N) first_tomb = idx;        // 记第一个墓碑
            }
            idx = (idx + 1U) % N;
        }
        // 表满且未命中：优先复用墓碑
        return (first_tomb != N) ? first_tomb : 0U;
    }

    TileSlot slots_[N]{};
    std::size_t size_{0};
};

}  // namespace spatial_map
