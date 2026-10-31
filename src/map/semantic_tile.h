// 语义瓦片层：在几何哈希瓦片之上挂语义标签。
//
// 对接真机的数据入口：语义 SLAM（如 Super-LIO + 语义分割）输出
// 每个检测对象的「局部位姿 + 语义类别 + 置信度」，经 geo_anchor
// 投影到瓦片后，把语义标签挂到对应瓦片，供上游（边侧世界模型 /
// 端侧策略）按瓦片查询消费。
//
// 设计延续有界资源 / 无动态分配：每个瓦片最多挂 kMaxTagsPerTile 个标签，
// 语义类别用枚举 + 名称表，置信度量化到 uint8（0..100）减少存储。
#pragma once

#include <cstdint>
#include <cstddef>

#include "tile_math.h"

namespace spatial_map {

// 语义类别（与 edge-semantic-gateway 的 frontier 语义对齐）。
enum class SemanticCategory : std::uint16_t {
    Unknown = 0,
    Corridor,
    Door,
    Kitchen,
    Bedroom,
    LivingRoom,
    Bathroom,
    Staircase,
    Elevator,
    Obstacle,
    FreeSpace,
    GoalObject,   // 开放目标（如"厨房里的冰箱"）
    kCount,
};

// 类别 → 小写字符串（序列化用，与真机语义 SLAM 标签对齐）。
[[nodiscard]] inline const char* semantic_name(SemanticCategory c) noexcept {
    switch (c) {
        case SemanticCategory::Corridor:   return "corridor";
        case SemanticCategory::Door:       return "door";
        case SemanticCategory::Kitchen:    return "kitchen";
        case SemanticCategory::Bedroom:    return "bedroom";
        case SemanticCategory::LivingRoom: return "living_room";
        case SemanticCategory::Bathroom:   return "bathroom";
        case SemanticCategory::Staircase:  return "staircase";
        case SemanticCategory::Elevator:   return "elevator";
        case SemanticCategory::Obstacle:   return "obstacle";
        case SemanticCategory::FreeSpace:  return "free_space";
        case SemanticCategory::GoalObject: return "goal_object";
        default:                           return "unknown";
    }
}

// 字符串 → 类别（真机语义 SLAM 标签解析入口；未知返回 Unknown）。
[[nodiscard]] inline SemanticCategory semantic_from_name(const char* name) noexcept {
    if (name == nullptr) return SemanticCategory::Unknown;
    for (std::uint16_t i = 0; i < static_cast<std::uint16_t>(SemanticCategory::kCount); ++i) {
        const auto c = static_cast<SemanticCategory>(i);
        const char* n = semantic_name(c);
        if (n != nullptr) {
            // 简单前缀比较，足够匹配下划线标签
            const char* a = name;
            const char* b = n;
            while (*a && *b && *a == *b) { ++a; ++b; }
            if (*a == '\0' && *b == '\0') return c;
        }
    }
    return SemanticCategory::Unknown;
}

// 语义标签：类别 + 量化置信度（0..100）。
struct SemanticTag {
    SemanticCategory category{SemanticCategory::Unknown};
    std::uint8_t confidence{0};  // 0..100
};

inline constexpr std::size_t kMaxTagsPerTile = 8;

// 语义瓦片：几何瓦片 + 语义标签集合（固定容量，无动态分配）。
struct SemanticTile {
    TileKey key{};
    SemanticTag tags[kMaxTagsPerTile]{};
    std::uint8_t tag_count{0};

    // 追加标签；已存在同类别则取较高置信度。
    bool add_tag(SemanticCategory cat, std::uint8_t confidence) noexcept {
        for (std::uint8_t i = 0; i < tag_count; ++i) {
            if (tags[i].category == cat) {
                if (confidence > tags[i].confidence) tags[i].confidence = confidence;
                return true;
            }
        }
        if (tag_count >= kMaxTagsPerTile) return false;
        tags[tag_count] = SemanticTag{cat, confidence};
        ++tag_count;
        return true;
    }

    // 查询某类别置信度（未找到返回 0）。
    [[nodiscard]] std::uint8_t confidence_of(SemanticCategory cat) const noexcept {
        for (std::uint8_t i = 0; i < tag_count; ++i) {
            if (tags[i].category == cat) return tags[i].confidence;
        }
        return 0;
    }
};

// 语义瓦片存储：哈希瓦片 → 语义标签（开放寻址，复用 TileKey 哈希）。
// 每个槽存 SemanticTile；未占用槽 tag_count==0 且 key 为默认值。
template <std::size_t N>
class SemanticTileStore {
public:
    static constexpr std::size_t kCapacity = N;

    [[nodiscard]] std::size_t size() const noexcept { return size_; }
    [[nodiscard]] bool full() const noexcept { return size_ == N; }

    // 登记语义瓦片（幂等：已存在则合并标签）。
    bool insert(const SemanticTile& tile) noexcept {
        const std::uint64_t key = encode_tile_key(tile.key);
        const std::size_t idx = find_slot(key);
        if (!slots_[idx].occupied) {
            if (full()) return false;
            slots_[idx] = Slot{key, tile, true, false};
            ++size_;
            return true;
        }
        // 已存在：合并标签
        for (std::uint8_t i = 0; i < tile.tag_count; ++i) {
            slots_[idx].tile.add_tag(tile.tags[i].category, tile.tags[i].confidence);
        }
        return true;
    }

    // 查询瓦片的语义标签（返回指针，未找到返回 nullptr）。
    [[nodiscard]] const SemanticTile* find(const TileKey& tile) const noexcept {
        const std::size_t idx = find_slot(encode_tile_key(tile));
        return slots_[idx].occupied ? &slots_[idx].tile : nullptr;
    }

    template <typename Fn>
    void for_each(Fn&& fn) const noexcept {
        for (std::size_t i = 0; i < N; ++i) {
            if (slots_[i].occupied) fn(slots_[i].tile);
        }
    }

private:
    struct Slot {
        std::uint64_t key{0};
        SemanticTile tile{};
        bool occupied{false};
        bool tombstone{false};
    };

    [[nodiscard]] std::size_t hash(std::uint64_t key) const noexcept {
        constexpr std::uint64_t kMul = 11400714819323198485ULL;
        std::uint64_t h = key * kMul;
        h ^= h >> 33U;
        return static_cast<std::size_t>(h % N);
    }

    [[nodiscard]] std::size_t find_slot(std::uint64_t key) const noexcept {
        std::size_t idx = hash(key);
        std::size_t first_tomb = N;
        for (std::size_t probe = 0; probe < N; ++probe) {
            const Slot& s = slots_[idx];
            if (s.occupied && s.key == key) return idx;
            if (!s.occupied) {
                if (!s.tombstone) return idx;
                if (first_tomb == N) first_tomb = idx;
            }
            idx = (idx + 1U) % N;
        }
        return (first_tomb != N) ? first_tomb : 0U;
    }

    Slot slots_[N]{};
    std::size_t size_{0};
};

}  // namespace spatial_map
