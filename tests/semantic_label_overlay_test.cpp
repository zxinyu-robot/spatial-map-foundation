// 语义标注回写测试：版本闸门、坐标落瓦片、置信度合并策略.
//
// 版本闸门是整个回写链路唯一能防止「旧图标注贴到新图」的关卡，
// 必须单测覆盖，不能只靠端到端手测。
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <iostream>

#include "map/amap_tile.h"
#include "map/semantic_tile.h"
#include "runtime/semantic_label_overlay.h"

namespace {

int g_failures = 0;

#define CHECK(cond)                                                      \
    do {                                                                 \
        if (!(cond)) {                                                   \
            std::cerr << "  [FAIL] " << #cond << " @ line " << __LINE__  \
                      << "\n";                                           \
            ++g_failures;                                                \
        }                                                                \
    } while (0)

using namespace spatial_map;

constexpr double kLat = 30.2741;
constexpr double kLon = 120.1551;
constexpr std::uint32_t kZoom = 18;

void test_version_gate() {
    SemanticTileStore<64> s;
    OverlayStats st;

    // 版本匹配 → 应用
    const char* ok =
        R"({"map_version":7,"lat":30.2741,"lon":120.1551,"category":"kitchen","confidence":85})";
    CHECK(apply_overlay_entry(ok, s, 7, kZoom, st));
    CHECK(st.applied == 1);

    // 版本不匹配 → 丢弃（这是最关键的一条）
    const char* wrong =
        R"({"map_version":6,"lat":30.2741,"lon":120.1551,"category":"bedroom","confidence":99})";
    CHECK(!apply_overlay_entry(wrong, s, 7, kZoom, st));
    CHECK(st.skipped_version == 1);
    CHECK(st.applied == 1);

    // base_version=0（未知）→ 一律丢弃
    const char* v0 =
        R"({"map_version":0,"lat":30.2741,"lon":120.1551,"category":"door","confidence":80})";
    CHECK(!apply_overlay_entry(v0, s, 0, kZoom, st));
    CHECK(st.skipped_version == 2);

    // 被丢弃的条目不得写进 store
    const TileKey k = wgs84_to_amap_tile(kLat, kLon, kZoom);
    const SemanticTile* t = s.find(k);
    CHECK(t != nullptr);
    if (t != nullptr) {
        CHECK(t->confidence_of(SemanticCategory::Kitchen) == 85);
        CHECK(t->confidence_of(SemanticCategory::Bedroom) == 0);   // 版本不符被拒
        CHECK(t->confidence_of(SemanticCategory::Door) == 0);      // v0 被拒
    }
    std::cout << "  [ok] version gate\n";
}

void test_missing_version_field() {
    SemanticTileStore<64> s;
    OverlayStats st;
    const char* no_ver = R"({"lat":30.2741,"lon":120.1551,"category":"kitchen"})";
    CHECK(!apply_overlay_entry(no_ver, s, 7, kZoom, st));
    CHECK(st.skipped_bad == 1);
    std::cout << "  [ok] missing map_version rejected\n";
}

void test_unknown_category() {
    SemanticTileStore<64> s;
    OverlayStats st;
    const char* bad =
        R"({"map_version":7,"lat":30.2741,"lon":120.1551,"category":"nonsense","confidence":80})";
    CHECK(!apply_overlay_entry(bad, s, 7, kZoom, st));
    CHECK(st.skipped_bad == 1);

    const char* missing_cat = R"({"map_version":7,"lat":30.2741,"lon":120.1551})";
    CHECK(!apply_overlay_entry(missing_cat, s, 7, kZoom, st));
    CHECK(st.skipped_bad == 2);
    std::cout << "  [ok] unknown category rejected\n";
}

void test_coordinate_out_of_range() {
    SemanticTileStore<64> s;
    OverlayStats st;
    const char* bad_lat =
        R"({"map_version":7,"lat":999.0,"lon":120.1551,"category":"kitchen"})";
    CHECK(!apply_overlay_entry(bad_lat, s, 7, kZoom, st));
    CHECK(st.skipped_bad == 1);

    const char* bad_lon =
        R"({"map_version":7,"lat":30.2741,"lon":-999.0,"category":"kitchen"})";
    CHECK(!apply_overlay_entry(bad_lon, s, 7, kZoom, st));
    CHECK(st.skipped_bad == 2);
    std::cout << "  [ok] out-of-range coordinate rejected\n";
}

void test_lands_on_correct_tile() {
    // WGS84 → GCJ-02 落瓦片：同一经纬度必须落到 amap_tile 算出的那一格
    SemanticTileStore<64> s;
    OverlayStats st;
    const char* line =
        R"({"map_version":7,"lat":30.2741,"lon":120.1551,"category":"kitchen","confidence":85})";
    CHECK(apply_overlay_entry(line, s, 7, kZoom, st));

    const TileKey expected = wgs84_to_amap_tile(kLat, kLon, kZoom);
    const SemanticTile* t = s.find(expected);
    CHECK(t != nullptr);
    CHECK(s.size() == 1);
    std::cout << "  [ok] lands on correct tile via GCJ-02\n";
}

void test_default_confidence() {
    // 缺省置信度应为 70（低于 SLAM 实测，合并不压真实测）
    SemanticTileStore<64> s;
    OverlayStats st;
    const char* no_conf =
        R"({"map_version":7,"lat":30.2741,"lon":120.1551,"category":"kitchen"})";
    CHECK(apply_overlay_entry(no_conf, s, 7, kZoom, st));
    const TileKey k = wgs84_to_amap_tile(kLat, kLon, kZoom);
    const SemanticTile* t = s.find(k);
    CHECK(t != nullptr);
    if (t != nullptr) {
        CHECK(t->confidence_of(SemanticCategory::Kitchen) ==
              static_cast<std::uint8_t>(kTeacherDefaultConfidence));
    }
    std::cout << "  [ok] default confidence = "
              << static_cast<int>(kTeacherDefaultConfidence) << "\n";
}

void test_confidence_clamped() {
    SemanticTileStore<64> s;
    OverlayStats st;
    const char* over =
        R"({"map_version":7,"lat":30.2741,"lon":120.1551,"category":"kitchen","confidence":500})";
    CHECK(apply_overlay_entry(over, s, 7, kZoom, st));
    const TileKey k = wgs84_to_amap_tile(kLat, kLon, kZoom);
    CHECK(s.find(k)->confidence_of(SemanticCategory::Kitchen) == 100);

    const char* neg =
        R"({"map_version":7,"lat":30.2741,"lon":120.1551,"category":"door","confidence":-50})";
    CHECK(apply_overlay_entry(neg, s, 7, kZoom, st));
    CHECK(s.find(k)->confidence_of(SemanticCategory::Door) == 0);
    std::cout << "  [ok] confidence clamped to 0..100\n";
}

void test_merge_keeps_higher_confidence() {
    // 老师低置信标注不得压掉 SLAM 高置信实测
    SemanticTileStore<64> s;
    SemanticTile base;
    const TileKey k = wgs84_to_amap_tile(kLat, kLon, kZoom);
    base.key = k;
    base.add_tag(SemanticCategory::Kitchen, 95);   // SLAM 实测
    s.insert(base);

    OverlayStats st;
    const char* teacher =
        R"({"map_version":7,"lat":30.2741,"lon":120.1551,"category":"kitchen","confidence":60})";
    CHECK(apply_overlay_entry(teacher, s, 7, kZoom, st));
    CHECK(s.find(k)->confidence_of(SemanticCategory::Kitchen) == 95);  // 未被压低

    // 老师高置信标注则更新
    const char* strong =
        R"({"map_version":7,"lat":30.2741,"lon":120.1551,"category":"kitchen","confidence":99})";
    CHECK(apply_overlay_entry(strong, s, 7, kZoom, st));
    CHECK(s.find(k)->confidence_of(SemanticCategory::Kitchen) == 99);
    std::cout << "  [ok] merge keeps higher confidence\n";
}

void test_tag_capacity() {
    // 单瓦片标签上限 kMaxTagsPerTile；超出的计入 skipped_capacity
    SemanticTileStore<64> s;
    OverlayStats st;
    const SemanticCategory cats[] = {
        SemanticCategory::Kitchen, SemanticCategory::Door,
        SemanticCategory::Corridor, SemanticCategory::Bedroom,
        SemanticCategory::LivingRoom, SemanticCategory::Bathroom,
        SemanticCategory::Staircase, SemanticCategory::Elevator,
        SemanticCategory::Obstacle,   // 第 9 个，超容量
    };
    char line[256];
    for (SemanticCategory c : cats) {
        std::snprintf(line, sizeof(line),
                      R"({"map_version":7,"lat":30.2741,"lon":120.1551,"category":"%s","confidence":80})",
                      semantic_name(c));
        (void)apply_overlay_entry(line, s, 7, kZoom, st);
    }
    CHECK(st.applied == static_cast<std::int64_t>(kMaxTagsPerTile));
    CHECK(st.skipped_capacity == 1);

    const TileKey k = wgs84_to_amap_tile(kLat, kLon, kZoom);
    CHECK(s.find(k)->tag_count == kMaxTagsPerTile);
    std::cout << "  [ok] tag capacity bounded (" << kMaxTagsPerTile << ")\n";
}

void test_null_line() {
    SemanticTileStore<64> s;
    OverlayStats st;
    CHECK(!apply_overlay_entry(nullptr, s, 7, kZoom, st));
    CHECK(st.skipped_bad == 1);
    std::cout << "  [ok] null line rejected\n";
}

void test_different_positions_different_tiles() {
    // 三个不同位置应落到三个不同瓦片（端到端闭环已验证过，此处固化为单测）
    SemanticTileStore<64> s;
    OverlayStats st;
    const char* l1 =
        R"({"map_version":7,"lat":30.2741,"lon":120.1551,"category":"kitchen","confidence":85})";
    const char* l2 =
        R"({"map_version":7,"lat":30.2741,"lon":120.1562,"category":"bathroom","confidence":85})";
    const char* l3 =
        R"({"map_version":7,"lat":30.2741,"lon":120.1540,"category":"corridor","confidence":85})";
    CHECK(apply_overlay_entry(l1, s, 7, kZoom, st));
    CHECK(apply_overlay_entry(l2, s, 7, kZoom, st));
    CHECK(apply_overlay_entry(l3, s, 7, kZoom, st));
    CHECK(st.applied == 3);
    CHECK(s.size() == 3);   // 落在三格不同瓦片
    std::cout << "  [ok] distinct positions -> distinct tiles\n";
}

}  // namespace

int main() {
    std::cout << "== semantic label overlay tests ==\n";
    test_version_gate();
    test_missing_version_field();
    test_unknown_category();
    test_coordinate_out_of_range();
    test_lands_on_correct_tile();
    test_default_confidence();
    test_confidence_clamped();
    test_merge_keeps_higher_confidence();
    test_tag_capacity();
    test_null_line();
    test_different_positions_different_tiles();

    if (g_failures != 0) {
        std::cerr << "== " << g_failures << " failure(s) ==\n";
        return 1;
    }
    std::cout << "== all passed ==\n";
    return 0;
}
