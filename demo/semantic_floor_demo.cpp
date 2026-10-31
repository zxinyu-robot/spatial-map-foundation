// 语义地图消费 demo：楼层语义布局 + VLM 标注回写 + 可视化入口。
//
// 演示完整链路（对应 README「语义地图链路」）：
//   1. 楼层布局（ENU 米）→ 地理锚定 → GCJ-02 瓦片 → 语义标签
//   2. 模拟老师（VLM）识别出新房间 → overlay 回写 → map_version +1
//   3. 输出两份瓦片文件，交给 visualize_semantic_map.py 出图
//
// 缩放级选择：z20（约 37m/瓦片）接近"一间房"的尺度。
// 用 z18（约 150m/瓦片）会把整栋楼标成一个"厨房"，物理上不成立 ——
// 语义地图的瓦片粒度必须与语义对象的尺度匹配。
#include <cstdint>
#include <cstring>
#include <iostream>
#include <string>

#include "map/amap_tile.h"
#include "map/geo_anchor.h"
#include "map/semantic_persistence.h"
#include "map/semantic_tile.h"
#include "map/tile_math.h"
#include "runtime/semantic_label_overlay.h"

namespace {

using namespace spatial_map;

// 武林广场附近（与其它 demo 同一锚点）
constexpr double kAnchorLat = 30.2741;
constexpr double kAnchorLon = 120.1551;
constexpr std::uint32_t kZoom = 20;

// 一间房在 z20 下的近似尺寸：约 37m（纬向）/ 32m（经向）。
constexpr double kRoomM = 34.0;

SemanticTileStore<256> g_store;
std::uint32_t g_tile_count = 0;

// 局部 ENU（米，x=东 y=北）→ 语义瓦片并登记。
// 用「瓦片格网 ±dx/±dy」描述楼层，dy 正向为南（瓦片 y 增大方向）。
void place_cell(int dx, int dy,
                std::initializer_list<std::pair<SemanticCategory, std::uint8_t>> tags) {
    constexpr double kMetersPerDegLat = 111320.0;
    const double east_m = dx * kRoomM;
    const double north_m = -dy * kRoomM;      // 屏幕下方 = 南
    const double lat = kAnchorLat + north_m / kMetersPerDegLat;
    const double lon = kAnchorLon +
                       east_m / (kMetersPerDegLat * std::cos(kAnchorLat * M_PI / 180.0));

    const TileKey key = wgs84_to_amap_tile(lat, lon, kZoom);
    SemanticTile t;
    t.key = key;
    for (const auto& kv : tags) t.add_tag(kv.first, kv.second);
    if (g_store.insert(t)) ++g_tile_count;
}

void build_floor_plan() {
    // 东西向主走廊（dy=0），连接两侧房间
    for (int dx = -2; dx <= 2; ++dx) {
        place_cell(dx, 0, {{SemanticCategory::Corridor,
                            static_cast<std::uint8_t>(92 - std::abs(dx))}});
    }

    // 走廊北侧：卧室、厨房（含目标物）、卫生间
    place_cell(-2, -1, {{SemanticCategory::Bedroom, 87}});
    place_cell(-1, -1, {{SemanticCategory::Bedroom, 83}});
    place_cell(0, -1, {
        {SemanticCategory::Kitchen, 90},
        {SemanticCategory::GoalObject, 80},   // "厨房里的冰箱"
    });
    place_cell(1, -1, {{SemanticCategory::Bathroom, 84}});

    // 走廊南侧：客厅、卧室、楼梯间
    place_cell(-2, 1, {{SemanticCategory::LivingRoom, 88}});
    place_cell(-1, 1, {{SemanticCategory::LivingRoom, 82}});
    place_cell(0, 1, {{SemanticCategory::Bedroom, 86}});
    place_cell(1, 1, {{SemanticCategory::Bedroom, 81}});
    place_cell(2, -1, {{SemanticCategory::Staircase, 89}});
    place_cell(2, 1, {{SemanticCategory::Elevator, 82}});

    // 门：走廊与房间交界
    place_cell(0, 0, {
        {SemanticCategory::Corridor, 90},
        {SemanticCategory::Door, 85},
    });
    place_cell(1, 0, {
        {SemanticCategory::Corridor, 88},
        {SemanticCategory::Door, 80},
    });
}

// 模拟老师（VLM）走了两处，识别出未标注的区域并回写。
std::int64_t simulate_vlm_writeback(const std::string& overlay_path) {
    std::FILE* f = std::fopen(overlay_path.c_str(), "wb");
    if (f == nullptr) return -1;

    // 走廊北端尽头看到开阔区域；东侧走廊尽头看到另一间房。
    constexpr double kMetersPerDegLat = 111320.0;
    const double lon_scale = kMetersPerDegLat * std::cos(kAnchorLat * M_PI / 180.0);

    struct Sample {
        double east_m, north_m;
        const char* category;
        int confidence;
        const char* note;
    };
    // 走廊东端外：走廊延伸段 + 一间未标注的小房间
    const Sample samples[] = {
        {3.0 * kRoomM, 0.0, "corridor", 76, "走廊东端延伸"},
        {3.0 * kRoomM, -1.0 * kRoomM, "bathroom", 72, "东端北侧小卫生间"},
    };

    std::int64_t n = 0;
    for (const Sample& s : samples) {
        const double lat = kAnchorLat + s.north_m / kMetersPerDegLat;
        const double lon = kAnchorLon + s.east_m / lon_scale;
        // map_version 用 base 的版本（v1 写入时为 1）
        if (std::fprintf(f,
                         R"({"schema":"semantictile.overlay.v1","map_version":1,)"
                         R"("lat":%.9f,"lon":%.9f,"category":"%s","confidence":%d,)"
                         R"("source":"qwen_vl","explanation":"%s"})"
                         "\n",
                         lat, lon, s.category, s.confidence, s.note) > 0) {
            ++n;
        }
    }
    std::fclose(f);
    return n;
}

}  // namespace

int main(int argc, char** argv) {
    const std::string out_dir = (argc > 1) ? argv[1] : "/tmp";

    std::cout << "== 语义地图消费 demo ==\n";
    std::cout << "锚点 wgs84(" << kAnchorLat << ", " << kAnchorLon
              << ") zoom=" << kZoom
              << "（约 " << static_cast<int>(kRoomM) << "m/瓦片，房间尺度）\n\n";

    build_floor_plan();
    std::cout << "① 楼层布局登记：" << g_tile_count << " 个语义瓦片\n";

    const std::string base_path = out_dir + "/semantic_floor_base.txt";
    const std::int64_t n_base =
        save_semantic_tiles(g_store, kZoom, base_path, 1);
    if (n_base < 0) {
        std::cerr << "写 base 失败: " << base_path << "\n";
        return 1;
    }
    std::cout << "   已保存 base: " << base_path
              << " (" << n_base << " 个瓦片, version=1)\n\n";

    // ② 老师回写
    const std::string overlay_path = out_dir + "/semantic_floor_labels.jsonl";
    const std::int64_t n_labels = simulate_vlm_writeback(overlay_path);
    if (n_labels < 0) {
        std::cerr << "写 overlay 失败: " << overlay_path << "\n";
        return 1;
    }
    std::cout << "② 模拟 VLM 回写：" << n_labels << " 条标注 → " << overlay_path << "\n";

    // 直接在本进程内应用（等价于 semantic_label_merge 的 C++ 路径）
    std::FILE* f = std::fopen(overlay_path.c_str(), "rb");
    if (f == nullptr) {
        std::cerr << "读 overlay 失败\n";
        return 1;
    }
    OverlayStats stats;
    char line[1024];
    while (std::fgets(line, sizeof(line), f) != nullptr) {
        if (line[0] == '\n') continue;
        (void)apply_overlay_entry(line, g_store, 1, kZoom, stats);
    }
    std::fclose(f);

    std::cout << "   应用 " << stats.applied << " 条"
              << "（版本不符 " << stats.skipped_version
              << "，非法 " << stats.skipped_bad
              << "，容量满 " << stats.skipped_capacity << "）\n";

    // 版本 +1：内容变了，版本必须递增，否则 trace 样本会错误配对
    const std::string merged_path = out_dir + "/semantic_floor_v2.txt";
    const std::int64_t n_merged =
        save_semantic_tiles(g_store, kZoom, merged_path, 2);
    if (n_merged < 0) {
        std::cerr << "写合并结果失败: " << merged_path << "\n";
        return 1;
    }
    std::cout << "   已保存合并结果: " << merged_path
              << " (" << n_merged << " 个瓦片, version=2)\n\n";

    // ③ 统计与可视化入口
    std::size_t tagged = 0;
    g_store.for_each([&](const SemanticTile& t) {
        if (t.tag_count > 0) ++tagged;
    });
    std::cout << "③ 结果：共 " << g_store.size() << " 个瓦片，"
              << tagged << " 个带语义标签\n";

    std::cout << "\n下一步（生成可视化）：\n"
              << "  python3 tools/visualize_semantic_map.py "
              << merged_path << " " << out_dir << "/semantic_floor_map.png\n"
              << "  # 或加 --no-basemap 离线渲染\n";
    return 0;
}
