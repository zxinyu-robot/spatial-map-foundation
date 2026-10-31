// 语义地图演示：模拟语义 SLAM 输出 → 地理锚定 → 语义瓦片 → 查询 + 序列化。
//
// 对接真机：语义 SLAM 每个检测对象输出「局部位姿 + 类别 + 置信度」，
// 本链路把它投影到瓦片并挂语义标签，供上游按瓦片消费。
#include <cstdint>
#include <iostream>
#include <string>

#include "map/tile_math.h"
#include "map/amap_tile.h"
#include "map/geo_anchor.h"
#include "map/semantic_tile.h"
#include "map/semantic_persistence.h"

int main() {
    using namespace spatial_map;

    constexpr std::uint32_t kZoom = 18;
    const GeoAnchor anchor{30.2740, 120.1550, 0.0};

    // 模拟语义 SLAM 输出：局部位姿 + 语义 + 置信度
    struct Det {
        double x, y;
        SemanticCategory cat;
        std::uint8_t conf;
    };
    const Det dets[]{
        {0.0, 0.0, SemanticCategory::Kitchen, 90},
        {0.0, 500.0, SemanticCategory::Corridor, 85},
        {700.0, 0.0, SemanticCategory::Door, 95},
        {0.0, 0.0, SemanticCategory::GoalObject, 80},  // 同瓦片，合并
    };

    SemanticTileStore<64> store;
    std::cout << "== 语义 SLAM 输出 → 语义瓦片 ==\n";
    for (const Det& d : dets) {
        const LatLon wgs = local_to_wgs84(anchor, d.x, d.y);
        const TileKey key = wgs84_to_amap_tile(wgs.lat, wgs.lon, kZoom);
        SemanticTile t;
        t.key = key;
        t.add_tag(d.cat, d.conf);
        const bool inserted = store.insert(t);
        std::cout << "  [" << (inserted ? "+ok" : "merge") << "] 局部("
                  << d.x << "," << d.y << ") → z" << key.z << " x" << key.x
                  << " y" << key.y << " " << semantic_name(d.cat)
                  << " conf=" << static_cast<int>(d.conf) << "\n";
    }

    std::cout << "\n== 按瓦片查询语义 ==\n";
    store.for_each([](const SemanticTile& t) {
        std::cout << "  z" << t.key.z << " x" << t.key.x << " y" << t.key.y << ":";
        for (std::uint8_t i = 0; i < t.tag_count; ++i) {
            std::cout << " " << semantic_name(t.tags[i].category)
                      << "=" << static_cast<int>(t.tags[i].confidence);
        }
        std::cout << "\n";
    });

    std::cout << "\n== 语义瓦片序列化（对接真机 / 供上游消费）==\n";
    const std::string path = "/tmp/semantic_tiles.txt";
    const std::int64_t saved = save_semantic_tiles(store, kZoom, path);
    std::cout << "  保存 " << saved << " 个语义瓦片 → " << path << "\n";

    // 回读验证
    SemanticTileStore<64> loaded;
    std::uint32_t z = 0;
    const std::int64_t n = load_semantic_tiles(loaded, z, path);
    std::cout << "  回读 " << n << " 个语义瓦片 (zoom=" << z << ")\n";
    const TileKey q = wgs84_to_amap_tile(30.2740, 120.1550, kZoom);
    const SemanticTile* qt = loaded.find(q);
    if (qt != nullptr) {
        std::cout << "  查询瓦片 x" << q.x << " y" << q.y << " 语义:";
        for (std::uint8_t i = 0; i < qt->tag_count; ++i) {
            std::cout << " " << semantic_name(qt->tags[i].category)
                      << "=" << static_cast<int>(qt->tags[i].confidence);
        }
        std::cout << "\n";
    }

    return 0;
}
