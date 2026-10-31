// 语义瓦片测试：标签合并、类别解析、序列化回读。
#include <cassert>
#include <iostream>
#include <string>

#include "map/tile_math.h"
#include "map/semantic_tile.h"
#include "map/semantic_persistence.h"

namespace {

void test_tag_merge_and_lookup() {
    using namespace spatial_map;
    SemanticTile t;
    t.key = TileKey{18, 1, 2};
    assert(t.add_tag(SemanticCategory::Kitchen, 90));
    assert(t.add_tag(SemanticCategory::Kitchen, 95));   // 同类别取高置信
    assert(t.confidence_of(SemanticCategory::Kitchen) == 95);
    assert(t.add_tag(SemanticCategory::Door, 80));
    assert(t.tag_count == 2);
    assert(t.confidence_of(SemanticCategory::Corridor) == 0);  // 未找到
    std::cout << "  [ok] tag merge and lookup\n";
}

void test_semantic_name_roundtrip() {
    using namespace spatial_map;
    assert(semantic_from_name("kitchen") == SemanticCategory::Kitchen);
    assert(semantic_from_name("living_room") == SemanticCategory::LivingRoom);
    assert(semantic_from_name("not_a_thing") == SemanticCategory::Unknown);
    std::cout << "  [ok] semantic name roundtrip\n";
}

void test_semantic_store_idempotent_merge() {
    using namespace spatial_map;
    SemanticTileStore<64> s;
    SemanticTile a;
    a.key = TileKey{18, 10, 20};
    a.add_tag(SemanticCategory::Kitchen, 90);
    assert(s.insert(a));
    SemanticTile b;
    b.key = TileKey{18, 10, 20};
    b.add_tag(SemanticCategory::Door, 85);   // 同瓦片，合并
    assert(s.insert(b));
    assert(s.size() == 1);
    const SemanticTile* t = s.find(TileKey{18, 10, 20});
    assert(t != nullptr);
    assert(t->confidence_of(SemanticCategory::Kitchen) == 90);
    assert(t->confidence_of(SemanticCategory::Door) == 85);
    std::cout << "  [ok] store idempotent merge\n";
}

void test_semantic_persistence() {
    using namespace spatial_map;
    SemanticTileStore<64> a;
    SemanticTile t;
    t.key = TileKey{18, 3, 4};
    t.add_tag(SemanticCategory::Kitchen, 90);
    t.add_tag(SemanticCategory::GoalObject, 80);
    a.insert(t);

    const std::string path = "/tmp/test_semantic.persist";
    const std::int64_t saved = save_semantic_tiles(a, 18, path, 42);
    assert(saved == 1);

    SemanticTileStore<64> b;
    std::uint32_t zoom = 0;
    std::uint32_t mv = 0;
    const std::int64_t loaded = load_semantic_tiles(b, zoom, path, &mv);
    assert(loaded == 1);
    assert(zoom == 18);
    assert(mv == 42);            // v2 带 map_version
    const SemanticTile* rt = b.find(TileKey{18, 3, 4});
    assert(rt != nullptr);
    assert(rt->confidence_of(SemanticCategory::Kitchen) == 90);
    assert(rt->confidence_of(SemanticCategory::GoalObject) == 80);
    std::cout << "  [ok] semantic persistence (v2 with map_version)\n";
}

// v1 旧文件必须仍可读，map_version 读为 0。
void test_semantic_persistence_v1_compat() {
    using namespace spatial_map;
    const std::string path = "/tmp/test_semantic_v1.persist";
    std::FILE* f = std::fopen(path.c_str(), "wb");
    assert(f != nullptr);
    std::fputs("semantic-tile 1 18\ntile 5 6 kitchen 88\n", f);
    std::fclose(f);

    SemanticTileStore<64> s;
    std::uint32_t zoom = 0;
    std::uint32_t mv = 99;
    const std::int64_t loaded = load_semantic_tiles(s, zoom, path, &mv);
    assert(loaded == 1);
    assert(zoom == 18);
    assert(mv == 0);             // v1 无语义版本
    const SemanticTile* t = s.find(TileKey{18, 5, 6});
    assert(t != nullptr);
    assert(t->confidence_of(SemanticCategory::Kitchen) == 88);
    std::cout << "  [ok] semantic persistence v1 backward compatible\n";
}

}  // namespace

int main() {
    std::cout << "== semantic tile tests ==\n";
    test_tag_merge_and_lookup();
    test_semantic_name_roundtrip();
    test_semantic_store_idempotent_merge();
    test_semantic_persistence();
    test_semantic_persistence_v1_compat();
    std::cout << "== all passed ==\n";
    return 0;
}
