// 无框架单元测试：哈希瓦片编码往返 + 幂等登记 + 状态事务 + 地理锚定。
#include <cassert>
#include <cmath>
#include <iostream>
#include <memory>
#include <span>
#include <string>

#include "map/tile_math.h"
#include "map/hash_tile_store.h"
#include "map/amap_tile.h"
#include "map/geo_anchor.h"
#include "map/tile_persistence.h"
#include "runtime/spatial_runtime.h"

namespace {

void test_tile_roundtrip() {
    using spatial_map::latlon_to_tile;
    using spatial_map::tile_to_latlon;

    // zoom 0 时全世界是一个瓦片 (0,0,0)
    auto k = latlon_to_tile(0.0, 0.0, 0);
    assert(k.z == 0 && k.x == 0 && k.y == 0);

    // 往返：用瓦片中心点（远离边界）投影，应回到同一瓦片
    const auto a = latlon_to_tile(30.2740, 120.1550, 18);
    double nlat = 0.0, wlon = 0.0;
    tile_to_latlon(a.z, a.x, a.y, nlat, wlon);
    double slat = 0.0, elon = 0.0;
    tile_to_latlon(a.z, a.x + 1, a.y + 1, slat, elon);
    const double clat = (nlat + slat) / 2.0;
    const double clon = (wlon + elon) / 2.0;
    const auto b = latlon_to_tile(clat, clon, 18);
    assert(a == b);

    std::cout << "  [ok] tile roundtrip\n";
}

void test_hash_store_idempotent() {
    using spatial_map::HashTileStore;
    using spatial_map::latlon_to_tile;

    HashTileStore<64> store;
    const auto k = latlon_to_tile(30.2740, 120.1550, 18);
    assert(store.insert(k));
    assert(!store.insert(k));  // 幂等拒绝
    assert(store.contains(k));
    assert(store.size() == 1);
    assert(store.erase(k));
    assert(!store.contains(k));
    assert(store.size() == 0);

    std::cout << "  [ok] hash store idempotent\n";
}

void test_runtime_transaction() {
    using namespace fork_spatial::runtime;

    struct Backend final : IOptimizerBackend {
        bool apply(const GraphTransaction&, std::string&) override { return true; }
        EstimateSnapshot optimize(std::uint64_t v) override { return {v, 0, {}}; }
        bool marginalize(std::span<const StateKey>, FactorProposal& p,
                         std::string&) override {
            p = {};
            return true;
        }
    };

    SpatialRuntime runtime(std::make_unique<Backend>(),
                           std::make_unique<FixedLagWindowPolicy>(1'000'000'000));
    const StateKey key{1, 1, StateType::SubmapPose, 100, 7};
    FactorBatch batch;
    batch.state_requests.push_back({key, {{30.0, 120.0, 18, 1, 1}}});
    const SubmitResult r = runtime.submit(batch);
    assert(r);
    assert(r.graph_version == 1);
    assert(runtime.states().size() == 1);

    std::cout << "  [ok] runtime transaction\n";
}

void test_geo_anchor() {
    using spatial_map::GeoAnchor;
    using spatial_map::LatLon;
    using spatial_map::local_to_wgs84;

    const GeoAnchor anchor{30.0, 120.0, 0.0};

    // 原点 → 原点
    const LatLon o = local_to_wgs84(anchor, 0.0, 0.0);
    assert(std::abs(o.lat - 30.0) < 1e-9);
    assert(std::abs(o.lon - 120.0) < 1e-9);

    // 北向 500m → 纬度约 +500/111320 度
    const LatLon n = local_to_wgs84(anchor, 0.0, 500.0);
    assert(std::abs(n.lat - (30.0 + 500.0 / 111320.0)) < 1e-6);
    assert(std::abs(n.lon - 120.0) < 1e-9);

    // 东向 500m → 经度约 +500/(111320*cos30) 度
    const LatLon e = local_to_wgs84(anchor, 500.0, 0.0);
    const double meters_per_deg_lon = 111320.0 * std::cos(30.0 * M_PI / 180.0);
    assert(std::abs(e.lon - (120.0 + 500.0 / meters_per_deg_lon)) < 1e-6);
    assert(std::abs(e.lat - 30.0) < 1e-9);

    std::cout << "  [ok] geo anchor\n";
}

// 追加：瓦片持久化测试
void test_tile_persistence() {
    using spatial_map::HashTileStore;
    using spatial_map::TileKey;
    using spatial_map::latlon_to_tile;
    using spatial_map::save_tiles;
    using spatial_map::load_tiles;

    HashTileStore<64> a;
    a.insert(latlon_to_tile(30.2740, 120.1550, 18));
    a.insert(latlon_to_tile(30.2780, 120.1550, 18));

    const std::string path = "/tmp/test_tiles.persist";
    const std::int64_t saved = save_tiles(a, 42, path);
    assert(saved == 2);

    HashTileStore<64> b;
    std::uint64_t version = 0;
    const std::int64_t loaded = load_tiles(b, version, path);
    assert(loaded == 2);
    assert(version == 42);
    assert(b.contains(latlon_to_tile(30.2740, 120.1550, 18)));
    assert(b.contains(latlon_to_tile(30.2780, 120.1550, 18)));
    assert(b.size() == 2);

    std::cout << "  [ok] tile persistence\n";
}

}  // namespace

int main() {
    std::cout << "== map-state tests ==\n";
    test_tile_roundtrip();
    test_hash_store_idempotent();
    test_runtime_transaction();
    test_geo_anchor();
    test_tile_persistence();
    std::cout << "== all passed ==\n";
    return 0;
}

