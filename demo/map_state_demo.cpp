// 组合演示：Super-LIO 增量位姿 → 地理锚定 → 哈希瓦片 → 空间状态事务 → 快照。
//
// 链路：
//   1. 模拟 LIO 前端（如 Super-LIO）输出的局部 ENU 位姿（x/y/z + 四元数）；
//   2. 经地图锚定（原点经纬度 + 航向）换算为 WGS84 经纬度；
//   3. WGS84 → GCJ-02 → 高德/XYZ 瓦片 z/x/y，哈希编码；
//   4. HashTileStore 幂等登记（重复拒绝）；
//   5. 每个新瓦片作为 SubmapPose 状态提交进 SpatialRuntime（原子事务）；
//   6. optimize() 返回带版本快照，验证查询链路。
#include <cmath>
#include <cstdint>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "map/tile_math.h"
#include "map/hash_tile_store.h"
#include "map/amap_tile.h"
#include "map/geo_anchor.h"
#include "runtime/spatial_runtime.h"

namespace {

class DemoBackend final : public fork_spatial::runtime::IOptimizerBackend {
public:
    bool apply(const fork_spatial::runtime::GraphTransaction& t, std::string&) override {
        operation_count_ += t.operations.size();
        return true;
    }
    fork_spatial::runtime::EstimateSnapshot optimize(std::uint64_t v) override {
        return {v, 0, {}};
    }
    bool marginalize(std::span<const fork_spatial::runtime::StateKey>,
                     fork_spatial::runtime::FactorProposal& prior,
                     std::string&) override {
        prior = {};
        return true;
    }
    std::size_t operation_count() const { return operation_count_; }

private:
    std::size_t operation_count_{};
};

// Super-LIO 风格的局部位姿：ENU 米制位置 + 单位四元数。
struct LioPose {
    double x, y, z;
    double qx, qy, qz, qw;  // 未在本 demo 使用，仅保留输入形状
    std::int64_t t_ns;
};

}  // namespace

int main() {
    using fork_spatial::runtime::FactorBatch;
    using fork_spatial::runtime::FixedLagWindowPolicy;
    using fork_spatial::runtime::SpatialRuntime;
    using fork_spatial::runtime::StateKey;
    using fork_spatial::runtime::StateLifecycle;
    using fork_spatial::runtime::StateType;
    using fork_spatial::runtime::SubmitResult;

    constexpr std::uint32_t kZoom = 18;

    // 地图锚定：杭州某处作为局部 map 原点，局部 x 轴与正北对齐。
    const spatial_map::GeoAnchor anchor{30.2740, 120.1550, 0.0};

    // 1. 模拟 Super-LIO 增量位姿（ENU 米），跨 3 个瓦片 + 1 个重复。
    //    z18 瓦片约 130m，位姿间距 400~800m 以确保跨瓦片。
    const std::vector<LioPose> poses{
        {0.0, 0.0, 0.0, 0, 0, 0, 1, 100'000'000},        // 原点
        {0.0, 500.0, 0.0, 0, 0, 0, 1, 200'000'000},      // 向北 500m
        {700.0, 0.0, 0.0, 0, 0, 0, 1, 300'000'000},      // 向东 700m
        {0.0, 0.0, 0.0, 0, 0, 0, 1, 400'000'000},        // 重复原点 → 幂等拒绝
    };

    spatial_map::HashTileStore<64> tiles;

    auto backend = std::make_unique<DemoBackend>();
    DemoBackend* backend_view = backend.get();
    SpatialRuntime runtime(std::move(backend),
                           std::make_unique<FixedLagWindowPolicy>(10'000'000'000));

    std::cout << "== Super-LIO 增量位姿 → 哈希瓦片 → 空间状态事务 ==\n";
    std::uint32_t registered = 0;
    std::uint32_t duplicates = 0;

    for (const LioPose& p : poses) {
        // 2. 局部 ENU → WGS84 → GCJ-02 → 高德瓦片
        const spatial_map::LatLon wgs = spatial_map::local_to_wgs84(anchor, p.x, p.y);
        const spatial_map::TileKey key =
            spatial_map::wgs84_to_amap_tile(wgs.lat, wgs.lon, kZoom);
        const std::uint64_t h = spatial_map::encode_tile_key(key);

        // 3. 瓦片幂等登记
        if (!tiles.insert(key)) {
            std::cout << "  [dup] 局部(" << p.x << "," << p.y << ") → z" << key.z
                      << " x" << key.x << " y" << key.y << " 已存在，拒绝重复\n";
            ++duplicates;
            continue;
        }

        // 4. 新瓦片 → SubmapPose 状态
        double nlat = 0.0, wlon = 0.0;
        spatial_map::tile_to_latlon(key.z, key.x, key.y, nlat, wlon);

        StateKey skey{1, 1, StateType::SubmapPose, p.t_ns,
                      static_cast<std::uint32_t>(h & 0xFFFFFFFFU)};
        FactorBatch batch;
        batch.state_requests.push_back(
            {skey, {{nlat, wlon, static_cast<double>(key.z),
                     static_cast<double>(key.x), static_cast<double>(key.y)}},
             StateLifecycle::Active});

        const SubmitResult r = runtime.submit(batch);
        if (!r) {
            std::cerr << "  提交失败: " << r.message << "\n";
            return 1;
        }
        std::cout << "  [+ok] 局部(" << p.x << "," << p.y << ") → wgs(" << wgs.lat
                  << "," << wgs.lon << ") → z" << key.z << " x" << key.x << " y"
                  << key.y << "  graph_version=" << r.graph_version << "\n";
        ++registered;
    }

    // 5. 版本化快照
    const auto snap = runtime.optimize();
    std::cout << "\n== 状态快照 ==\n"
              << "  graph_version=" << snap.graph_version
              << "  active_states=" << runtime.states().size()
              << "  backend_operations=" << backend_view->operation_count()
              << "  tiles_registered=" << registered
              << "  duplicates_rejected=" << duplicates << "\n";

    // 6. 查询验证
    const spatial_map::LatLon q_wgs = spatial_map::local_to_wgs84(anchor, 200.0, 300.0);
    const spatial_map::TileKey q = spatial_map::wgs84_to_amap_tile(q_wgs.lat, q_wgs.lon, kZoom);
    std::cout << "\n== 查询验证 ==\n"
              << "  局部(200,300) → wgs(" << q_wgs.lat << "," << q_wgs.lon << ") → z"
              << q.z << " x" << q.x << " y" << q.y
              << (tiles.contains(q) ? " → 命中已登记瓦片" : " → 未登记") << "\n";

    return 0;
}
