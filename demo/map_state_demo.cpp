// 组合演示：局部 SLAM 增量状态 → 哈希瓦片嵌入 → 空间状态事务 → 版本化快照。
//
// 链路：
//   1. 模拟局部 SLAM 输出的一组位姿（经纬度）；
//   2. 每个位姿经 Web Mercator 投影编码为高德/XYZ 瓦片 z/x/y；
//   3. HashTileStore 幂等登记瓦片（重复拒绝）；
//   4. 每个新瓦片作为 SubmapPose 状态提交进 SpatialRuntime（原子事务）；
//   5. optimize() 返回带版本快照，验证查询链路。
#include <cmath>
#include <cstdint>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "map/tile_math.h"
#include "map/hash_tile_store.h"
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

struct SlamPose {
    double lat;
    double lon;
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

    constexpr std::uint32_t kZoom = 18;  // 城市街区级瓦片

    // 1. 模拟局部 SLAM 增量位姿（杭州某街区附近，跨 3 个瓦片 + 1 个重复）
    const std::vector<SlamPose> poses{
        {30.2740, 120.1550, 100'000'000},  // 瓦片 A
        {30.2780, 120.1550, 200'000'000},  // 瓦片 B（纬度 +0.004）
        {30.2740, 120.1590, 300'000'000},  // 瓦片 C（经度 +0.004）
        {30.2740, 120.1550, 400'000'000},  // 重复瓦片 A → 幂等拒绝
    };

    spatial_map::HashTileStore<64> tiles;

    auto backend = std::make_unique<DemoBackend>();
    DemoBackend* backend_view = backend.get();
    SpatialRuntime runtime(std::move(backend),
                           std::make_unique<FixedLagWindowPolicy>(10'000'000'000));

    std::cout << "== 哈希瓦片嵌入 + 空间状态事务 ==\n";
    std::uint32_t registered = 0;
    std::uint32_t duplicates = 0;

    for (const SlamPose& p : poses) {
        const spatial_map::TileKey key = spatial_map::latlon_to_tile(p.lat, p.lon, kZoom);
        const std::uint64_t h = spatial_map::encode_tile_key(key);

        // 2. 瓦片幂等登记
        if (!tiles.insert(key)) {
            std::cout << "  [dup] z" << key.z << " x" << key.x << " y" << key.y
                      << " 已存在，拒绝重复登记\n";
            ++duplicates;
            continue;
        }

        // 3. 新瓦片 → SubmapPose 状态（value 存瓦片西北角经纬度 + z/x/y）
        double nlat = 0.0, wlon = 0.0;
        spatial_map::tile_to_latlon(key.z, key.x, key.y, nlat, wlon);

        StateKey skey{/*robot_id=*/1, /*session_id=*/1, StateType::SubmapPose,
                      p.t_ns, static_cast<std::uint32_t>(h & 0xFFFFFFFFU)};
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
        std::cout << "  [+ok] z" << key.z << " x" << key.x << " y" << key.y
                  << "  瓦片NW=(" << nlat << "," << wlon << ")  graph_version="
                  << r.graph_version << "\n";
        ++registered;
    }

    // 4. 版本化快照
    const auto snap = runtime.optimize();
    std::cout << "\n== 状态快照 ==\n"
              << "  graph_version=" << snap.graph_version
              << "  active_states=" << runtime.states().size()
              << "  backend_operations=" << backend_view->operation_count()
              << "  tiles_registered=" << registered
              << "  duplicates_rejected=" << duplicates << "\n";

    // 5. 查询验证：新坐标 → 是否命中已登记瓦片
    const spatial_map::TileKey q =
        spatial_map::latlon_to_tile(30.27405, 120.15505, kZoom);
    std::cout << "\n== 查询验证 ==\n"
              << "  查询坐标(30.27405,120.15505) → z" << q.z << " x" << q.x
              << " y" << q.y << (tiles.contains(q) ? " → 命中已登记瓦片" : " → 未登记")
              << "\n";

    return 0;
}
