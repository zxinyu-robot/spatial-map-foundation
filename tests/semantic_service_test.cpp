// 语义瓦片查询服务测试：坐标系、过滤、排序、JSON 构造.
//
// 重点锁住 GCJ-02 / WGS84 混用 bug：瓦片索引空间是 GCJ-02，
// 机器人位姿是 WGS84；若查询时直接拿 WGS84 点比 GCJ-02 瓦片中心，
// 会产生约 500m 的系统性偏移，表现为「目标莫名其妙远了 500 米」。
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <iostream>

#include "map/amap_tile.h"
#include "map/semantic_tile.h"
#include "map/tile_math.h"
#include "runtime/semantic_service.h"

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

#define CHECK_NEAR(a, b, tol)                                                 \
    do {                                                                      \
        const double va = (a);                                                \
        const double vb = (b);                                                \
        if (std::fabs(va - vb) > (tol)) {                                      \
            std::cerr << "  [FAIL] " << #a << "=" << va << " vs " << #b "="   \
                      << vb << " (tol " << (tol) << ") @ line " << __LINE__   \
                      << "\n";                                                \
            ++g_failures;                                                     \
        }                                                                     \
    } while (0)

using namespace spatial_map;

// 锚点：杭州武林广场附近
constexpr double kAnchorLat = 30.2741;
constexpr double kAnchorLon = 120.1551;

void add_tile(SemanticTileStore<64>& s, std::uint32_t z, std::uint32_t x,
              std::uint32_t y, SemanticCategory cat, std::uint8_t conf) {
    SemanticTile t;
    t.key = TileKey{z, x, y};
    t.add_tag(cat, conf);
    s.insert(t);
}

void test_haversine() {
    // 赤道上 1 度经度 ≈ 111.19 km
    CHECK_NEAR(haversine_m(0.0, 0.0, 0.0, 1.0), 111195.0, 100.0);
    // 同一点距离为 0
    CHECK_NEAR(haversine_m(kAnchorLat, kAnchorLon, kAnchorLat, kAnchorLon), 0.0, 1e-6);
    // 北纬 30.27 处 1 度经度 ≈ 111.32km * cos(30.27°)
    const double d = haversine_m(30.0, 120.0, 30.0, 121.0);
    CHECK(d > 90000.0 && d < 100000.0);   // 实际约 96 km
    std::cout << "  [ok] haversine\n";
}

void test_gcj02_vs_wgs84_offset() {
    // 瓦片由 wgs84_to_amap_tile 生成 → 索引空间是 GCJ-02
    const TileKey k = wgs84_to_amap_tile(kAnchorLat, kAnchorLon, 18);

    double g_lat = 0.0, g_lon = 0.0;
    tile_center_gcj02(k, g_lat, g_lon);
    double w_lat = 0.0, w_lon = 0.0;
    tile_center_wgs84(k, w_lat, w_lon);

    // 两者必须不同，且偏移量在百米量级（GCJ-02 特征）
    const double offset = haversine_m(g_lat, g_lon, w_lat, w_lon);
    CHECK(offset > 100.0 && offset < 800.0);

    // 关键：用 WGS84 锚点查该瓦片，距离应远小于用 GCJ-02 点查
    SemanticTileStore<64> s;
    add_tile(s, 18, k.x, k.y, SemanticCategory::Kitchen, 90);

    SemanticHit hits[8];
    bool trunc = false;
    const std::size_t n_w = query_radius(s, kAnchorLat, kAnchorLon, 300.0,
                                         CategoryFilter{}, 0, hits, trunc);
    CHECK(n_w == 1);
    // 瓦片中心到锚点的真实距离：< 一个瓦片对角线(~200m)
    if (n_w == 1) CHECK(hits[0].dist_m < 200.0);

    // 若误把 GCJ-02 点当作 WGS84 查询点（错误用法），偏移量 ~500m 会把
    // 正确瓦片推出查询半径 —— 用户可见症状是「语义瓦片莫名查不到」，
    // 而不是报错。这是本 bug 最隐蔽之处，故显式断言。
    const std::size_t n_g = query_radius(s, g_lat, g_lon, 300.0,
                                         CategoryFilter{}, 0, hits, trunc);
    CHECK(n_g == 0);   // 300m 半径下被 519m 偏移排除

    // 放宽半径则能查到，但距离被虚增约 500m
    const std::size_t n_g_wide = query_radius(s, g_lat, g_lon, 5000.0,
                                              CategoryFilter{}, 0, hits, trunc);
    CHECK(n_g_wide == 1);
    if (n_g_wide == 1) CHECK(hits[0].dist_m > 100.0);

    std::cout << "  [ok] GCJ-02 vs WGS84 offset guarded (offset="
              << static_cast<int>(offset) << "m)\n";
}

void test_category_filter() {
    const TileKey a = wgs84_to_amap_tile(kAnchorLat, kAnchorLon, 18);
    SemanticTileStore<64> s;
    add_tile(s, 18, a.x, a.y, SemanticCategory::Kitchen, 90);
    add_tile(s, 18, a.x + 1, a.y, SemanticCategory::Door, 90);
    add_tile(s, 18, a.x, a.y + 1, SemanticCategory::Corridor, 90);

    SemanticHit hits[8];
    bool trunc = false;

    // 不过滤
    CHECK(query_radius(s, kAnchorLat, kAnchorLon, 5000.0, CategoryFilter{}, 0,
                       hits, trunc) == 3);

    // 单类别
    const char* k1[] = {"kitchen"};
    CHECK(query_radius(s, kAnchorLat, kAnchorLon, 5000.0,
                       CategoryFilter{k1, 1}, 0, hits, trunc) == 1);
    CHECK(std::strcmp(hits[0].tile->tags[0].category == SemanticCategory::Kitchen
                          ? "kitchen" : "?", "kitchen") == 0);

    // 多类别
    const char* k2[] = {"kitchen", "door"};
    CHECK(query_radius(s, kAnchorLat, kAnchorLon, 5000.0,
                       CategoryFilter{k2, 2}, 0, hits, trunc) == 2);

    // 不存在的类别
    const char* k3[] = {"elevator"};
    CHECK(query_radius(s, kAnchorLat, kAnchorLon, 5000.0,
                       CategoryFilter{k3, 1}, 0, hits, trunc) == 0);

    std::cout << "  [ok] category filter\n";
}

void test_confidence_filter() {
    const TileKey a = wgs84_to_amap_tile(kAnchorLat, kAnchorLon, 18);
    SemanticTileStore<64> s;
    add_tile(s, 18, a.x, a.y, SemanticCategory::Kitchen, 90);
    add_tile(s, 18, a.x + 1, a.y, SemanticCategory::Door, 60);

    SemanticHit hits[8];
    bool trunc = false;
    CHECK(query_radius(s, kAnchorLat, kAnchorLon, 5000.0, CategoryFilter{}, 0,
                       hits, trunc) == 2);
    CHECK(query_radius(s, kAnchorLat, kAnchorLon, 5000.0, CategoryFilter{}, 80,
                       hits, trunc) == 1);
    CHECK(query_radius(s, kAnchorLat, kAnchorLon, 5000.0, CategoryFilter{}, 95,
                       hits, trunc) == 0);

    // 瓦片有多个标签时，任一标签满足即可命中
    SemanticTileStore<64> m;
    SemanticTile t;
    t.key = TileKey{18, a.x, a.y};
    t.add_tag(SemanticCategory::Kitchen, 95);
    t.add_tag(SemanticCategory::Door, 20);
    m.insert(t);
    CHECK(query_radius(m, kAnchorLat, kAnchorLon, 5000.0, CategoryFilter{}, 80,
                       hits, trunc) == 1);
    std::cout << "  [ok] confidence filter\n";
}

void test_sorted_ascending() {
    const TileKey a = wgs84_to_amap_tile(kAnchorLat, kAnchorLon, 18);
    SemanticTileStore<64> s;
    // 故意逆序插入：远的先插
    add_tile(s, 18, a.x + 3, a.y + 3, SemanticCategory::Bedroom, 90);
    add_tile(s, 18, a.x + 1, a.y + 1, SemanticCategory::Door, 90);
    add_tile(s, 18, a.x, a.y, SemanticCategory::Kitchen, 90);

    SemanticHit hits[8];
    bool trunc = false;
    const std::size_t n = query_radius(s, kAnchorLat, kAnchorLon, 20000.0,
                                       CategoryFilter{}, 0, hits, trunc);
    CHECK(n == 3);
    for (std::size_t i = 1; i < n; ++i) {
        CHECK(hits[i - 1].dist_m <= hits[i].dist_m);
    }
    std::cout << "  [ok] sorted ascending\n";
}

void test_radius_excludes() {
    const TileKey a = wgs84_to_amap_tile(kAnchorLat, kAnchorLon, 18);
    SemanticTileStore<64> s;
    add_tile(s, 18, a.x, a.y, SemanticCategory::Kitchen, 90);
    add_tile(s, 18, a.x + 20, a.y + 20, SemanticCategory::Door, 90);  // 很远

    SemanticHit hits[8];
    bool trunc = false;
    CHECK(query_radius(s, kAnchorLat, kAnchorLon, 300.0, CategoryFilter{}, 0,
                       hits, trunc) == 1);
    CHECK(query_radius(s, kAnchorLat, kAnchorLon, 100000.0, CategoryFilter{}, 0,
                       hits, trunc) == 2);
    std::cout << "  [ok] radius excludes\n";
}

void test_truncation() {
    const TileKey a = wgs84_to_amap_tile(kAnchorLat, kAnchorLon, 18);
    SemanticTileStore<512> s;
    for (std::uint32_t i = 0; i < kMaxQueryResults + 10; ++i) {
        SemanticTile t;
        t.key = TileKey{18, a.x + (i % 20), a.y + (i / 20)};
        t.add_tag(SemanticCategory::FreeSpace, 90);
        s.insert(t);
    }
    SemanticHit hits[kMaxQueryResults];
    bool trunc = false;
    const std::size_t n = query_radius(s, kAnchorLat, kAnchorLon, 1e9,
                                       CategoryFilter{}, 0, hits, trunc);
    CHECK(n == kMaxQueryResults);
    CHECK(trunc == true);
    std::cout << "  [ok] bounded result / truncated flag\n";
}

void test_json_info() {
    char buf[kServiceReplyMax];
    CHECK(build_info_json(buf, 7, 18, 3, kAnchorLat, kAnchorLon));
    CHECK(std::strstr(buf, "\"map_version\":7") != nullptr);
    CHECK(std::strstr(buf, "\"zoom\":18") != nullptr);
    CHECK(std::strstr(buf, "\"tile_count\":3") != nullptr);
    CHECK(std::strstr(buf, "\"lat\":30.27") != nullptr);
    CHECK(std::strstr(buf, "\"ok\":true") != nullptr);
    std::cout << "  [ok] json info\n";
}

void test_json_hits() {
    const TileKey a = wgs84_to_amap_tile(kAnchorLat, kAnchorLon, 18);
    SemanticTileStore<64> s;
    add_tile(s, 18, a.x, a.y, SemanticCategory::Kitchen, 90);

    SemanticHit hits[8];
    bool trunc = false;
    const std::size_t n = query_radius(s, kAnchorLat, kAnchorLon, 5000.0,
                                       CategoryFilter{}, 0, hits, trunc);
    char buf[kServiceReplyMax];
    CHECK(build_hits_json(buf, 7, hits, n, trunc, true));
    CHECK(std::strstr(buf, "\"map_version\":7") != nullptr);
    CHECK(std::strstr(buf, "\"count\":1") != nullptr);
    CHECK(std::strstr(buf, "\"truncated\":false") != nullptr);
    CHECK(std::strstr(buf, "\"dist_m\"") != nullptr);
    CHECK(std::strstr(buf, "\"cat\":\"kitchen\"") != nullptr);
    CHECK(std::strstr(buf, "\"conf\":90") != nullptr);
    std::cout << "  [ok] json hits\n";
}

void test_json_error() {
    char buf[kServiceReplyMax];
    CHECK(build_error_json(buf, "unknown cmd"));
    CHECK(std::strstr(buf, "\"ok\":false") != nullptr);
    CHECK(std::strstr(buf, "unknown cmd") != nullptr);
    std::cout << "  [ok] json error\n";
}

void test_json_parsing() {
    char s[32] = {};
    CHECK(detail::json_get_string("{\"cmd\":\"info\"}", "cmd", s, sizeof(s)));
    CHECK(std::strcmp(s, "info") == 0);

    double d = -1.0;
    CHECK(detail::json_get_double("{\"radius_m\":500}", "radius_m", d));
    CHECK_NEAR(d, 500.0, 1e-9);

    CHECK(detail::json_get_double("{\"lat\":30.2741}", "lat", d));
    CHECK_NEAR(d, 30.2741, 1e-6);

    char cats[4][32] = {};
    const std::size_t n = detail::json_get_string_array(
        "{\"categories\":[\"kitchen\",\"door\"]}", "categories", cats, 4);
    CHECK(n == 2);
    CHECK(std::strcmp(cats[0], "kitchen") == 0);
    CHECK(std::strcmp(cats[1], "door") == 0);

    // 缺字段
    CHECK(!detail::json_get_string("{\"cmd\":\"info\"}", "nope", s, sizeof(s)));
    CHECK(!detail::json_get_double("{\"cmd\":\"info\"}", "lat", d));

    // 空数组
    CHECK(detail::json_get_string_array("{\"categories\":[]}", "categories",
                                        cats, 4) == 0);
    std::cout << "  [ok] json parsing\n";
}

void test_filter_allows_empty() {
    // 空过滤器放行一切
    CategoryFilter none{};
    CHECK(none.allows(SemanticCategory::Kitchen));
    CHECK(none.allows(SemanticCategory::Unknown));

    const char* k[] = {"kitchen"};
    CategoryFilter f{k, 1};
    CHECK(f.allows(SemanticCategory::Kitchen));
    CHECK(!f.allows(SemanticCategory::Door));
    std::cout << "  [ok] category filter semantics\n";
}

}  // namespace

int main() {
    std::cout << "== semantic service tests ==\n";
    test_haversine();
    test_gcj02_vs_wgs84_offset();
    test_category_filter();
    test_confidence_filter();
    test_sorted_ascending();
    test_radius_excludes();
    test_truncation();
    test_json_info();
    test_json_hits();
    test_json_error();
    test_json_parsing();
    test_filter_allows_empty();

    if (g_failures != 0) {
        std::cerr << "== " << g_failures << " failure(s) ==\n";
        return 1;
    }
    std::cout << "== all passed ==\n";
    return 0;
}
