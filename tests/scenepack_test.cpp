// ScenePack v1 测试：round-trip、版本不匹配拒绝、校验和篡改拒绝。
//
// 用显式 CHECK 而非 assert —— Release 构建下 NDEBUG 会关掉 assert。
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <iostream>

#include <sys/stat.h>

#include "io/scenepack.h"

namespace {

int g_failures = 0;

#define CHECK(cond)                                                       \
    do {                                                                  \
        if (!(cond)) {                                                    \
            std::cerr << "  [FAIL] " << #cond << " @ line " << __LINE__   \
                      << "\n";                                            \
            ++g_failures;                                                 \
        }                                                                 \
    } while (0)

constexpr const char* kDir = "/tmp/scenepack_test_dir";

using namespace spatial_map;

void write_pack(std::uint32_t map_version) {
    // 测试自建目录：不依赖外部预先 mkdir，否则单独运行时莫名失败。
    ::mkdir(kDir, 0755);

    ScenePackManifest m;
    m.map_version = map_version;
    m.base_hash = 0xa1b2c3d4e5f60718ULL;
    m.anchor_lat = 30.2741;
    m.anchor_lon = 120.1551;
    m.anchor_heading_rad = 0.785398163;
    m.pose_count = 3;
    m.edge_count = 2;
    m.chunk_count = 4;
    m.created_ns = 1730000000000000000LL;
    std::snprintf(m.device_id, sizeof(m.device_id), "%s", "go2-orin-01");

    char p[kScenePackPathMax];

    CHECK(detail::join_path(p, kDir, "manifest.txt"));
    CHECK(save_manifest(m, p) == ScenePackError::Ok);

    PoseNode poses[3];
    for (std::uint32_t i = 0; i < 3; ++i) {
        poses[i].id = i + 1;
        poses[i].stamp_ns = 1000000LL * (i + 1);
        poses[i].t[0] = 0.5f * static_cast<float>(i);
        poses[i].t[1] = 1.5f * static_cast<float>(i);
        poses[i].t[2] = 0.25f;
        poses[i].q[0] = 0.0f;
        poses[i].q[1] = 0.0f;
        poses[i].q[2] = 0.0f;
        poses[i].q[3] = 1.0f;
    }
    CHECK(detail::join_path(p, kDir, "poses.bin"));
    CHECK(save_poses(poses, 3, p) == ScenePackError::Ok);

    GraphEdge edges[2];
    edges[0].from = 1;
    edges[0].to = 2;
    edges[0].type = EdgeType::Odometry;
    edges[0].t_ij[0] = 1.0f;
    edges[0].confidence = 200;
    edges[0].sqrt_info[0] = 100.0f;
    edges[1].from = 3;
    edges[1].to = 1;
    edges[1].type = EdgeType::Loop;
    edges[1].t_ij[1] = -2.0f;
    edges[1].confidence = 120;
    edges[1].sqrt_info[20] = 50.0f;
    CHECK(detail::join_path(p, kDir, "edges.bin"));
    CHECK(save_edges(edges, 2, p) == ScenePackError::Ok);

    ChecksumEntry entries[3] = {};
    const char* names[3] = {"manifest.txt", "poses.bin", "edges.bin"};
    for (std::size_t i = 0; i < 3; ++i) {
        CHECK(detail::join_path(entries[i].path, kDir, names[i]));
        std::uint64_t size = 0;
        entries[i].hash = fnv1a64_file(entries[i].path, &size);
        entries[i].size = size;
        CHECK(entries[i].hash != 0);
    }
    CHECK(detail::join_path(p, kDir, "CHECKSUMS"));
    CHECK(save_checksums(entries, 3, p) == ScenePackError::Ok);
}

void test_roundtrip() {
    write_pack(42);

    char p[kScenePackPathMax];
    CHECK(detail::join_path(p, kDir, "manifest.txt"));
    ScenePackManifest m;
    CHECK(load_manifest(m, p) == ScenePackError::Ok);
    CHECK(m.schema == kScenePackSchema);
    CHECK(m.map_version == 42);
    CHECK(m.base_hash == 0xa1b2c3d4e5f60718ULL);
    CHECK(m.pose_count == 3);
    CHECK(m.edge_count == 2);
    CHECK(m.chunk_count == 4);
    CHECK(m.created_ns == 1730000000000000000LL);
    CHECK(m.anchor_lat == 30.2741);
    CHECK(std::strcmp(m.device_id, "go2-orin-01") == 0);

    CHECK(detail::join_path(p, kDir, "poses.bin"));
    PoseNode poses[8];
    std::uint32_t pc = 0;
    CHECK(load_poses(poses, pc, p) == ScenePackError::Ok);
    CHECK(pc == 3);
    CHECK(poses[1].id == 2);
    CHECK(poses[2].t[1] == 3.0f);
    CHECK(poses[0].q[3] == 1.0f);
    CHECK(poses[2].stamp_ns == 3000000LL);

    CHECK(detail::join_path(p, kDir, "edges.bin"));
    GraphEdge edges[8];
    std::uint32_t ec = 0;
    CHECK(load_edges(edges, ec, p) == ScenePackError::Ok);
    CHECK(ec == 2);
    CHECK(edges[0].type == EdgeType::Odometry);
    CHECK(edges[1].type == EdgeType::Loop);
    CHECK(edges[1].sqrt_info[20] == 50.0f);
    CHECK(edges[0].from == 1);
    CHECK(edges[1].to == 1);

    std::cout << "  [ok] roundtrip\n";
}

void test_capacity_exceeded() {
    write_pack(42);
    char p[kScenePackPathMax];
    CHECK(detail::join_path(p, kDir, "poses.bin"));

    PoseNode tiny[2];                       // 文件有 3 条，容量只有 2
    std::uint32_t pc = 0;
    CHECK(load_poses(tiny, pc, p) == ScenePackError::CapacityExceeded);
    CHECK(pc == 2);
    std::cout << "  [ok] capacity exceeded detected\n";
}

void test_version_mismatch() {
    write_pack(42);
    CHECK(verify_scenepack(kDir, 42) == ScenePackError::Ok);
    CHECK(verify_scenepack(kDir, 43) == ScenePackError::VersionMismatch);
    CHECK(verify_scenepack(kDir, 0) == ScenePackError::Ok);  // 0 = 不校验
    std::cout << "  [ok] version mismatch rejected\n";
}

void test_checksum_tamper() {
    write_pack(42);
    char p[kScenePackPathMax];
    CHECK(detail::join_path(p, kDir, "poses.bin"));

    std::FILE* f = std::fopen(p, "ab");
    CHECK(f != nullptr);
    if (f != nullptr) {
        const unsigned char junk = 0xff;
        std::fwrite(&junk, 1, 1, f);
        std::fclose(f);
    }
    CHECK(verify_scenepack(kDir, 42) == ScenePackError::ChecksumMismatch);
    std::cout << "  [ok] checksum tamper rejected\n";
}

void test_bad_magic() {
    char p[kScenePackPathMax];
    CHECK(detail::join_path(p, kDir, "manifest.txt"));

    std::FILE* f = std::fopen(p, "wb");
    CHECK(f != nullptr);
    if (f != nullptr) {
        std::fputs("not-a-scenepack\n", f);
        std::fclose(f);
    }
    ScenePackManifest m;
    CHECK(load_manifest(m, p) == ScenePackError::BadMagic);
    std::cout << "  [ok] bad magic rejected\n";
}

}  // namespace

int main() {
    std::cout << "== scenepack v1 tests ==\n";
    test_roundtrip();
    test_capacity_exceeded();
    test_version_mismatch();
    test_checksum_tamper();
    test_bad_magic();

    if (g_failures != 0) {
        std::cerr << "== " << g_failures << " failure(s) ==\n";
        return 1;
    }
    std::cout << "== all passed ==\n";
    return 0;
}
