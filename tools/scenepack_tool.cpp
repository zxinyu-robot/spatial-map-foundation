// ScenePack v1 工具：真机数据打包与校验。
//
// 用法：
//   scenepack_tool pack <out_dir> <anchor_lat> <anchor_lon> <heading_rad>
//                       <map_version> <poses.txt> [edges.txt] [semantic_tiles.txt]
//   scenepack_tool verify <dir> [expected_map_version]
//
// poses.txt（每行一个关键帧，真机 Super-LIO / 里程计直接吐即可）：
//   <id> <stamp_ns> <x> <y> <z> <qx> <qy> <qz> <qw>
//
// edges.txt（可选，每行一条约束边）：
//   <from> <to> <type:odom|loop|imu|anchor> <tx> <ty> <tz> <qx> <qy> <qz> <qw> <confidence>
//
// 本工具只处理索引面（位姿图 + 语义瓦片）；数据面 chunk 只登记计数。
// 语义瓦片文件按字节原样拷入包内 —— 解析由下游用 semantic_persistence.h 负责。
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>

#include "io/scenepack.h"

namespace {

using namespace spatial_map;

constexpr std::size_t kMaxPoses = 4096;
constexpr std::size_t kMaxEdges = 8192;

[[nodiscard]] EdgeType parse_edge_type(const char* s) noexcept {
    if (std::strcmp(s, "loop") == 0) return EdgeType::Loop;
    if (std::strcmp(s, "imu") == 0) return EdgeType::Imu;
    if (std::strcmp(s, "anchor") == 0) return EdgeType::Anchor;
    return EdgeType::Odometry;
}

// 返回读取的行数；-1 表示打不开。
[[nodiscard]] std::int64_t read_poses(const std::string& path,
                                      PoseNode (&poses)[kMaxPoses]) noexcept {
    std::FILE* f = std::fopen(path.c_str(), "rb");
    if (f == nullptr) return -1;

    std::int64_t n = 0;
    char line[512];
    while (std::fgets(line, sizeof(line), f) != nullptr) {
        if (line[0] == '#') continue;
        PoseNode p;
        long long stamp = 0;
        if (std::sscanf(line, "%u %lld %f %f %f %f %f %f %f",
                        &p.id, &stamp,
                        &p.t[0], &p.t[1], &p.t[2],
                        &p.q[0], &p.q[1], &p.q[2], &p.q[3]) != 9) {
            continue;
        }
        if (static_cast<std::size_t>(n) >= kMaxPoses) break;
        p.stamp_ns = stamp;
        poses[n++] = p;
    }
    std::fclose(f);
    return n;
}

[[nodiscard]] std::int64_t read_edges(const std::string& path,
                                      GraphEdge (&edges)[kMaxEdges]) noexcept {
    std::FILE* f = std::fopen(path.c_str(), "rb");
    if (f == nullptr) return -1;

    std::int64_t n = 0;
    char line[512];
    while (std::fgets(line, sizeof(line), f) != nullptr) {
        if (line[0] == '#') continue;
        GraphEdge e;
        char type[16] = {};
        unsigned int conf = 0;
        if (std::sscanf(line, "%u %u %15s %f %f %f %f %f %f %f %u",
                        &e.from, &e.to, type,
                        &e.t_ij[0], &e.t_ij[1], &e.t_ij[2],
                        &e.q_ij[0], &e.q_ij[1], &e.q_ij[2], &e.q_ij[3],
                        &conf) != 11) {
            continue;
        }
        if (static_cast<std::size_t>(n) >= kMaxEdges) break;
        e.type = parse_edge_type(type);
        e.confidence = static_cast<std::uint8_t>(conf > 255U ? 255U : conf);
        // sqrtinfo 暂置单位阵上三角（1.0），真实系统应填入协方差平方根。
        for (std::size_t i = 0; i < kSqrtInfoCount; ++i) e.sqrt_info[i] = 0.0f;
        e.sqrt_info[0] = 1.0f;
        e.sqrt_info[6] = 1.0f;
        e.sqrt_info[11] = 1.0f;
        e.sqrt_info[15] = 1.0f;
        e.sqrt_info[18] = 1.0f;
        e.sqrt_info[20] = 1.0f;
        edges[n++] = e;
    }
    std::fclose(f);
    return n;
}

[[nodiscard]] int do_pack(int argc, char** argv) {
    if (argc < 8) {
        std::cerr << "用法: scenepack_tool pack <out_dir> <anchor_lat> <anchor_lon>"
                     " <heading_rad> <map_version> <poses.txt> [edges.txt]"
                     " [semantic_tiles.txt]\n";
        return 2;
    }

    const std::string dir = argv[2];
    ScenePackManifest m;
    m.anchor_lat = std::atof(argv[3]);
    m.anchor_lon = std::atof(argv[4]);
    m.anchor_heading_rad = std::atof(argv[5]);
    m.map_version = static_cast<std::uint32_t>(std::atoi(argv[6]));
    m.created_ns = 0;

    static PoseNode poses[kMaxPoses];
    const std::int64_t pn = read_poses(argv[7], poses);
    if (pn < 0) {
        std::cerr << "打开 poses 文件失败: " << argv[7] << "\n";
        return 1;
    }
    m.pose_count = static_cast<std::uint32_t>(pn);

    static GraphEdge edges[kMaxEdges];
    std::uint32_t en = 0;
    if (argc >= 9) {
        const std::int64_t r = read_edges(argv[8], edges);
        if (r < 0) {
            std::cerr << "打开 edges 文件失败: " << argv[8] << "\n";
            return 1;
        }
        en = static_cast<std::uint32_t>(r);
    }
    m.edge_count = en;

    // base_hash：以 poses 内容为种子，保证同批数据同 hash。
    std::uint64_t h = 1469598103934665603ULL;
    for (std::uint32_t i = 0; i < m.pose_count; ++i) {
        const unsigned char* raw = reinterpret_cast<const unsigned char*>(&poses[i]);
        for (std::size_t b = 0; b < sizeof(PoseNode); ++b) {
            h ^= raw[b];
            h *= 1099511628211ULL;
        }
    }
    m.base_hash = h;
    std::snprintf(m.device_id, sizeof(m.device_id), "%s", "scenepack_tool");

    char p[kScenePackPathMax];
    if (!detail::join_path(p, dir.c_str(), "manifest.txt")) {
        std::cerr << "输出路径过长\n";
        return 1;
    }
    if (save_manifest(m, p) != ScenePackError::Ok) {
        std::cerr << "写 manifest 失败: " << p << "\n";
        return 1;
    }

    if (!detail::join_path(p, dir.c_str(), "poses.bin")) return 1;
    if (save_poses(poses, m.pose_count, p) != ScenePackError::Ok) {
        std::cerr << "写 poses 失败\n";
        return 1;
    }

    ChecksumEntry entries[4] = {};
    std::size_t ne = 0;

    std::snprintf(entries[ne].path, kScenePackPathMax, "%s/manifest.txt", dir.c_str());
    ++ne;

    if (en > 0) {
        if (!detail::join_path(p, dir.c_str(), "edges.bin")) return 1;
        if (save_edges(edges, en, p) != ScenePackError::Ok) {
            std::cerr << "写 edges 失败\n";
            return 1;
        }
    }

    std::snprintf(entries[ne].path, kScenePackPathMax, "%s/poses.bin", dir.c_str());
    ++ne;
    if (en > 0) {
        std::snprintf(entries[ne].path, kScenePackPathMax, "%s/edges.bin", dir.c_str());
        ++ne;
    }

    // 语义瓦片：字节原样拷入，供下游用 semantic_persistence.h 解析。
    if (argc >= 10) {
        std::FILE* src = std::fopen(argv[9], "rb");
        if (src == nullptr) {
            std::cerr << "打开 semantic_tiles 失败: " << argv[9] << "\n";
            return 1;
        }
        if (!detail::join_path(p, dir.c_str(), "semantic_tiles.txt")) {
            std::fclose(src);
            return 1;
        }
        std::FILE* dst = std::fopen(p, "wb");
        if (dst == nullptr) {
            std::fclose(src);
            std::cerr << "写 semantic_tiles 失败\n";
            return 1;
        }
        char buf[4096];
        std::size_t n = 0;
        bool copy_ok = true;
        while ((n = std::fread(buf, 1, sizeof(buf), src)) > 0) {
            if (std::fwrite(buf, 1, n, dst) != n) { copy_ok = false; break; }
        }
        std::fclose(src);
        std::fclose(dst);
        if (!copy_ok) {
            std::cerr << "拷贝 semantic_tiles 失败\n";
            return 1;
        }
        std::snprintf(entries[ne].path, kScenePackPathMax,
                      "%s/semantic_tiles.txt", dir.c_str());
        ++ne;
    }

    for (std::size_t i = 0; i < ne; ++i) {
        std::uint64_t size = 0;
        entries[i].hash = fnv1a64_file(entries[i].path, &size);
        entries[i].size = size;
        if (entries[i].hash == 0) {
            std::cerr << "校验和计算失败: " << entries[i].path << "\n";
            return 1;
        }
    }

    if (!detail::join_path(p, dir.c_str(), "CHECKSUMS")) return 1;
    if (save_checksums(entries, ne, p) != ScenePackError::Ok) {
        std::cerr << "写 CHECKSUMS 失败\n";
        return 1;
    }

    std::cout << "pack ok: " << dir << "\n"
              << "  map_version = " << m.map_version << "\n"
              << "  base_hash   = " << std::hex << m.base_hash << std::dec << "\n"
              << "  poses       = " << m.pose_count << "\n"
              << "  edges       = " << m.edge_count << "\n"
              << "  files       = " << ne << "\n";
    return 0;
}

[[nodiscard]] int do_verify(int argc, char** argv) {
    if (argc < 3) {
        std::cerr << "用法: scenepack_tool verify <dir> [expected_map_version]\n";
        return 2;
    }
    const std::uint32_t expect =
        (argc >= 4) ? static_cast<std::uint32_t>(std::atoi(argv[3])) : 0U;

    const ScenePackError e = verify_scenepack(argv[2], expect);
    std::cout << "verify " << argv[2] << ": " << scenepack_error_name(e) << "\n";
    return (e == ScenePackError::Ok) ? 0 : 1;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "用法: scenepack_tool <pack|verify> ...\n";
        return 2;
    }
    if (std::strcmp(argv[1], "pack") == 0) return do_pack(argc, argv);
    if (std::strcmp(argv[1], "verify") == 0) return do_verify(argc, argv);

    std::cerr << "未知子命令: " << argv[1] << "\n";
    return 2;
}
