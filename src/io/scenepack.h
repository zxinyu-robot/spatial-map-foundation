// ScenePack v1：端侧采集包（真机数据落盘与跨机交换的容器）。
//
// 分层对应「三平面 + 一缓存」：
//   控制面  manifest.txt       坐标系锚点 + 版本链 + 计数
//   索引面  poses.bin          关键帧绝对位姿（定长记录，可 seek）
//   索引面  edges.bin          约束边（相对位姿 + sqrtinfo 上三角）
//   索引面  semantic_tiles.txt 复用 map/semantic_persistence.h 格式
//   数据面  cloud/chunk_*.bin  本层只登记计数，不解码内容
//   校验    CHECKSUMS          FNV-1a 64 逐文件完整性
//
// 设计口径：定长记录、有界缓冲、返回码而非异常。
// 端序：假定小端（x86_64 / arm64 同构），跨异构端需另加字节序转换。
#pragma once

#include <cstdint>
#include <cstdio>
#include <cstring>

namespace spatial_map {

inline constexpr std::uint32_t kScenePackSchema = 1;
inline constexpr std::size_t kScenePackDeviceIdMax = 32;
inline constexpr std::size_t kScenePackPathMax = 128;
inline constexpr std::size_t kSqrtInfoCount = 21;  // 6x6 信息矩阵上三角

enum class ScenePackError : std::uint8_t {
    Ok = 0,
    OpenFailed,
    ReadFailed,
    WriteFailed,
    BadMagic,
    SchemaMismatch,
    VersionMismatch,
    ChecksumMismatch,
    CapacityExceeded,
};

[[nodiscard]] inline const char* scenepack_error_name(ScenePackError e) noexcept {
    switch (e) {
        case ScenePackError::Ok:               return "ok";
        case ScenePackError::OpenFailed:       return "open_failed";
        case ScenePackError::ReadFailed:       return "read_failed";
        case ScenePackError::WriteFailed:      return "write_failed";
        case ScenePackError::BadMagic:         return "bad_magic";
        case ScenePackError::SchemaMismatch:   return "schema_mismatch";
        case ScenePackError::VersionMismatch:  return "version_mismatch";
        case ScenePackError::ChecksumMismatch: return "checksum_mismatch";
        case ScenePackError::CapacityExceeded: return "capacity_exceeded";
    }
    return "unknown";
}

// ---------- 控制面 ----------

struct ScenePackManifest {
    std::uint32_t schema{kScenePackSchema};
    std::uint32_t map_version{0};
    std::uint64_t base_hash{0};
    double anchor_lat{0.0};
    double anchor_lon{0.0};
    double anchor_heading_rad{0.0};
    std::uint32_t pose_count{0};
    std::uint32_t edge_count{0};
    std::uint32_t chunk_count{0};
    std::int64_t created_ns{0};
    char device_id[kScenePackDeviceIdMax]{};
};

// ---------- 索引面·图 ----------

// 关键帧绝对位姿 T_w_i（ENU 米 + 单位四元数 x,y,z,w）。
struct PoseNode {
    std::uint32_t id{0};
    std::int64_t stamp_ns{0};
    float t[3]{};
    float q[4]{0.0f, 0.0f, 0.0f, 1.0f};
};
inline constexpr std::size_t kPoseNodeBytes = 4 + 8 + 12 + 16;  // 40

enum class EdgeType : std::uint8_t {
    Odometry = 0,
    Loop = 1,
    Imu = 2,
    Anchor = 3,
};

// 约束边：相对位姿 T_ij + 信息矩阵平方根上三角 + 置信度。
struct GraphEdge {
    std::uint32_t from{0};
    std::uint32_t to{0};
    EdgeType type{EdgeType::Odometry};
    float t_ij[3]{};
    float q_ij[4]{0.0f, 0.0f, 0.0f, 1.0f};
    float sqrt_info[kSqrtInfoCount]{};
    std::uint8_t confidence{0};
};

// ---------- 校验 ----------

struct ChecksumEntry {
    char path[kScenePackPathMax]{};
    std::uint64_t hash{0};
    std::uint64_t size{0};
};

// 整文件 FNV-1a 64。失败返回 0（空文件哈希非 0，0 可作错误哨兵）。
[[nodiscard]] inline std::uint64_t fnv1a64_file(const char* path,
                                                std::uint64_t* out_size = nullptr) noexcept {
    std::FILE* f = std::fopen(path, "rb");
    if (f == nullptr) return 0;

    std::uint64_t h = 1469598103934665603ULL;
    std::uint64_t total = 0;
    unsigned char buf[4096];
    std::size_t n = 0;
    while ((n = std::fread(buf, 1, sizeof(buf), f)) > 0) {
        for (std::size_t i = 0; i < n; ++i) {
            h ^= static_cast<std::uint64_t>(buf[i]);
            h *= 1099511628211ULL;
        }
        total += n;
    }
    std::fclose(f);
    if (out_size != nullptr) *out_size = total;
    return h;
}

namespace detail {

[[nodiscard]] inline bool w_u32(std::FILE* f, std::uint32_t v) noexcept {
    return std::fwrite(&v, sizeof(v), 1, f) == 1;
}
[[nodiscard]] inline bool w_u64(std::FILE* f, std::uint64_t v) noexcept {
    return std::fwrite(&v, sizeof(v), 1, f) == 1;
}
[[nodiscard]] inline bool w_i64(std::FILE* f, std::int64_t v) noexcept {
    return std::fwrite(&v, sizeof(v), 1, f) == 1;
}
[[nodiscard]] inline bool w_u8(std::FILE* f, std::uint8_t v) noexcept {
    return std::fwrite(&v, sizeof(v), 1, f) == 1;
}
[[nodiscard]] inline bool w_f32(std::FILE* f, const float* p, std::size_t n) noexcept {
    return std::fwrite(p, sizeof(float), n, f) == n;
}

[[nodiscard]] inline bool r_u32(std::FILE* f, std::uint32_t& v) noexcept {
    return std::fread(&v, sizeof(v), 1, f) == 1;
}
[[nodiscard]] inline bool r_u64(std::FILE* f, std::uint64_t& v) noexcept {
    return std::fread(&v, sizeof(v), 1, f) == 1;
}
[[nodiscard]] inline bool r_i64(std::FILE* f, std::int64_t& v) noexcept {
    return std::fread(&v, sizeof(v), 1, f) == 1;
}
[[nodiscard]] inline bool r_u8(std::FILE* f, std::uint8_t& v) noexcept {
    return std::fread(&v, sizeof(v), 1, f) == 1;
}
[[nodiscard]] inline bool r_f32(std::FILE* f, float* p, std::size_t n) noexcept {
    return std::fread(p, sizeof(float), n, f) == n;
}

// dir + "/" + name → 定长路径缓冲；超长则失败。
[[nodiscard]] inline bool join_path(char (&out)[kScenePackPathMax],
                                    const char* dir, const char* name) noexcept {
    const std::size_t dl = std::strlen(dir);
    const std::size_t nl = std::strlen(name);
    if (dl + 1 + nl + 1 > kScenePackPathMax) return false;
    std::memcpy(out, dir, dl);
    out[dl] = '/';
    std::memcpy(out + dl + 1, name, nl);
    out[dl + 1 + nl] = '\0';
    return true;
}

}  // namespace detail

// ---------- 控制面读写 ----------

[[nodiscard]] inline ScenePackError save_manifest(const ScenePackManifest& m,
                                                  const char* path) noexcept {
    std::FILE* f = std::fopen(path, "wb");
    if (f == nullptr) return ScenePackError::OpenFailed;

    const int rc = std::fprintf(
        f,
        "scenepack %u\n"
        "map_version=%u\n"
        "base_hash=%llu\n"
        "anchor_lat=%.9f\n"
        "anchor_lon=%.9f\n"
        "anchor_heading_rad=%.9f\n"
        "pose_count=%u\n"
        "edge_count=%u\n"
        "chunk_count=%u\n"
        "created_ns=%lld\n"
        "device_id=%s\n",
        m.schema, m.map_version,
        static_cast<unsigned long long>(m.base_hash),
        m.anchor_lat, m.anchor_lon, m.anchor_heading_rad,
        m.pose_count, m.edge_count, m.chunk_count,
        static_cast<long long>(m.created_ns), m.device_id);

    const bool ok = (rc > 0);
    std::fclose(f);
    return ok ? ScenePackError::Ok : ScenePackError::WriteFailed;
}

[[nodiscard]] inline ScenePackError load_manifest(ScenePackManifest& m,
                                                  const char* path) noexcept {
    std::FILE* f = std::fopen(path, "rb");
    if (f == nullptr) return ScenePackError::OpenFailed;

    char line[256];
    bool magic_ok = false;

    while (std::fgets(line, sizeof(line), f) != nullptr) {
        if (!magic_ok) {
            unsigned int schema = 0;
            if (std::sscanf(line, "scenepack %u", &schema) == 1) {
                if (schema != kScenePackSchema) {
                    std::fclose(f);
                    return ScenePackError::SchemaMismatch;
                }
                m.schema = schema;
                magic_ok = true;
                continue;
            }
            std::fclose(f);
            return ScenePackError::BadMagic;
        }

        unsigned int u = 0;
        unsigned long long ull = 0;
        long long i64 = 0;
        if (std::sscanf(line, "map_version=%u", &u) == 1)                  { m.map_version = u; }
        else if (std::sscanf(line, "base_hash=%llu", &ull) == 1)           { m.base_hash = ull; }
        else if (std::sscanf(line, "anchor_lat=%lf", &m.anchor_lat) == 1)  {}
        else if (std::sscanf(line, "anchor_lon=%lf", &m.anchor_lon) == 1)  {}
        else if (std::sscanf(line, "anchor_heading_rad=%lf",
                             &m.anchor_heading_rad) == 1)                  {}
        else if (std::sscanf(line, "pose_count=%u", &u) == 1)              { m.pose_count = u; }
        else if (std::sscanf(line, "edge_count=%u", &u) == 1)              { m.edge_count = u; }
        else if (std::sscanf(line, "chunk_count=%u", &u) == 1)             { m.chunk_count = u; }
        else if (std::sscanf(line, "created_ns=%lld", &i64) == 1)          { m.created_ns = i64; }
        else if (std::sscanf(line, "device_id=%31s", m.device_id) == 1)    {}
    }

    std::fclose(f);
    return magic_ok ? ScenePackError::Ok : ScenePackError::BadMagic;
}

// ---------- 索引面·图读写 ----------

[[nodiscard]] inline ScenePackError save_poses(const PoseNode* poses, std::uint32_t n,
                                               const char* path) noexcept {
    std::FILE* f = std::fopen(path, "wb");
    if (f == nullptr) return ScenePackError::OpenFailed;

    bool ok = true;
    for (std::uint32_t i = 0; i < n && ok; ++i) {
        const PoseNode& p = poses[i];
        ok = detail::w_u32(f, p.id) && detail::w_i64(f, p.stamp_ns) &&
             detail::w_f32(f, p.t, 3) && detail::w_f32(f, p.q, 4);
    }
    std::fclose(f);
    return ok ? ScenePackError::Ok : ScenePackError::WriteFailed;
}

template <std::size_t MaxPoses>
[[nodiscard]] inline ScenePackError load_poses(PoseNode (&poses)[MaxPoses],
                                               std::uint32_t& count,
                                               const char* path) noexcept {
    std::FILE* f = std::fopen(path, "rb");
    if (f == nullptr) return ScenePackError::OpenFailed;

    count = 0;
    while (count < MaxPoses) {
        PoseNode p;
        const bool ok = detail::r_u32(f, p.id) && detail::r_i64(f, p.stamp_ns) &&
                        detail::r_f32(f, p.t, 3) && detail::r_f32(f, p.q, 4);
        if (!ok) break;
        poses[count++] = p;
    }

    // 容量已满但文件仍有剩余 → 包比缓冲大。
    const bool overflow = (count == MaxPoses) && (std::fgetc(f) != EOF);
    std::fclose(f);
    return overflow ? ScenePackError::CapacityExceeded : ScenePackError::Ok;
}

[[nodiscard]] inline ScenePackError save_edges(const GraphEdge* edges, std::uint32_t n,
                                               const char* path) noexcept {
    std::FILE* f = std::fopen(path, "wb");
    if (f == nullptr) return ScenePackError::OpenFailed;

    bool ok = true;
    for (std::uint32_t i = 0; i < n && ok; ++i) {
        const GraphEdge& e = edges[i];
        ok = detail::w_u32(f, e.from) && detail::w_u32(f, e.to) &&
             detail::w_u8(f, static_cast<std::uint8_t>(e.type)) &&
             detail::w_f32(f, e.t_ij, 3) && detail::w_f32(f, e.q_ij, 4) &&
             detail::w_f32(f, e.sqrt_info, kSqrtInfoCount) &&
             detail::w_u8(f, e.confidence);
    }
    std::fclose(f);
    return ok ? ScenePackError::Ok : ScenePackError::WriteFailed;
}

template <std::size_t MaxEdges>
[[nodiscard]] inline ScenePackError load_edges(GraphEdge (&edges)[MaxEdges],
                                               std::uint32_t& count,
                                               const char* path) noexcept {
    std::FILE* f = std::fopen(path, "rb");
    if (f == nullptr) return ScenePackError::OpenFailed;

    count = 0;
    while (count < MaxEdges) {
        GraphEdge e;
        std::uint8_t type = 0;
        const bool ok = detail::r_u32(f, e.from) && detail::r_u32(f, e.to) &&
                        detail::r_u8(f, type) &&
                        detail::r_f32(f, e.t_ij, 3) && detail::r_f32(f, e.q_ij, 4) &&
                        detail::r_f32(f, e.sqrt_info, kSqrtInfoCount) &&
                        detail::r_u8(f, e.confidence);
        if (!ok) break;
        e.type = static_cast<EdgeType>(type);
        edges[count++] = e;
    }

    const bool overflow = (count == MaxEdges) && (std::fgetc(f) != EOF);
    std::fclose(f);
    return overflow ? ScenePackError::CapacityExceeded : ScenePackError::Ok;
}

// ---------- 校验清单 ----------

[[nodiscard]] inline ScenePackError save_checksums(const ChecksumEntry* entries,
                                                   std::size_t n,
                                                   const char* path) noexcept {
    std::FILE* f = std::fopen(path, "wb");
    if (f == nullptr) return ScenePackError::OpenFailed;

    bool ok = true;
    for (std::size_t i = 0; i < n && ok; ++i) {
        ok = std::fprintf(f, "%016llx  %llu  %s\n",
                          static_cast<unsigned long long>(entries[i].hash),
                          static_cast<unsigned long long>(entries[i].size),
                          entries[i].path) > 0;
    }
    std::fclose(f);
    return ok ? ScenePackError::Ok : ScenePackError::WriteFailed;
}

// 逐文件重算并比对；任何一项不符即返回 ChecksumMismatch。
[[nodiscard]] inline ScenePackError verify_checksums(const char* path) noexcept {
    std::FILE* f = std::fopen(path, "rb");
    if (f == nullptr) return ScenePackError::OpenFailed;

    char line[512];
    ScenePackError result = ScenePackError::Ok;
    while (std::fgets(line, sizeof(line), f) != nullptr) {
        unsigned long long hash = 0, size = 0;
        char p[kScenePackPathMax] = {};
        if (std::sscanf(line, "%llx  %llu  %127s", &hash, &size, p) != 3) continue;

        std::uint64_t actual_size = 0;
        const std::uint64_t actual = fnv1a64_file(p, &actual_size);
        if (actual == 0 || actual != hash || actual_size != size) {
            result = ScenePackError::ChecksumMismatch;
            break;
        }
    }
    std::fclose(f);
    return result;
}

// ---------- 整包校验（验收标准唯一入口） ----------

// expected_map_version 为 0 表示不校验版本。
// 顺序：先校验和（防篡改/截断），再读 manifest（读版本）。
[[nodiscard]] inline ScenePackError verify_scenepack(const char* dir,
                                                     std::uint32_t expected_map_version) noexcept {
    char p[kScenePackPathMax];

    if (!detail::join_path(p, dir, "CHECKSUMS")) return ScenePackError::CapacityExceeded;
    {
        const ScenePackError e = verify_checksums(p);
        if (e != ScenePackError::Ok) return e;
    }

    if (!detail::join_path(p, dir, "manifest.txt")) return ScenePackError::CapacityExceeded;
    ScenePackManifest m;
    const ScenePackError me = load_manifest(m, p);
    if (me != ScenePackError::Ok) return me;

    if (expected_map_version != 0 && m.map_version != expected_map_version) {
        return ScenePackError::VersionMismatch;
    }
    return ScenePackError::Ok;
}

}  // namespace spatial_map
