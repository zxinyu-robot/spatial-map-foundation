// 语义瓦片查询服务（TCP，行分隔 JSON）。
//
// 用法：
//   semantic_tile_service --file <semantic_tiles.txt> [--port 9881]
//                         [--host 127.0.0.1] [--anchor-lat 30.2741] [--anchor-lon 120.1551]
//
// 协议见 src/runtime/semantic_service.h 头部。一次连接可发多条请求，
// 每条一行，逐行应答。
//
// 只读服务：不改瓦片。老师标注回写走文件，由上层显式触发。
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include "map/semantic_persistence.h"
#include "map/semantic_tile.h"
#include "runtime/semantic_service.h"

namespace {

using namespace spatial_map;

constexpr std::size_t kStoreCapacity = 4096;
constexpr std::size_t kMaxCategories = 16;
constexpr std::size_t kRequestMax = 2048;

struct Options {
    std::string file;
    int port = 9881;
    std::string host = "127.0.0.1";
    double anchor_lat = 0.0;
    double anchor_lon = 0.0;
    bool default_confidence_zero = true;
};

[[nodiscard]] bool parse_args(int argc, char** argv, Options& opt) {
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        auto next = [&](const char* name) -> const char* {
            if (i + 1 >= argc) {
                std::cerr << name << " 缺少参数值\n";
                return nullptr;
            }
            return argv[++i];
        };
        if (a == "--file") {
            const char* v = next("--file");
            if (v == nullptr) return false;
            opt.file = v;
        } else if (a == "--port") {
            const char* v = next("--port");
            if (v == nullptr) return false;
            opt.port = std::atoi(v);
        } else if (a == "--host") {
            const char* v = next("--host");
            if (v == nullptr) return false;
            opt.host = v;
        } else if (a == "--anchor-lat") {
            const char* v = next("--anchor-lat");
            if (v == nullptr) return false;
            opt.anchor_lat = std::atof(v);
        } else if (a == "--anchor-lon") {
            const char* v = next("--anchor-lon");
            if (v == nullptr) return false;
            opt.anchor_lon = std::atof(v);
        } else if (a == "-h" || a == "--help") {
            std::cout << "用法: semantic_tile_service --file <semantic_tiles.txt>"
                         " [--port 9881] [--host 127.0.0.1]"
                         " [--anchor-lat 0] [--anchor-lon 0]\n";
            return false;
        } else {
            std::cerr << "未知参数: " << a << "\n";
            return false;
        }
    }
    if (opt.file.empty()) {
        std::cerr << "必须指定 --file\n";
        return false;
    }
    (void)opt.default_confidence_zero;
    return true;
}

// 处理一行请求，写入 reply（含换行）。返回 false 表示应答构造失败。
[[nodiscard]] bool handle_request(const char* req,
                                 const SemanticTileStore<kStoreCapacity>& store,
                                 std::uint32_t map_version, std::uint32_t zoom,
                                 double anchor_lat, double anchor_lon,
                                 char (&reply)[kServiceReplyMax]) {
    char cmd[32] = {};
    if (!detail::json_get_string(req, "cmd", cmd, sizeof(cmd))) {
        return build_error_json(reply, "missing cmd");
    }

    if (std::strcmp(cmd, "info") == 0) {
        return build_info_json(reply, map_version, zoom, store.size(),
                               anchor_lat, anchor_lon);
    }

    const bool is_query = (std::strcmp(cmd, "query") == 0);
    const bool is_tiles = (std::strcmp(cmd, "tiles") == 0);
    if (!is_query && !is_tiles) {
        return build_error_json(reply, "unknown cmd");
    }

    // 类别过滤
    static char cats[kMaxCategories][32];
    const std::size_t n_cats = detail::json_get_string_array(
        req, "categories", cats, kMaxCategories);
    const char* cat_ptrs[kMaxCategories];
    for (std::size_t i = 0; i < n_cats; ++i) cat_ptrs[i] = cats[i];
    CategoryFilter filter{cat_ptrs, n_cats};

    double min_conf_d = 0.0;
    // 缺省视为 0（不过滤置信度）
    if (!detail::json_get_double(req, "min_confidence", min_conf_d)) min_conf_d = 0.0;
    const std::uint8_t min_conf =
        static_cast<std::uint8_t>(min_conf_d < 0.0 ? 0.0 : (min_conf_d > 100.0 ? 100.0 : min_conf_d));

    static SemanticHit hits[kMaxQueryResults];
    bool truncated = false;
    std::size_t count = 0;
    bool with_dist = false;

    if (is_query) {
        double lat = 0.0, lon = 0.0, radius = 0.0;
        if (!detail::json_get_double(req, "lat", lat) ||
            !detail::json_get_double(req, "lon", lon)) {
            return build_error_json(reply, "query requires lat/lon");
        }
        if (!detail::json_get_double(req, "radius_m", radius) || radius <= 0.0) {
            radius = 500.0;  // 默认半径
        }
        count = query_radius(store, lat, lon, radius, filter, min_conf, hits, truncated);
        with_dist = true;
    } else {
        // tiles：全量（受 kMaxQueryResults 约束），不做距离过滤
        store.for_each([&](const SemanticTile& t) {
            if (count >= kMaxQueryResults) { truncated = true; return; }
            bool any = false;
            for (std::uint8_t i = 0; i < t.tag_count; ++i) {
                if (t.tags[i].confidence < min_conf) continue;
                if (!filter.allows(t.tags[i].category)) continue;
                any = true;
                break;
            }
            if (!any) return;
            hits[count++] = SemanticHit{t.key, &t, 0.0};
        });
    }

    return build_hits_json(reply, map_version, hits, count, truncated, with_dist);
}

}  // namespace

int main(int argc, char** argv) {
    Options opt;
    if (!parse_args(argc, argv, opt)) return 2;

    static SemanticTileStore<kStoreCapacity> store;
    std::uint32_t zoom = 0;
    std::uint32_t map_version = 0;
    const std::int64_t n = load_semantic_tiles(store, zoom, opt.file, &map_version);
    if (n < 0) {
        std::cerr << "读取语义瓦片失败: " << opt.file << "\n";
        return 1;
    }
    std::cout << "已加载 " << n << " 个语义瓦片 (zoom=" << zoom
              << ", map_version=" << map_version << ")\n";
    if (map_version == 0) {
        std::cerr << "警告: map_version=0（v1 格式或无版本），消费方将无法校验版本一致性\n";
    }

    const int srv = ::socket(AF_INET, SOCK_STREAM, 0);
    if (srv < 0) {
        std::perror("socket");
        return 1;
    }
    const int one = 1;
    ::setsockopt(srv, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<std::uint16_t>(opt.port));
    if (::inet_pton(AF_INET, opt.host.c_str(), &addr.sin_addr) != 1) {
        std::cerr << "非法 host: " << opt.host << "\n";
        ::close(srv);
        return 1;
    }
    if (::bind(srv, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        std::perror("bind");
        ::close(srv);
        return 1;
    }
    if (::listen(srv, 8) < 0) {
        std::perror("listen");
        ::close(srv);
        return 1;
    }
    std::cout << "语义瓦片服务监听 " << opt.host << ":" << opt.port << "\n";
    std::cout.flush();

    for (;;) {
        const int cli = ::accept(srv, nullptr, nullptr);
        if (cli < 0) continue;

        std::string acc;
        char buf[1024];
        for (;;) {
            const ssize_t r = ::recv(cli, buf, sizeof(buf), 0);
            if (r <= 0) break;
            acc.append(buf, static_cast<std::size_t>(r));

            // 逐行处理
            std::size_t nl = 0;
            while ((nl = acc.find('\n')) != std::string::npos) {
                std::string line = acc.substr(0, nl);
                acc.erase(0, nl + 1);
                if (line.empty()) continue;
                if (line.size() >= kRequestMax) {
                    char err[kServiceReplyMax];
                    (void)build_error_json(err, "request too long");
                    const std::string out = std::string(err) + "\n";
                    ::send(cli, out.c_str(), out.size(), 0);
                    continue;
                }
                char reply[kServiceReplyMax];
                if (!handle_request(line.c_str(), store, map_version, zoom,
                                    opt.anchor_lat, opt.anchor_lon, reply)) {
                    (void)build_error_json(reply, "reply overflow");
                }
                const std::string out = std::string(reply) + "\n";
                ::send(cli, out.c_str(), out.size(), 0);
            }
        }
        ::close(cli);
    }
}
