// 语义标签回写合并工具：把老师（VLM）标注叠加到语义瓦片上。
//
// 为什么用 C++ 做合并而不是 Python：
// 标注里给的是 WGS84 经纬度，落瓦片要经过 GCJ-02 换算。这个换算只能有
// 一份实现（map/amap_tile.h）—— 在 Python 里再写一遍，必然重演
// 「GCJ-02 当 WGS84 用」那类 500m 偏移 bug。
//
// 回滚设计：base 瓦片文件**只读不改**，合并结果另存。
// 回滚 = 丢掉输出文件，重跑下一轮。
//
// 版本语义：合并改变了地图内容，故默认 map_version +1。
// 这不是形式主义 —— map_version 是 trace 样本的连接键，不递增会让
// 「标注前采集的样本」和「标注后的地图」错误配对。
//
// 用法：
//   semantic_label_merge --base <tiles.txt> --overlay <labels.jsonl>
//                        --out <merged.txt> [--zoom 18]
//                        [--expected-version N] [--no-bump]
//
// overlay 每行一条（JSONL）：
//   {"map_version":7,"lat":30.2741,"lon":120.1551,
//    "category":"kitchen","confidence":85,"source":"qwen_vl"}
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>

#include "map/semantic_persistence.h"
#include "map/semantic_tile.h"
#include "runtime/semantic_label_overlay.h"

namespace {

using namespace spatial_map;

constexpr std::size_t kStoreCapacity = 4096;
constexpr std::size_t kLineMax = 1024;

}  // namespace

int main(int argc, char** argv) {
    std::string base_path;
    std::string overlay_path;
    std::string out_path;
    std::uint32_t zoom = 0;                 // 0 = 沿用 base 文件的 zoom
    std::uint32_t expected_version = 0;     // 0 = 不校验
    bool bump = true;

    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        auto next = [&](const char* name) -> const char* {
            if (i + 1 >= argc) {
                std::cerr << name << " 缺少参数值\n";
                return nullptr;
            }
            return argv[++i];
        };
        if (a == "--base") {
            const char* v = next("--base"); if (v == nullptr) return 2;  base_path = v;
        } else if (a == "--overlay") {
            const char* v = next("--overlay"); if (v == nullptr) return 2; overlay_path = v;
        } else if (a == "--out") {
            const char* v = next("--out"); if (v == nullptr) return 2;  out_path = v;
        } else if (a == "--zoom") {
            const char* v = next("--zoom"); if (v == nullptr) return 2;
            zoom = static_cast<std::uint32_t>(std::atoi(v));
        } else if (a == "--expected-version") {
            const char* v = next("--expected-version"); if (v == nullptr) return 2;
            expected_version = static_cast<std::uint32_t>(std::atoi(v));
        } else if (a == "--no-bump") {
            bump = false;
        } else if (a == "-h" || a == "--help") {
            std::cout << "用法: semantic_label_merge --base <tiles.txt>"
                         " --overlay <labels.jsonl> --out <merged.txt>"
                         " [--zoom 18] [--expected-version N] [--no-bump]\n";
            return 0;
        } else {
            std::cerr << "未知参数: " << a << "\n";
            return 2;
        }
    }

    if (base_path.empty() || overlay_path.empty() || out_path.empty()) {
        std::cerr << "必须指定 --base / --overlay / --out\n";
        return 2;
    }

    static SemanticTileStore<kStoreCapacity> store;
    std::uint32_t base_zoom = 0;
    std::uint32_t base_version = 0;
    const std::int64_t n_base =
        load_semantic_tiles(store, base_zoom, base_path, &base_version);
    if (n_base < 0) {
        std::cerr << "读取 base 失败: " << base_path << "\n";
        return 1;
    }
    if (base_version == 0) {
        std::cerr << "base 无 map_version（v1 格式）：回写必须能校验版本，请先升级到 v2\n";
        return 1;
    }
    if (expected_version != 0 && base_version != expected_version) {
        std::cerr << "base 版本不匹配: 期望 " << expected_version
                  << "，实际 " << base_version << "\n";
        return 1;
    }
    if (zoom == 0) zoom = base_zoom;
    if (zoom == 0) {
        std::cerr << "zoom 无法确定（base 未标注且未给 --zoom）\n";
        return 1;
    }
    if (zoom != base_zoom) {
        std::cerr << "zoom 与 base 不一致: base=" << base_zoom
                  << " 指定=" << zoom << "\n";
        return 1;
    }

    // 逐行读 overlay
    std::FILE* f = std::fopen(overlay_path.c_str(), "rb");
    if (f == nullptr) {
        std::cerr << "打开 overlay 失败: " << overlay_path << "\n";
        return 1;
    }
    OverlayStats st;
    char line[kLineMax];
    std::int64_t n_overlay = 0;
    while (std::fgets(line, sizeof(line), f) != nullptr) {
        if (line[0] == '\n' || line[0] == '#') continue;
        ++n_overlay;
        (void)apply_overlay_entry(line, store, base_version, zoom, st);
    }
    std::fclose(f);

    const std::uint32_t out_version = bump ? (base_version + 1U) : base_version;
    const std::int64_t n_out = save_semantic_tiles(store, zoom, out_path, out_version);
    if (n_out < 0) {
        std::cerr << "写入输出失败: " << out_path << "\n";
        return 1;
    }

    std::cout << "base=" << n_base << " 个瓦片 (version=" << base_version
              << ", zoom=" << zoom << ")\n"
              << "overlay=" << n_overlay << " 行，应用 " << st.applied << " 条\n";
    if (st.skipped_version > 0) {
        std::cout << "  跳过(版本不匹配) " << st.skipped_version << " 条\n";
    }
    if (st.skipped_bad > 0) {
        std::cout << "  跳过(字段非法/类别未知) " << st.skipped_bad << " 条\n";
    }
    if (st.skipped_capacity > 0) {
        std::cout << "  跳过(标签容量已满) " << st.skipped_capacity << " 条\n";
    }
    std::cout << "输出 " << n_out << " 个瓦片 → " << out_path
              << " (version=" << out_version
              << (bump ? "，已递增" : "，未递增") << ")\n";

    if (st.applied == 0 && n_overlay > 0) {
        std::cerr << "警告: overlay 非空但一条都没应用，检查 map_version 与字段\n";
        return 1;
    }
    return 0;
}
