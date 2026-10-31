// 高斯泼溅导出测试：PLY 头与顶点数正确、可被简单解析。
#include <cassert>
#include <cstdio>
#include <iostream>
#include <string>
#include <vector>

#include "render/gaussian_splat.h"

namespace {

void test_write_splat_ply() {
    using namespace spatial_map;

    std::vector<GaussianSplat> splats{
        {1.0, 2.0, 3.0, 1.0f, 0.0f, 0.0f, 0.9f, 0.1f, 0.2f, 0.3f, 0.0f, 0.0f, 0.0f, 1.0f},
        {4.0, 5.0, 6.0, 0.0f, 1.0f, 0.0f, 0.8f, 0.4f, 0.5f, 0.6f, 0.0f, 0.0f, 0.0f, 1.0f},
    };

    const std::string path = "/tmp/test_splats.ply";
    const std::int64_t n = write_splat_ply(splats, path);
    assert(n == 2);

    // 解析验证：header 含 element vertex 2 与标准属性，数据行数=2
    std::FILE* f = std::fopen(path.c_str(), "rb");
    assert(f != nullptr);
    char line[256];
    bool has_vertex = false;
    bool has_opacity = false;
    bool has_scale = false;
    bool has_rot = false;
    int data_lines = 0;
    bool header_done = false;
    while (std::fgets(line, sizeof(line), f) != nullptr) {
        std::string s(line);
        if (!header_done) {
            if (s.rfind("element vertex 2", 0) == 0) has_vertex = true;
            if (s.rfind("property float opacity", 0) == 0) has_opacity = true;
            if (s.rfind("property float scale_0", 0) == 0) has_scale = true;
            if (s.rfind("property float rot_3", 0) == 0) has_rot = true;
            if (s.rfind("end_header", 0) == 0) header_done = true;
        } else {
            ++data_lines;
        }
    }
    std::fclose(f);

    assert(has_vertex);
    assert(has_opacity);
    assert(has_scale);
    assert(has_rot);
    assert(data_lines == 2);

    std::cout << "  [ok] splat ply export\n";
}

}  // namespace

int main() {
    std::cout << "== splat tests ==\n";
    test_write_splat_ply();
    std::cout << "== all passed ==\n";
    return 0;
}
