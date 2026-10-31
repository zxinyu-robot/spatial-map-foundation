// 高斯泼溅渲染数据接口。
//
// 瓦片内点云 / 关键帧最终以 3D Gaussian Splatting 形式渲染。
// 本层定义 splat 数据模型，并导出 3DGS 标准 PLY（DC-only 子集），
// 可被 gsplat、SuperSplat、SIBR、3DGS 官方查看器等直接读取。
//
// 坐标系约定：splat 位置用 ENU 米（东/北/天），与地图锚定一致；
// 后续接真实渲染引擎时，再把 ENU 转到渲染相机系。
#pragma once

#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

namespace spatial_map {

// 单个 3D 高斯。
struct GaussianSplat {
    double x{0.0}, y{0.0}, z{0.0};   // 均值位置（ENU 米）
    float r{1.0f}, g{1.0f}, b{1.0f}; // 零阶球谐颜色（DC）
    float opacity{0.9f};             // 不透明度 (0..1)
    float sx{1.0f}, sy{1.0f}, sz{1.0f};  // 三轴尺度（标准差，米）
    float qw{1.0f}, qx{0.0f}, qy{0.0f}, qz{0.0f};  // 旋转四元数
};

// 把 splat 集合写成 3DGS PLY（DC-only，不含高阶 SH）。
// 返回写入的顶点数；失败返回 -1。
[[nodiscard]] inline std::int64_t write_splat_ply(
    const std::vector<GaussianSplat>& splats, const std::string& path) noexcept {
    std::FILE* f = std::fopen(path.c_str(), "wb");
    if (f == nullptr) return -1;

    const char header_fmt[] =
        "ply\n"
        "format ascii 1.0\n"
        "element vertex %zu\n"
        "property float x\n"
        "property float y\n"
        "property float z\n"
        "property float nx\n"
        "property float ny\n"
        "property float nz\n"
        "property float f_dc_0\n"
        "property float f_dc_1\n"
        "property float f_dc_2\n"
        "property float opacity\n"
        "property float scale_0\n"
        "property float scale_1\n"
        "property float scale_2\n"
        "property float rot_0\n"
        "property float rot_1\n"
        "property float rot_2\n"
        "property float rot_3\n"
        "end_header\n";

    if (std::fprintf(f, header_fmt, splats.size()) < 0) {
        std::fclose(f);
        return -1;
    }

    for (const GaussianSplat& s : splats) {
        // 法线在 3DGS 中不用，置 0
        if (std::fprintf(f,
                         "%.6f %.6f %.6f 0 0 0 "
                         "%.6f %.6f %.6f "
                         "%.6f "
                         "%.6f %.6f %.6f "
                         "%.6f %.6f %.6f %.6f\n",
                         s.x, s.y, s.z,
                         s.r, s.g, s.b,
                         s.opacity,
                         s.sx, s.sy, s.sz,
                         s.qx, s.qy, s.qz, s.qw) < 0) {
            std::fclose(f);
            return -1;
        }
    }

    std::fclose(f);
    return static_cast<std::int64_t>(splats.size());
}

}  // namespace spatial_map
