# Spatial Map Foundation

[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)
[![C++20](https://img.shields.io/badge/C%2B%2B-20-00599C.svg)](https://isocpp.org/)

**哈希瓦片地图底座 + 空间状态运行时**。把局部 SLAM 的增量状态，通过哈希瓦片编码嵌入到城市级地图坐标，并以版本化状态事务管理地图状态。

> A hash-tiled map foundation and spatial-state runtime. Local SLAM incremental states are embedded into city-scale map coordinates via hash tile encoding, and managed as versioned state transactions.

## 这条链路在解决什么

```text
局部 SLAM 增量状态图
      │ 经纬度 → Web Mercator 瓦片 z/x/y
      ▼
哈希瓦片登记（幂等，重复拒绝）        ← src/map
      │ 新瓦片 → SubmapPose 状态
      ▼
空间状态事务（生命周期 + 原子提交）   ← src/runtime
      │ optimize()
      ▼
版本化状态快照
      │ （规划中）
      ├── 高德地图 API 数据（瓦片 ID 对齐）
      ├── 高斯泼溅（Gaussian Splatting）渲染
      └── 元宇宙持久空间地图
```

核心思想：**地图是导航的度量底座**。三空间（度量 / 符号 / 世界）之下，先要有一个可哈希、可版本、可查询的空间坐标层。

## 已实现

- **哈希瓦片编码**（`src/map/tile_math.h`）：经纬度 ↔ 高德 / XYZ / Slippy Map 瓦片 `z/x/y` 互转，Web Mercator 投影，瓦片编号与主流地图服务一致。
- **哈希瓦片存储**（`src/map/hash_tile_store.h`）：固定容量开放寻址哈希表，无动态分配；瓦片幂等登记、重复拒绝、墓碑删除，查询 O(1) 均摊。
- **高德底图对齐**（`src/map/amap_tile.h`）：WGS84 ↔ GCJ-02 坐标转换（境外不偏移、往返误差 < 0.01m）+ 高德瓦片 URL 构造（子域名轮询）。
- **空间状态运行时**（`src/runtime/`，自 fork-spatial-runtime 迁入）：状态生命周期 `Requested → Initialized → Active / Fixed → Marginalized → Archived`，因子批次校验、原子事务、固定延迟窗口。
- **组合演示**（`demo/map_state_demo.cpp`）：SLAM 位姿 → 哈希瓦片 → 状态事务 → 版本化快照。
- **拉瓦片工具**（`tools/`）：`amap_tile_tool` 输出 GCJ-02 坐标与瓦片 URL；`fetch_amap_tile.sh` 一键拉取底图验证对齐。

## 快速开始

要求：CMake 3.20+ 或支持 C++20 的编译器。

```bash
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
./build/map_state_demo
```

拉一张高德底图瓦片，验证位姿与底图对齐：

```bash
./tools/fetch_amap_tile.sh 30.2740 120.1550 18 amap_tile.png
# wgs84=30.274,120.155
# gcj02=30.27167,120.1597
# tile=z18/218569/107925
# saved: amap_tile.png (PNG image data)
```

无 CMake 时可直接编译：

```bash
clang++ -std=c++20 -I src src/runtime/*.cpp demo/map_state_demo.cpp -o map_state_demo
clang++ -std=c++20 -I src src/runtime/*.cpp tests/map_state_test.cpp -o map_state_test
```

演示输出：

```text
== 哈希瓦片嵌入 + 空间状态事务 ==
  [+ok] z18 x218566 y107923  瓦片NW=(30.2745,120.155)  graph_version=1
  [+ok] z18 x218566 y107920  瓦片NW=(30.278,120.155)  graph_version=2
  [+ok] z18 x218569 y107923  瓦片NW=(30.2745,120.159)  graph_version=3
  [dup] z18 x218566 y107923 已存在，拒绝重复登记

== 状态快照 ==
  graph_version=3  active_states=3  backend_operations=9  tiles_registered=3  duplicates_rejected=1
```

## 目录结构

```text
src/map/        哈希瓦片编码 / 存储 / 高德底图对齐（header-only）
src/runtime/    空间状态运行时（fork-spatial 核心）
demo/           组合演示
tools/          高德瓦片工具（坐标转换 + 拉取）
tests/          无框架单元测试
docs/           架构说明与路线图
```

## 规划中

- 高斯泼溅渲染接口：瓦片内点云/关键帧作为 splat 输入，向外提供渲染数据。
- 元宇宙持久空间地图：瓦片版本链作为持久化与共享的索引。

## License

[MIT](LICENSE)
