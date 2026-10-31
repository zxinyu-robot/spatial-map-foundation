# Spatial Map Foundation

[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)
[![C++20](https://img.shields.io/badge/C%2B%2B-20-00599C.svg)](https://isocpp.org/)

**哈希瓦片地图底座 + 空间状态运行时**。把局部 SLAM 的增量状态，通过哈希瓦片编码嵌入到城市级地图坐标，并以版本化状态事务管理地图状态。

> A hash-tiled map foundation and spatial-state runtime. Local SLAM incremental states are embedded into city-scale map coordinates via hash tile encoding, and managed as versioned state transactions.

## 这条链路在解决什么

```text
Super-LIO 增量位姿（局部 ENU x/y/z + 四元数）
      │ 地理锚定：地图原点经纬度 + 航向 → WGS84
      ▼
WGS84 → GCJ-02 → 高德 / XYZ 瓦片 z/x/y
      ▼
哈希瓦片登记（幂等，重复拒绝）        ← src/map
      │ 新瓦片 → SubmapPose 状态
      ▼
空间状态事务（生命周期 + 原子提交）   ← src/runtime
      │ optimize()
      ▼
版本化状态快照
      │ （规划中）
      ├── 高德地图 API 底图（瓦片 ID 对齐，已可拉取）
      ├── 高斯泼溅（Gaussian Splatting）渲染
      └── 元宇宙持久空间地图
```

核心思想：**地图是导航的度量底座**。三空间（度量 / 符号 / 世界）之下，先要有一个可哈希、可版本、可查询的空间坐标层。

## 已实现

- **哈希瓦片编码**（`src/map/tile_math.h`）：经纬度 ↔ 高德 / XYZ / Slippy Map 瓦片 `z/x/y` 互转，Web Mercator 投影，瓦片编号与主流地图服务一致。
- **哈希瓦片存储**（`src/map/hash_tile_store.h`）：固定容量开放寻址哈希表，无动态分配；瓦片幂等登记、重复拒绝、墓碑删除，查询 O(1) 均摊。
- **地理锚定**（`src/map/geo_anchor.h`）：LIO 局部 ENU 位姿 + 地图原点经纬度/航向 → WGS84，ENU 平面近似。
- **高德底图对齐**（`src/map/amap_tile.h`）：WGS84 ↔ GCJ-02 坐标转换（境外不偏移、往返误差 < 0.01m）+ 高德瓦片 URL 构造（子域名轮询）。
- **空间状态运行时**（`src/runtime/`，自 fork-spatial-runtime 迁入）：状态生命周期 `Requested → Initialized → Active / Fixed → Marginalized → Archived`，因子批次校验、原子事务、固定延迟窗口。
- **语义瓦片层**（`src/map/semantic_tile.h` + `semantic_persistence.h`）：在几何瓦片之上挂语义标签（厨房/门/走廊/开放目标），固定容量、无动态分配；语义 SLAM 输出经地理锚定投影到瓦片后挂标签，序列化为文本（v2 格式带 `map_version`，兼容读取 v1）供上游消费与真机对接。
- **高斯泼溅接口**（`src/render/gaussian_splat.h`）：3D Gaussian Splatting 数据模型 + 标准 3DGS PLY 导出（DC-only，可被 gsplat / SuperSplat 读取）。
- **瓦片版本链持久化**（`src/map/tile_persistence.h`）：瓦片集合 + `graph_version` 序列化为可恢复文本，幂等重载——元宇宙持久空间地图的索引雏形。
- **组合演示**（`demo/map_state_demo.cpp`）：Super-LIO 风格位姿 → 地理锚定 → 哈希瓦片 → 状态事务 → 版本化快照。
- **拉瓦片工具**（`tools/`）：`amap_tile_tool` 输出 GCJ-02 坐标与瓦片 URL；`fetch_amap_tile.sh` 一键拉取底图验证对齐。
- **splat 导出工具**（`tools/export_splats.cpp`）：瓦片 → 代理高斯泼溅 → 3DGS PLY。
- **语义瓦片查询服务**（`src/runtime/semantic_service.h` + `tools/semantic_tile_service.cpp`）：TCP 行分隔 JSON 的只读查询服务，把语义地图暴露给边侧 / 端侧消费。支持 `info` / `tiles` / `query`（半径 + 类别 + 置信度过滤，按距离升序）。**查询点用 WGS84（机器人位姿空间），服务端内部换算到瓦片的 GCJ-02 空间** —— 混用会产生约 500m 系统性偏移。每次响应必带 `map_version` 供消费方校验。
- **端侧采集包 ScenePack v1**（`src/io/scenepack.h`）：真机数据落盘与跨机交换容器。控制面 `manifest`（坐标系锚点 + `map_version` + `base_hash`）、索引面 `poses.bin`（40 字节定长关键帧）、`edges.bin`（相对位姿 + sqrtinfo 上三角）、语义瓦片、数据面 chunk 计数；FNV-1a 64 逐文件校验。定长记录、有界缓冲、返回码而非异常。

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

把已登记瓦片导出为高斯泼溅 PLY：

```bash
./build/export_splats 30.2740 120.1550 18 splats.ply
# 导出 3 个 splat → splats.ply
```

拼接 3×3 高德底图，叠加已登记 splat 瓦片可视化：

```bash
python3 tools/visualize_tiles.py 30.2740 120.1550 18 docs/assets/map_3x3.png
```

## Demo 展示

### ① 高德底图 + splat 瓦片叠加

![3×3 高德底图 + splat 瓦片叠加](docs/assets/map_3x3.png)

> 蓝框 = 中心瓦片；红框 + 红点 = 已登记 splat 瓦片（对应 Super-LIO 轨迹跨过的 3 个瓦片）。底图为杭州武林广场真实高德瓦片。

### ② 终端全链路回放

`./build/map_state_demo` 的实际输出：

```text
== Super-LIO 增量位姿 → 哈希瓦片 → 空间状态事务 ==
  [+ok] 局部(0,0) → wgs(30.274,120.155) → z18 x218569 y107925  graph_version=1
  [+ok] 局部(0,500) → wgs(30.2785,120.155) → z18 x218569 y107921  graph_version=2
  [+ok] 局部(700,0) → wgs(30.274,120.162) → z18 x218574 y107925  graph_version=3
  [dup] 局部(0,0) → z18 x218569 y107925 已存在，拒绝重复

== 状态快照 ==
  graph_version=3  active_states=3  backend_operations=9  tiles_registered=3  duplicates_rejected=1
```

### ③ 高斯泼溅 PLY（拖进 SuperSplat 旋转查看）

`splats.ply`（3 个代理 splat）已导出到 [docs/assets/splats.ply](docs/assets/splats.ply)，可拖入 https://superspl.at/editor 交互查看。

```text
$ ./build/export_splats 30.2740 120.1550 18 splats.ply
导出 3 个 splat → splats.ply
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
src/io/         端侧采集包 ScenePack（manifest + 位姿图 + 校验）
src/runtime/    空间状态运行时（fork-spatial 核心）+ 语义瓦片查询服务
demo/           组合演示
tools/          高德瓦片工具（坐标转换 + 拉取）
tests/          无框架单元测试
docs/           架构说明与路线图
```

## 语义地图链路（对接真机）

```text
语义 SLAM 输出（局部位姿 + 类别 + 置信度）
  → 地理锚定 → 语义瓦片（SemanticTile：瓦片 + 标签集合）
  → 序列化（semantic-tile 2 <zoom> <map_version>）→ 供上游（边侧世界模型 / 端侧策略）消费
  → 可选：与位姿图一起打进 ScenePack，跨机交换
```

运行语义 demo：

```bash
./build/semantic_map_demo
```

### 语义瓦片查询服务

把语义地图暴露给边侧（Python）消费 —— 这是「老师 VLM 理解 → 语义瓦片 → 端侧策略」链路的通道。

```bash
./build/semantic_tile_service --file /tmp/semantic_tiles.txt --port 9881 \
  --anchor-lat 30.2741 --anchor-lon 120.1551
```

```bash
# info：版本 / 缩放级 / 瓦片数 / 锚点
printf '{"cmd":"info"}\n' | nc 127.0.0.1 9881

# 半径查询（lat/lon 为 WGS84；可加 categories / min_confidence）
printf '{"cmd":"query","lat":30.2741,"lon":120.1551,"radius_m":300,"categories":["kitchen"]}\n' \
  | nc 127.0.0.1 9881
# → {"ok":true,"map_version":7,"count":1,"truncated":false,
#    "tiles":[{"z":18,"x":218569,"y":107925,"dist_m":37.68,
#              "tags":[{"cat":"kitchen","conf":90}]}]}
```

**坐标系铁律**：瓦片索引空间是 GCJ-02（高德），机器人位姿是 WGS84。查询点必须给 WGS84，服务端负责换算。混用会让同一瓦片的距离从 38m 变成 513m，症状是「语义瓦片莫名查不到」而非报错 —— 测试已显式锁住这个坑。

### ScenePack：真机数据打包与校验

真机位姿直接写成文本（每行一个关键帧），工具转成定长二进制包：

```text
# poses.txt —— Super-LIO / 里程计直接吐即可
<id> <stamp_ns> <x> <y> <z> <qx> <qy> <qz> <qw>
```

```bash
# 打包（edges.txt / semantic_tiles.txt 可选）
./build/scenepack_tool pack out/ 30.2741 120.1551 0.785398 7 poses.txt edges.txt semantic_tiles.txt

# 校验：先比对 FNV-1a 校验和，再比对 map_version
./build/scenepack_tool verify out/ 7
# → verify out/: ok
```

校验失败返回非零并给出具体原因（`checksum_mismatch` / `version_mismatch` / `bad_magic` / `capacity_exceeded`），不静默通过。

## 规划中

- 真实点云 / 关键帧 → splat 生成（当前为每瓦片代理 splat，接真实渲染引擎时替换）。
- 瓦片版本链的增量同步与共享（当前为单机文本持久化，跨机共享待接 DDS / 对象存储）。
- 语义标签与 edge-semantic-gateway 的 frontier 语义对齐（师生蒸馏链路的第一版）。
- ScenePack 增量同步与内容寻址续传（当前为单机整包）。
- 高斯分块序列化（量化列存 + 空间分块）；当前 `export_splats` 仍是每瓦片代理 splat，无真实高斯场可序列化。

## License

[MIT](LICENSE)
