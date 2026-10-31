#!/usr/bin/env bash
# 拉取高德底图瓦片，验证 WGS84 位姿与高德底图的对齐。
# 用法：./tools/fetch_amap_tile.sh <lat> <lon> <zoom> [输出png路径]
set -euo pipefail

LAT="${1:?缺少纬度}"
LON="${2:?缺少经度}"
Z="${3:?缺少缩放级别}"
OUT="${4:-amap_tile.png}"

# 编译工具（若未编译）
BIN="build/amap_tile_tool"
if [[ ! -x "$BIN" ]]; then
  mkdir -p build
  clang++ -std=c++20 -I src tools/amap_tile_tool.cpp -o "$BIN"
fi

read -r WGS GCJ TILE URL < <("$BIN" "$LAT" "$LON" "$Z" \
  | awk -F= '/^wgs84=/{w=$2} /^gcj02=/{g=$2} /^tile=/{t=$2} /^url=/{sub(/^url=/,""); u=$0} END{print w, g, t, u}')

echo "wgs84=$WGS"
echo "gcj02=$GCJ"
echo "tile=$TILE"
echo "url=$URL"

curl -sS --fail --max-time 20 -o "$OUT" "$URL"
echo "saved: $OUT ($(file -b "$OUT" | cut -d, -f1))"
