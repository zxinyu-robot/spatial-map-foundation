#!/usr/bin/env python3
"""语义地图可视化工具测试（解析与坐标处理）.

运行：python3 tests/test_visualize_semantic_map.py

只测纯逻辑（解析、类别色表覆盖）。渲染部分依赖 Pillow 与可选网络底图，
不做断言 —— 图像正确性靠 demo 产物人工核对。
"""
from __future__ import annotations

import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

sys.path.insert(0, str(ROOT / "tools"))
from visualize_semantic_map import (  # noqa: E402
    CATEGORY_COLORS,
    parse_semantic_tiles,
)

FAILURES: list[str] = []


def check(cond: bool, label: str) -> None:
    if not cond:
        FAILURES.append(label)
        print(f"  [FAIL] {label}")


def _write(text: str) -> Path:
    f = tempfile.NamedTemporaryFile("w", suffix=".txt", delete=False, encoding="utf-8")
    f.write(text)
    f.close()
    return Path(f.name)


def test_parse_v2() -> None:
    p = _write(
        "semantic-tile 2 20 8\n"
        "tile 874279 431701 kitchen 90 goal_object 80\n"
        "tile 874278 431700 corridor 92\n"
    )
    try:
        zoom, ver, tiles = parse_semantic_tiles(p)
        check(zoom == 20, "zoom parsed")
        check(ver == 8, "v2 map_version parsed")
        check(len(tiles) == 2, "two tiles")
        check(tiles[0][0] == 874279 and tiles[0][1] == 431701, "tile coords")
        check(tiles[0][2] == [("kitchen", 90), ("goal_object", 80)], "multi tags")
        check(tiles[1][2] == [("corridor", 92)], "single tag")
    finally:
        p.unlink()
    print("  [ok] parse v2")


def test_parse_v1_compat() -> None:
    """v1 文件（无语义版本）仍应可解析，map_version 记为 0."""
    p = _write("semantic-tile 1 18\ntile 100 200 door 80\n")
    try:
        zoom, ver, tiles = parse_semantic_tiles(p)
        check(zoom == 18, "v1 zoom parsed")
        check(ver == 0, "v1 has no map_version")
        check(len(tiles) == 1, "v1 tile parsed")
        check(tiles[0][2] == [("door", 80)], "v1 tag parsed")
    finally:
        p.unlink()
    print("  [ok] parse v1 compatible")


def test_parse_robustness() -> None:
    """空行 / 注释 / 畸形行不应让解析崩掉."""
    p = _write(
        "# 注释行\n"
        "\n"
        "semantic-tile 2 20 3\n"
        "\n"
        "tile\n"                    # 字段不足
        "garbage line here\n"       # 非 tile 行
        "tile 1 2\n"                # 无标签
        "tile 3 4 door 80\n"
    )
    try:
        zoom, ver, tiles = parse_semantic_tiles(p)
        check(zoom == 20, "zoom survived junk")
        check(ver == 3, "version survived junk")
        check(len(tiles) == 2, "only well-formed tiles kept")
        check(tiles[0][2] == [], "tagless tile kept with empty tags")
        check(tiles[1][2] == [("door", 80)], "valid tile parsed")
    finally:
        p.unlink()
    print("  [ok] parse robustness")


def _cpp_category_names() -> set[str]:
    """从 C++ 头文件读出 semantic_name() 实际返回的类别名.

    直接解析头文件而不是 import 另一个仓库的 Python 模块 ——
    测试不应产生跨仓库依赖，否则单独 clone 本仓库跑不通。
    """
    header = ROOT / "src" / "map" / "semantic_tile.h"
    text = header.read_text(encoding="utf-8")
    names: set[str] = set()
    in_fn = False
    for line in text.splitlines():
        if "semantic_name" in line and "inline const char*" in line:
            in_fn = True
            continue
        if in_fn:
            if line.strip().startswith("}"):
                break
            # case X: return "name";
            if 'return "' in line:
                seg = line.split('return "', 1)[1]
                names.add(seg.split('"', 1)[0])
    return names


def test_category_colors_cover_all() -> None:
    """C++ SemanticCategory 的每个名字都要有颜色，否则图上会退化成灰色."""
    names = _cpp_category_names()
    check(len(names) >= 10, f"parsed enough categories (got {len(names)})")
    for cat in sorted(names):
        if cat == "unknown":
            continue
        check(cat in CATEGORY_COLORS, f"color for {cat}")
    check("unknown" in CATEGORY_COLORS, "fallback color exists")
    print(f"  [ok] category colors cover all ({len(names)} C++ categories)")


def test_negative_coordinates() -> None:
    """瓦片坐标是 uint32，但解析不应因大数值溢出（Python int 无界，仅确认不崩）."""
    p = _write("semantic-tile 2 20 1\ntile 4294967295 4294967295 door 80\n")
    try:
        _zoom, _ver, tiles = parse_semantic_tiles(p)
        check(len(tiles) == 1, "large coords parsed")
        check(tiles[0][0] == 4294967295, "large x preserved")
    finally:
        p.unlink()
    print("  [ok] large coordinates")


def main() -> int:
    print("== visualize semantic map tests ==")
    test_parse_v2()
    test_parse_v1_compat()
    test_parse_robustness()
    test_category_colors_cover_all()
    test_negative_coordinates()

    if FAILURES:
        print(f"== {len(FAILURES)} failure(s) ==")
        for f in FAILURES:
            print(f"  - {f}")
        return 1
    print("== all passed ==")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
