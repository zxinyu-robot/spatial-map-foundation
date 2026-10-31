#!/usr/bin/env python3
"""语义地图可视化：把语义瓦片叠加到高德底图上。

这是「供上游消费」的可见证据 —— 语义 SLAM / VLM 标注写进地图后，
人眼能直接核对「标在哪、标成什么」。

用法:
    python3 tools/visualize_semantic_map.py <tiles.txt> [out.png]
        [--zoom 18] [--margin 1] [--no-basemap]

    tiles.txt 为 semantic-tile 格式（v1 或 v2）。
    底图从高德拉取；网络不可用时自动退化为深色网格画布（仍能看清语义布局）。

设计要点
--------
**不做坐标换算**：语义瓦片文件里存的 x/y 已经是 GCJ-02 瓦片索引，
与高德底图同一空间。本工具直接按 (x, y) 贴图，**不再做 WGS84↔GCJ-02
转换** —— 多一次转换就多一次 500m 偏移的机会。
"""
from __future__ import annotations

import math
import sys
import urllib.request
from io import BytesIO
from pathlib import Path

try:
    from PIL import Image, ImageDraw, ImageFont
except ImportError:  # pragma: no cover
    print("需要 Pillow：pip install pillow", file=sys.stderr)
    raise SystemExit(1)

TILE = 256

# 类别 → 颜色（RGBA）。同色系内按亮度区分，避免相近类别混淆。
CATEGORY_COLORS: dict[str, tuple[int, int, int]] = {
    "corridor":    (70, 110, 160),
    "door":        (230, 190, 60),
    "kitchen":     (240, 130, 50),
    "bedroom":     (150, 100, 200),
    "living_room": (60, 170, 110),
    "bathroom":    (60, 180, 200),
    "staircase":   (150, 110, 70),
    "elevator":    (220, 70, 70),
    "obstacle":    (140, 40, 40),
    "free_space":  (120, 190, 130),
    "goal_object": (220, 60, 170),
    "unknown":     (120, 120, 120),
}


# ---------- 解析 semantic-tile 文件 ----------

def parse_semantic_tiles(path: Path) -> tuple[int, int, list[tuple[int, int, list[tuple[str, int]]]]]:
    """返回 (zoom, map_version, [(x, y, [(category, conf), ...]), ...]).

    兼容 v1（semantic-tile 1 <zoom>）与 v2（semantic-tile 2 <zoom> <ver>）。
    v1 的 map_version 记为 0。
    """
    zoom = 0
    version = 0
    tiles: list[tuple[int, int, list[tuple[str, int]]]] = []

    for raw in path.read_text(encoding="utf-8").splitlines():
        line = raw.strip()
        if not line or line.startswith("#"):
            continue

        if line.startswith("semantic-tile"):
            parts = line.split()
            if len(parts) >= 3:
                zoom = int(parts[2])
            if len(parts) >= 4:
                version = int(parts[3])
            continue

        if not line.startswith("tile"):
            continue
        parts = line.split()
        if len(parts) < 3:
            continue
        x, y = int(parts[1]), int(parts[2])
        tags: list[tuple[str, int]] = []
        rest = parts[3:]
        for i in range(0, len(rest) - 1, 2):
            tags.append((rest[i], int(rest[i + 1])))
        tiles.append((x, y, tags))

    return zoom, version, tiles


# ---------- 底图 ----------

def fetch_amap_tile(z: int, x: int, y: int, timeout: float = 15.0) -> Image.Image | None:
    url = (f"https://webrd01.is.autonavi.com/appmaptile"
           f"?lang=zh_cn&size=1&scale=1&style=8&x={x}&y={y}&z={z}")
    try:
        with urllib.request.urlopen(url, timeout=timeout) as r:
            im = Image.open(BytesIO(r.read()))
            im.load()
            # 高德瓦片是带 tRNS 的 P 模式；先补白底再转 RGB，
            # 否则 Pillow 会报 "Palette images with Transparency"。
            if im.mode in ("P", "PA") or "transparency" in im.info:
                im = im.convert("RGBA")
                bg = Image.new("RGBA", im.size, (255, 255, 255, 255))
                im = Image.alpha_composite(bg, im)
            return im.convert("RGB")
    except Exception:
        return None


def grid_canvas(w: int, h: int) -> Image.Image:
    """网络不可用时的退化画布：深色底 + 网格，仍能看清语义布局."""
    im = Image.new("RGB", (w, h), (24, 28, 34))
    draw = ImageDraw.Draw(im)
    for gx in range(0, w, TILE):
        draw.line([(gx, 0), (gx, h)], fill=(48, 56, 66))
    for gy in range(0, h, TILE):
        draw.line([(0, gy), (w, gy)], fill=(48, 56, 66))
    return im


# ---------- 渲染 ----------

def _font(size: int):
    for name in ("/System/Library/Fonts/Supplemental/Arial Unicode.ttf",
                 "/System/Library/Fonts/PingFang.ttc",
                 "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"):
        try:
            return ImageFont.truetype(name, size)
        except OSError:
            continue
    return ImageFont.load_default()


def render(
    tiles: list[tuple[int, int, list[tuple[str, int]]]],
    zoom: int,
    map_version: int,
    out_path: Path,
    *,
    margin: int = 1,
    use_basemap: bool = True,
) -> dict:
    if not tiles:
        raise SystemExit("语义瓦片为空，无内容可渲染")

    xs = [t[0] for t in tiles]
    ys = [t[1] for t in tiles]
    x0, x1 = min(xs) - margin, max(xs) + margin
    y0, y1 = min(ys) - margin, max(ys) + margin
    cols, rows = x1 - x0 + 1, y1 - y0 + 1

    # 内边距：不加的话最外圈瓦片的边框会被画布边缘裁掉一半
    pad = 8
    w, h = cols * TILE + pad * 2, rows * TILE + pad * 2

    # 底图（失败则退化）
    basemap_ok = False
    if use_basemap:
        canvas = Image.new("RGB", (w, h), (24, 28, 34))
        fetched = 0
        for ty in range(y0, y1 + 1):
            for tx in range(x0, x1 + 1):
                t = fetch_amap_tile(zoom, tx, ty)
                if t is not None:
                    canvas.paste(t, (pad + (tx - x0) * TILE, pad + (ty - y0) * TILE))
                    fetched += 1
        basemap_ok = fetched == cols * rows
        if fetched == 0:
            canvas = grid_canvas(w, h)
        elif not basemap_ok:
            # 部分成功：没拉到的格子补网格纹理，避免出现黑洞
            grid = grid_canvas(w, h)
            mask = Image.new("L", (w, h), 0)
            mask_draw = ImageDraw.Draw(mask)
            for ty in range(y0, y1 + 1):
                for tx in range(x0, x1 + 1):
                    px, py = pad + (tx - x0) * TILE, pad + (ty - y0) * TILE
                    if canvas.crop([px, py, px + TILE, py + TILE]).getextrema() == ((0, 0), (0, 0), (0, 0)):
                        mask_draw.rectangle([px, py, px + TILE - 1, py + TILE - 1], fill=255)
            canvas.paste(grid, (0, 0), mask)
    else:
        canvas = grid_canvas(w, h)

    # 语义叠加（半透明填充 + 边框 + 标签）
    overlay = Image.new("RGBA", (w, h), (0, 0, 0, 0))
    od = ImageDraw.Draw(overlay)
    font_small = _font(15)
    font_label = _font(16)

    cat_counts: dict[str, int] = {}
    for tx, ty, tags in tiles:
        px, py = pad + (tx - x0) * TILE, pad + (ty - y0) * TILE
        if not tags:
            color = CATEGORY_COLORS["unknown"]
            od.rectangle([px + 6, py + 6, px + TILE - 7, py + TILE - 7],
                         outline=(*color, 160), width=2)
            continue

        # 以最高置信度类别决定填充色
        primary = max(tags, key=lambda kv: kv[1])
        color = CATEGORY_COLORS.get(primary[0], CATEGORY_COLORS["unknown"])
        od.rectangle([px + 6, py + 6, px + TILE - 7, py + TILE - 7],
                     fill=(*color, 165), outline=(*color, 245), width=5)

        for cat, _conf in tags:
            cat_counts[cat] = cat_counts.get(cat, 0) + 1

        # 标签：类别 + 置信度，最多两行。底图颜色不可控，故加深色底板。
        lines = [f"{c} {cf}" for c, cf in sorted(tags, key=lambda kv: -kv[1])[:2]]
        ty_text = py + 12
        for ln in lines:
            bbox = od.textbbox((0, 0), ln, font=font_label)
            tw, th = bbox[2] - bbox[0], bbox[3] - bbox[1]
            tx_text = px + (TILE - tw) // 2
            od.rounded_rectangle(
                [tx_text - 6, ty_text - 3, tx_text + tw + 6, ty_text + th + 5],
                radius=5, fill=(12, 14, 18, 205), outline=(*color, 210), width=2,
            )
            od.text((tx_text, ty_text - bbox[1]), ln, font=font_label,
                    fill=(255, 255, 255, 255))
            ty_text += th + 12

    canvas = Image.alpha_composite(canvas.convert("RGBA"), overlay).convert("RGB")

    # 图例与标题。列数按画布宽度自适应 —— 固定 4 列在窄画布上会被裁切。
    font_small = _font(15)
    col_w = 190
    n_cols = max(1, min(6, (w - 28) // col_w))
    legend_h = 68 + 24 * ((len(cat_counts) + n_cols - 1) // n_cols)
    titled = Image.new("RGB", (w, h + legend_h), (18, 20, 24))
    titled.paste(canvas, (0, 0))
    td = ImageDraw.Draw(titled)

    title_font = _font(20)
    ver_note = f"v{map_version}" if map_version else "v1(无版本)"
    if not use_basemap:
        base_note = "无底图(网格)"
    elif basemap_ok:
        # 高 zoom 下高德可能只有建筑底纹、无街道信息，如实标注避免误读
        base_note = "高德底图" if zoom <= 18 else "高德底图(z≥19 多为建筑底纹)"
    else:
        base_note = "部分底图/补网格"
    title = (f"语义地图 {ver_note} · zoom {zoom} · {len(tiles)} 个语义瓦片 · "
             f"{base_note}")
    td.text((14, h + 12), title, font=title_font, fill=(235, 240, 245))

    # 类别色块
    lx, ly = 14, h + 48
    for i, cat in enumerate(sorted(cat_counts)):
        color = CATEGORY_COLORS.get(cat, CATEGORY_COLORS["unknown"])
        col = i % n_cols
        row = i // n_cols
        bx = lx + col * col_w
        by = ly + row * 24
        td.rectangle([bx, by + 3, bx + 15, by + 17], fill=color)
        td.text((bx + 22, by), f"{cat} ×{cat_counts[cat]}",
                font=font_small, fill=(210, 216, 224))

    out_path.parent.mkdir(parents=True, exist_ok=True)
    titled.save(out_path)

    return {
        "out": str(out_path),
        "size": titled.size,
        "tiles": len(tiles),
        "zoom": zoom,
        "map_version": map_version,
        "basemap": basemap_ok,
        "categories": cat_counts,
        "bounds": {"x": [x0, x1], "y": [y0, y1]},
    }


def main(argv: list[str]) -> int:
    if len(argv) < 2:
        print(__doc__)
        return 2

    src = Path(argv[1])
    out = Path(argv[2]) if len(argv) > 2 and not argv[2].startswith("--") else Path("semantic_map.png")

    zoom_override = 0
    margin = 1
    use_basemap = True
    for i, a in enumerate(argv):
        if a == "--zoom" and i + 1 < len(argv):
            zoom_override = int(argv[i + 1])
        elif a == "--margin" and i + 1 < len(argv):
            margin = int(argv[i + 1])
        elif a == "--no-basemap":
            use_basemap = False

    if not src.exists():
        print(f"找不到文件: {src}", file=sys.stderr)
        return 1

    zoom, version, tiles = parse_semantic_tiles(src)
    if zoom_override:
        zoom = zoom_override
    if zoom == 0:
        print("无法确定 zoom（文件未标注且未给 --zoom）", file=sys.stderr)
        return 1

    try:
        stats = render(tiles, zoom, version, out, margin=margin, use_basemap=use_basemap)
    except SystemExit as exc:
        print(str(exc), file=sys.stderr)
        return 1

    print(f"已保存: {stats['out']} ({stats['size'][0]}x{stats['size'][1]})")
    print(f"  瓦片 {stats['tiles']} 个，zoom {stats['zoom']}，map_version {stats['map_version']}")
    print(f"  底图: {'高德' if stats['basemap'] else '网格/部分'}")
    print(f"  类别分布: {stats['categories']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
