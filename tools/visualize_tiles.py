#!/usr/bin/env python3
"""拼接 3x3 高德底图瓦片，并把已登记的 splat 瓦片叠加可视化。

用法:
    python3 tools/visualize_tiles.py <lat> <lon> <zoom> [out.png]

    lat/lon 为 WGS84 中心坐标，zoom 为瓦片缩放级别。
    输出的 PNG 用红框标出已登记瓦片，红点标出 splat 中心。
"""
import math
import sys
import urllib.request
from io import BytesIO

from PIL import Image, ImageDraw

# ---------- 与 C++ 一致的瓦片计算 ----------

def out_of_china(lat, lon):
    return lon < 72.004 or lon > 137.8347 or lat < 0.8293 or lat > 55.8271

def _lat(x, y):
    ret = -100.0 + 2.0*x + 3.0*y + 0.2*y*y + 0.1*x*y + 0.2*math.sqrt(abs(x))
    ret += (20.0*math.sin(6.0*x*math.pi) + 20.0*math.sin(2.0*x*math.pi))*2.0/3.0
    ret += (20.0*math.sin(y*math.pi) + 40.0*math.sin(y/3.0*math.pi))*2.0/3.0
    ret += (160.0*math.sin(y/12.0*math.pi) + 320.0*math.sin(y*math.pi/30.0))*2.0/3.0
    return ret

def _lon(x, y):
    ret = 300.0 + x + 2.0*y + 0.1*x*x + 0.1*x*y + 0.1*math.sqrt(abs(x))
    ret += (20.0*math.sin(6.0*x*math.pi) + 20.0*math.sin(2.0*x*math.pi))*2.0/3.0
    ret += (20.0*math.sin(x*math.pi) + 40.0*math.sin(x/3.0*math.pi))*2.0/3.0
    ret += (150.0*math.sin(x/12.0*math.pi) + 300.0*math.sin(x/30.0*math.pi))*2.0/3.0
    return ret

def wgs84_to_gcj02(lat, lon):
    if out_of_china(lat, lon):
        return lat, lon
    a, ee = 6378245.0, 0.00669342162296594323
    dlat = _lat(lon - 105.0, lat - 35.0)
    dlon = _lon(lon - 105.0, lat - 35.0)
    radlat = lat / 180.0 * math.pi
    magic = math.sin(radlat)
    magic = 1 - ee * magic * magic
    sqrtmagic = math.sqrt(magic)
    dlat = (dlat * 180.0) / ((a * (1 - ee)) / (magic * sqrtmagic) * math.pi)
    dlon = (dlon * 180.0) / (a / sqrtmagic * math.cos(radlat) * math.pi)
    return lat + dlat, lon + dlon

def latlon_to_tile(lat, lon, z):
    n = 2 ** z
    xtile = (lon + 180.0) / 360.0 * n
    ytile = (1.0 - math.asinh(math.tan(math.radians(lat))) / math.pi) / 2.0 * n
    xtile = max(0.0, min(xtile, n - 1))
    ytile = max(0.0, min(ytile, n - 1))
    return int(xtile), int(ytile)

# ---------- 拉瓦片 ----------

def fetch_tile(z, x, y):
    url = (f"https://webrd01.is.autonavi.com/appmaptile"
           f"?lang=zh_cn&size=1&scale=1&style=8&x={x}&y={y}&z={z}")
    with urllib.request.urlopen(url, timeout=20) as r:
        im = Image.open(BytesIO(r.read()))
        im.load()
        return im.convert("RGB")

def main():
    if len(sys.argv) < 4:
        print(f"用法: {sys.argv[0]} <lat> <lon> <zoom> [out.png]")
        return 2
    lat, lon = float(sys.argv[1]), float(sys.argv[2])
    z = int(sys.argv[3])
    out = sys.argv[4] if len(sys.argv) > 4 else "map_3x3.png"

    # 中心瓦片（WGS84 → GCJ-02 → z/x/y）
    glat, glon = wgs84_to_gcj02(lat, lon)
    cx, cy = latlon_to_tile(glat, glon, z)
    print(f"wgs84=({lat},{lon}) → gcj02=({glat:.5f},{glon:.5f}) → 中心瓦片 z{z}/{cx}/{cy}")

    # 拉 3x3
    TILE = 256
    canvas = Image.new("RGB", (TILE * 3, TILE * 3))
    for dy in range(-1, 2):
        for dx in range(-1, 2):
            t = fetch_tile(z, cx + dx, cy + dy)
            canvas.paste(t, ((dx + 1) * TILE, (dy + 1) * TILE))
            print(f"  已拉取瓦片 x={cx+dx} y={cy+dy}")

    # 已登记瓦片（与 map_state_demo / export_splats 的位姿一致）
    # 局部 ENU 位姿 → 高德瓦片
    poses = [(0, 0), (0, 500), (700, 0)]
    meters_per_deg_lat = 111320.0
    registered = []
    for ex, ey in poses:
        # 局部 ENU → WGS84（平面近似，与 geo_anchor.h 一致）
        dlat = ey / meters_per_deg_lat
        dlon = ex / (meters_per_deg_lat * math.cos(math.radians(lat)))
        wlat, wlon = lat + dlat, lon + dlon
        tlat, tlon = wgs84_to_gcj02(wlat, wlon)
        tx, ty = latlon_to_tile(tlat, tlon, z)
        if (tx, ty) not in registered:
            registered.append((tx, ty))

    draw = ImageDraw.Draw(canvas, "RGBA")
    # 中心瓦片蓝色虚线框
    draw.rectangle([TILE, TILE, TILE * 2 - 1, TILE * 2 - 1], outline=(0, 120, 255, 255), width=3)
    # 已登记瓦片红色框 + 红点
    for tx, ty in registered:
        px = tx - (cx - 1)
        py = ty - (cy - 1)
        if 0 <= px < 3 and 0 <= py < 3:
            x0, y0 = px * TILE, py * TILE
            draw.rectangle([x0 + 3, y0 + 3, x0 + TILE - 4, y0 + TILE - 4],
                           outline=(255, 40, 40, 255), width=4)
            draw.ellipse([x0 + TILE/2 - 6, y0 + TILE/2 - 6,
                          x0 + TILE/2 + 6, y0 + TILE/2 + 6], fill=(255, 40, 40, 255))

    canvas.save(out)
    print(f"已保存: {out} ({canvas.size[0]}x{canvas.size[1]})")
    print(f"图例: 蓝框=中心瓦片，红框+红点=已登记 splat 瓦片 ({len(registered)} 个)")
    return 0

if __name__ == "__main__":
    sys.exit(main())
