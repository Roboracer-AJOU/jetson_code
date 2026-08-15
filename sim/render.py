"""맵 + 라인 + 주행궤적 PNG 렌더. 시뮬 결과 눈으로 확인용."""
import math
import sys

sys.path.insert(0, "/home/nvidia/f1tenth_ajou/sim")
import numpy as np
from PIL import Image, ImageDraw
from race_sim import MAP_YAML as MAP
from race_sim import GridMap


def render(trace=None, obstacles=None, out="/tmp/sim_view.png", scale=6, line="raceline"):  # noqa: E501
    m = GridMap(MAP)
    base = np.zeros((m.h, m.w, 3), dtype=np.uint8)
    base[~m.free & ~m.wall] = (60, 60, 60)
    base[m.free] = (235, 235, 235)
    base[m.wall] = (20, 20, 20)
    img = Image.fromarray(np.flipud(base)).resize(
        (m.w * scale, m.h * scale), Image.NEAREST
    )
    d = ImageDraw.Draw(img)

    def px(x, y):
        cx = (x - m.ox) / m.res * scale
        cy = (m.h - (y - m.oy) / m.res) * scale
        return cx, cy

    p = f"/home/nvidia/f1tenth_ajou/src/path_following/config/{line}.csv"
    pts = []
    with open(p) as f:
        for ln in f:
            ln = ln.strip()
            if not ln or ln[0] in "#xX":
                continue
            c = ln.split(",")
            pts.append((float(c[0]), float(c[1])))
    d.line([px(a, b) for a, b in pts] + [px(*pts[0])], fill=(140, 60, 220), width=2)

    for ob in obstacles or []:
        cx, cy = px(ob.x, ob.y)
        r = ob.r / m.res * scale
        col = (255, 140, 0) if (ob.vx or ob.vy) else (220, 30, 30)
        d.ellipse([cx - r, cy - r, cx + r, cy + r], fill=col)

    if trace:
        for i in range(1, len(trace)):
            x0, y0 = trace[i - 1][0], trace[i - 1][1]
            x1, y1 = trace[i][0], trace[i][1]
            aeb = trace[i][6] if len(trace[i]) > 6 else 0
            col = (255, 0, 0) if aeb else (0, 150, 255)
            d.line([px(x0, y0), px(x1, y1)], fill=col, width=3)
        sx, sy = px(trace[0][0], trace[0][1])
        d.ellipse([sx - 8, sy - 8, sx + 8, sy + 8], outline=(0, 200, 0), width=3)
        ex, ey = px(trace[-1][0], trace[-1][1])
        d.ellipse([ex - 8, ey - 8, ex + 8, ey + 8], outline=(255, 0, 255), width=3)

    img.save(out)
    return out


if __name__ == "__main__":
    m = GridMap(MAP)
    print("wall check:")
    for x, y in [(-6.48, 1.32), (-6.84, 1.33), (-7.85, 1.54), (-6.0, 1.3), (-5.5, 1.3)]:
        print(f"  ({x:6.2f},{y:5.2f}) wall={m.is_wall(x,y)} clear={m.wall_clearance(x,y):.2f}")
    print(render(out="/tmp/map_view.png"))
