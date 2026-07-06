# svg_path

Image-to-robot-path pipeline for the sketch prototype: turns a source image into a smooth SVG line drawing, then into an ordered list of physical waypoints for the robot arm to trace.

图像转机器人路径的处理流程：先把源图片变成平滑的 SVG 线稿，再转换成一组排好顺序的机器人物理坐标路径点。

Two scripts, run in sequence:

```
apple.png -> canny.py -> apple_edges.png, apple_edges.svg
                      \-> robot_path.py -> ordered waypoints (Python data, no file output yet)
```

## canny.py — 图像转 SVG 线稿

**运行 / Run:**

```bash
source .venv/bin/activate
python svg_path/canny.py
```

**做什么 / What it does:**

1. 读取 `apple.png`（灰度），跑 Canny 边缘检测。
   Reads `apple.png` (grayscale), runs Canny edge detection.
2. 用 `skimage.morphology.skeletonize` 把边缘细化成单像素宽的线，避免后续重复描边（`cv2.findContours` 会把一条线的两侧当成一个色块描边，产生镜像重复线，所以这里不用它）。
   Skeletonizes the edges to 1px-wide centerlines, so each stroke gets traced once instead of `cv2.findContours` walking both sides of the stroke as a mirrored duplicate.
3. 在骨架像素图上直接按图遍历（把端点/分叉点当节点），得到 `strokes`：每条独立线的像素点列表，以及是否闭合（比如苹果表皮上的小圆点瑕疵是闭合环）。
   Walks the skeleton pixel graph directly (endpoints/junctions as nodes) to produce `strokes`: one pixel-point list per independent line, plus whether it's a closed loop (e.g. the small blemish rings on the apple).
4. 用 `simplify()`（Douglas-Peucker）精简点数，再用 `catmull_rom_path()` 把折线拟合成平滑的三次贝塞尔曲线，写出 SVG。
   Simplifies point count (Douglas-Peucker) and fits a smooth cubic Bezier curve through the points (Catmull-Rom), then writes the SVG.

**输出 / Output** (both git-ignored, regenerated each run):

- `svg_path/apple_edges.png` — Canny 边缘位图 / raw Canny edge bitmap
- `svg_path/apple_edges.svg` — 平滑矢量线稿 / smoothed vector line art

**可调参数 / Tunable parameters** (edit directly in the script — no CLI flags yet):

| 参数 / Parameter | 位置 / Where | 作用 / Effect |
|---|---|---|
| `threshold1`, `threshold2` in `cv2.Canny(...)` | line 10 | 边缘检测的灵敏度。调低会检测出更多细节/噪点，调高则只保留强边缘。Lower = more detail/noise, higher = only strong edges. |
| `0.002` in `simplify()` (the `epsilon` multiplier) | line 70 | 折线简化程度。数值越大，点越少、线越直；数值越小，越贴近原始像素轨迹但点更多。Larger = fewer points/straighter lines; smaller = closer to raw pixels but more points. |
| `APPLE_PATH` | line 7 | 换成别的输入图片。Swap in a different source image. |

**如果换图片要注意 / Note when swapping images:** 输入图必须先转成灰度、干净背景效果最好；复杂背景/低对比度图片会在 Canny 那一步产生大量噪点小环，导致 `strokes` 数量暴涨。Works best with clean-background, high-contrast source images — busy backgrounds or low contrast will flood `strokes` with tiny noise loops from Canny.

## robot_path.py — SVG 线稿转机器人路径点

**运行 / Run:**

```bash
python svg_path/robot_path.py
```

这个脚本会 `import canny`，也就是说每次运行都会重新跑一遍上面的整个图像处理流程（包括重新生成 `apple_edges.png/svg`），不是读取现成的 SVG 文件。
This script `import`s `canny`, so every run re-executes the entire image pipeline above (including regenerating `apple_edges.png/svg`) — it does not parse the SVG file back in.

**做什么 / What it does:**

1. 复用 `canny.strokes`（原始像素点）和 `canny.simplify()`（精简），得到每条线精简后的像素坐标。
   Reuses `canny.strokes` (raw pixel points) and `canny.simplify()` to get each stroke's simplified pixel coordinates.
2. `map_to_canvas()`：把像素坐标等比缩放到物理画布尺寸，保持长宽比（取宽高缩放比中较小的一个），可选择坐标系原点朝上还是朝下。
   `map_to_canvas()`: scales pixel coordinates onto the physical canvas size, preserving aspect ratio (uses the smaller of the width/height scale factors), with a choice of Y-axis origin.
3. `order_strokes()`：贪心最近邻算法，从 `home_position` 出发，每次找离当前笔位置最近的下一条线（开放线条会按端点就近自动反向），减少抬笔空移的总距离。
   `order_strokes()`: greedy nearest-neighbor — starting from `home_position`, repeatedly picks whichever remaining stroke has an endpoint closest to the pen's current position (open strokes may be reversed), to cut down total pen-up travel.
4. `build_robot_path()` 把以上两步串起来，是给其他代码调用的主入口。
   `build_robot_path()` chains the two steps together and is the main entry point for other code to call.

**输出 / Output:** 不写文件，直接返回 Python 数据结构（还没接具体的机器人协议 / 硬件画布尺寸，这两个待硬件那边定了再接）：
No file output — returns a Python data structure directly (not yet wired to a specific robot protocol / real canvas size, both still pending from hardware):

```python
list[list[(x, y)]]
# 外层：每条连续落笔轨迹（轨迹之间是抬笔移动）
# outer list: one continuous pen-down waypoint sequence per stroke (pen lifts between them)
```

直接运行 `python svg_path/robot_path.py` 会打印统计信息，不返回文件：

```
strokes: 428
total points: 1761
baseline pen-up travel: 26312.4
optimized pen-up travel: 1498.2
reduction: 94.3%
```

**在自己代码里调用 / Calling it from other code:**

```python
from robot_path import build_robot_path

waypoints = build_robot_path(
    canvas_size=(300.0, 300.0),   # (width, height)，单位和原点待硬件确认 / unit + origin TBD from hardware
    origin="top-left",            # or "bottom-left"
    home_position=(0.0, 0.0),     # 笔的起始位置 / pen's starting position, same units as canvas_size
)
```

**可调参数 / Tunable parameters:**

| 参数 / Parameter | 作用 / Effect |
|---|---|
| `canvas_size=(width, height)` | 目标物理画布尺寸（占位值，等硬件确认实际尺寸和单位）。Target physical canvas size — currently a placeholder until hardware confirms the real size/unit. |
| `origin="top-left"` / `"bottom-left"` | 画布坐标系原点位置；`"bottom-left"` 会翻转 Y 轴（常见于绘图机/G-code 习惯）。Canvas coordinate origin; `"bottom-left"` flips the Y axis (common for plotters/G-code). |
| `home_position=(x, y)` | 笔的起始位置，影响排序算法第一条线怎么选。Pen's starting position — affects which stroke the ordering algorithm picks first. |

**待办 / Still open:** 具体输出协议（G-code、自定义 JSON 等）和画布真实物理尺寸/单位，等硬件那边定下来后再对接。
Concrete output protocol (G-code, custom JSON, etc.) and the real physical canvas size/unit are still pending from the hardware side.
