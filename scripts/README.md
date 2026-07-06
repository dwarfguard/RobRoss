# Scripts

Utility scripts for this repo. Each script gets its own section below —
when adding a new script, copy the section format used for
`mondrian_generator.py`.

---

## mondrian_generator.py

Generates a randomized Mondrian/De Stijl-style SVG: a 12in x 12in canvas
recursively subdivided into cells, a few of which are filled with the
classic red/yellow/blue (and occasionally gray) accents, separated by
thick black grid lines.

### Usage

```bash
# Random graphic, new layout every run
python3 scripts/mondrian_generator.py

# Reproduce a specific graphic
python3 scripts/mondrian_generator.py --seed 2124073818
```

Output is written to `output/mondrian_preview.svg` (directory is created
if missing).

Every run prints the seed used, e.g.:

```
Generated output/mondrian_preview.svg (seed=2124073818)
```

Copy that seed value into `--seed` to regenerate the exact same graphic
later.

### Options

| Flag | Description |
| --- | --- |
| `--seed N` | Use a fixed integer seed for reproducible output. Omit for a random seed (printed after generation). |

### How it works

1. `subdivide()` recursively splits the canvas rectangle into two
   (vertical or horizontal cut, random position) until cells hit a
   minimum size or a depth-scaled stop probability triggers. This
   produces the leaf cells and the internal grid lines in one pass.
2. `generate_mondrian_svg()` picks 2-4 leaf cells at random to fill with
   accent colors (rest stay white via the background rect), picks a
   random stroke width for the grid/border lines, and assembles the SVG
   string.
3. `main()` resolves/reports the seed and writes the file.

### Tuning knobs (top of file)

| Constant | Effect |
| --- | --- |
| `CANVAS_SIZE_MM` | Physical canvas size (defaults to 12in square). |
| `ACCENT_COLORS` | Palette used for colored blocks. |
| `NEUTRAL_ACCENT` | Extra gray accent, added to the palette ~25% of the time. |
| `MIN_CELL_FRACTION` | Smallest allowed cell edge, as a fraction of canvas size. Lower = smaller/more numerous cells possible. |
| `MAX_SPLIT_DEPTH` | Hard cap on recursion depth. |

Accent count (2-4), stroke width (5-8mm), and the per-depth stop
probability are inline in `generate_mondrian_svg()` / `subdivide()`
rather than top-level constants — pull them up if they need to become
tunable too.

### Ideas for future features

- CLI flags for canvas size, min cell fraction, accent count range, and
  stroke width range (avoid hardcoding once more than one param needs
  tuning).
- Export to PNG/PDF alongside SVG.
- Alternate palettes (e.g. monochrome, pastel) selectable via flag.
- Batch mode: generate N variations at once into `output/`.

---

## canny.py / robot_path.py

Image-to-robot-path pipeline for the sketch (outline-tracing) prototype:
turns a source image into a smooth SVG line drawing, then into an ordered
list of physical waypoints for the robot arm to trace.

图像转机器人路径的处理流程：先把源图片变成平滑的 SVG 线稿，再转换成一组排好顺序的机器人物理坐标路径点。

Run in sequence:

```
assets/apple.png -> scripts/canny.py -> output/apple_edges.png, output/apple_edges.svg
                                     \-> scripts/robot_path.py -> ordered waypoints (Python data, no file output yet)
```

### canny.py — 图像转 SVG 线稿

**Usage / 运行:**

```bash
source .venv/bin/activate
python scripts/canny.py
```

**What it does / 做什么:**

1. 读取 `assets/apple.png`（灰度），跑 Canny 边缘检测。
   Reads `assets/apple.png` (grayscale), runs Canny edge detection.
2. 用 `skimage.morphology.skeletonize` 把边缘细化成单像素宽的线，避免后续重复描边（`cv2.findContours` 会把一条线的两侧当成一个色块描边，产生镜像重复线，所以这里不用它）。
   Skeletonizes the edges to 1px-wide centerlines, so each stroke gets traced once instead of `cv2.findContours` walking both sides of the stroke as a mirrored duplicate.
3. 在骨架像素图上直接按图遍历（把端点/分叉点当节点），得到 `strokes`：每条独立线的像素点列表，以及是否闭合（比如苹果表皮上的小圆点瑕疵是闭合环）。
   Walks the skeleton pixel graph directly (endpoints/junctions as nodes) to produce `strokes`: one pixel-point list per independent line, plus whether it's a closed loop (e.g. the small blemish rings on the apple).
4. 用 `simplify()`（Douglas-Peucker）精简点数，再用 `catmull_rom_path()` 把折线拟合成平滑的三次贝塞尔曲线，写出 SVG。
   Simplifies point count (Douglas-Peucker) and fits a smooth cubic Bezier curve through the points (Catmull-Rom), then writes the SVG.

**Output / 输出** (both git-ignored, regenerated each run):

- `output/apple_edges.png` — Canny 边缘位图 / raw Canny edge bitmap
- `output/apple_edges.svg` — 平滑矢量线稿 / smoothed vector line art

**Options / 可调参数** (edit directly in the script — no CLI flags yet):

| Parameter | Where | Effect |
| --- | --- | --- |
| `threshold1`, `threshold2` in `cv2.Canny(...)` | in the `cv2.Canny(...)` call | 边缘检测的灵敏度。调低会检测出更多细节/噪点，调高则只保留强边缘。Lower = more detail/noise, higher = only strong edges. |
| `0.002` in `simplify()` (the `epsilon` multiplier) | inside `simplify()` | 折线简化程度。数值越大，点越少、线越直；数值越小，越贴近原始像素轨迹但点更多。Larger = fewer points/straighter lines; smaller = closer to raw pixels but more points. |
| `APPLE_PATH` | top of file | 换成别的输入图片。Swap in a different source image. |

**Note when swapping images / 换图片注意:** 输入图必须先转成灰度、干净背景效果最好；复杂背景/低对比度图片会在 Canny 那一步产生大量噪点小环，导致 `strokes` 数量暴涨。Works best with clean-background, high-contrast source images — busy backgrounds or low contrast will flood `strokes` with tiny noise loops from Canny.

### robot_path.py — SVG 线稿转机器人路径点

**Usage / 运行:**

```bash
python scripts/robot_path.py
```

这个脚本会 `import canny`，也就是说每次运行都会重新跑一遍上面的整个图像处理流程（包括重新生成 `output/apple_edges.png/svg`），不是读取现成的 SVG 文件。
This script `import`s `canny`, so every run re-executes the entire image pipeline above (including regenerating `output/apple_edges.png/svg`) — it does not parse the SVG file back in.

**What it does / 做什么:**

1. 复用 `canny.strokes`（原始像素点）和 `canny.simplify()`（精简），得到每条线精简后的像素坐标。
   Reuses `canny.strokes` (raw pixel points) and `canny.simplify()` to get each stroke's simplified pixel coordinates.
2. `map_to_canvas()`：把像素坐标等比缩放到物理画布尺寸，保持长宽比（取宽高缩放比中较小的一个），可选择坐标系原点朝上还是朝下。
   `map_to_canvas()`: scales pixel coordinates onto the physical canvas size, preserving aspect ratio (uses the smaller of the width/height scale factors), with a choice of Y-axis origin.
3. `order_strokes()`：贪心最近邻算法，从 `home_position` 出发，每次找离当前笔位置最近的下一条线（开放线条会按端点就近自动反向），减少抬笔空移的总距离。
   `order_strokes()`: greedy nearest-neighbor — starting from `home_position`, repeatedly picks whichever remaining stroke has an endpoint closest to the pen's current position (open strokes may be reversed), to cut down total pen-up travel.
4. `build_robot_path()` 把以上两步串起来，是给其他代码调用的主入口。
   `build_robot_path()` chains the two steps together and is the main entry point for other code to call.

**Output / 输出:** 不写文件，直接返回 Python 数据结构（还没接具体的机器人协议 / 硬件画布尺寸，这两个待硬件那边定了再接）：
No file output — returns a Python data structure directly (not yet wired to a specific robot protocol / real canvas size, both still pending from hardware):

```python
list[list[(x, y)]]
# 外层：每条连续落笔轨迹（轨迹之间是抬笔移动）
# outer list: one continuous pen-down waypoint sequence per stroke (pen lifts between them)
```

直接运行 `python scripts/robot_path.py` 会打印统计信息，不返回文件：

```
strokes: 428
total points: 1761
baseline pen-up travel: 26312.4
optimized pen-up travel: 1498.2
reduction: 94.3%
```

**Calling it from other code / 在自己代码里调用:**

```python
from robot_path import build_robot_path

waypoints = build_robot_path(
    canvas_size=(300.0, 300.0),   # (width, height)，单位和原点待硬件确认 / unit + origin TBD from hardware
    origin="top-left",            # or "bottom-left"
    home_position=(0.0, 0.0),     # 笔的起始位置 / pen's starting position, same units as canvas_size
)
```

**Options / 可调参数:**

| Parameter | Effect |
| --- | --- |
| `canvas_size=(width, height)` | 目标物理画布尺寸（占位值，等硬件确认实际尺寸和单位）。Target physical canvas size — currently a placeholder until hardware confirms the real size/unit. |
| `origin="top-left"` / `"bottom-left"` | 画布坐标系原点位置；`"bottom-left"` 会翻转 Y 轴（常见于绘图机/G-code 习惯）。Canvas coordinate origin; `"bottom-left"` flips the Y axis (common for plotters/G-code). |
| `home_position=(x, y)` | 笔的起始位置，影响排序算法第一条线怎么选。Pen's starting position — affects which stroke the ordering algorithm picks first. |

**Still open / 待办:** 具体输出协议（G-code、自定义 JSON 等）和画布真实物理尺寸/单位，等硬件那边定下来后再对接。
Concrete output protocol (G-code, custom JSON, etc.) and the real physical canvas size/unit are still pending from the hardware side.
