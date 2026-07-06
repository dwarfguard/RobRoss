# Scripts

Utility scripts for this repo. Each script gets its own section below —
when adding a new script, copy the section format used for
`mondrian_generator.py`.

**Two independent technical routes to the robot / 两条独立的技术路线:**
this repo currently generates robot paths from two unrelated sources, and
they are *not* variations of the same pipeline:

- **Mondrian route** (`mondrian_generator.py` + `mondrian_robot_path.py`):
  a procedurally generated vector design — exact rectangle/line geometry,
  colors, and widths are all known up front. The robot both traces lines
  *and* paints color fills.
- **Sketch / outline-tracing route** (`canny.py` + `sketch_robot_path.py`):
  edges guessed from an arbitrary bitmap image via Canny edge detection.
  The robot only traces monochrome outlines, no fills.

Mondrian 路线（`mondrian_generator.py` + `mondrian_robot_path.py`）是程序化生成的矢量设计，坐标/颜色/线宽全部精确已知，机器人既要描线也要涂色；素描/描边路线（`canny.py` + `sketch_robot_path.py`）是从任意位图图片猜测出的边缘，只描边不涂色。两条路线的路径生成逻辑相互独立，不要混着改；文件名也刻意用 `sketch_` / `mondrian_` 前缀分开，一眼能看出属于哪条路线。

Both routes' entry-point functions return the same top-level shape —
`{"canvas_size"/"canvas_size_mm": ..., "tools": [{"kind", "color", "strokes"}, ...]}`
— so downstream code doesn't need to special-case which route produced the
data.

Both routes reuse the same greedy nearest-neighbor ordering logic from
`path_ordering.py` (see below) rather than each reimplementing it.

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

## canny.py / sketch_robot_path.py

Image-to-robot-path pipeline for the sketch (outline-tracing) prototype:
turns a source image into a smooth SVG line drawing, then into an ordered
list of physical waypoints for the robot arm to trace.

图像转机器人路径的处理流程：先把源图片变成平滑的 SVG 线稿，再转换成一组排好顺序的机器人物理坐标路径点。

Run in sequence:

```
assets/apple.png -> scripts/canny.py -> output/apple_edges.png, output/apple_edges.svg
                                     \-> scripts/sketch_robot_path.py -> {"canvas_size", "tools"} (also written to output/sketch_robot_path.json)
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

### sketch_robot_path.py — SVG 线稿转机器人路径点

**Usage / 运行:**

```bash
python scripts/sketch_robot_path.py
```

这个脚本会 `import canny`，也就是说每次运行都会重新跑一遍上面的整个图像处理流程（包括重新生成 `output/apple_edges.png/svg`），不是读取现成的 SVG 文件。
This script `import`s `canny`, so every run re-executes the entire image pipeline above (including regenerating `output/apple_edges.png/svg`) — it does not parse the SVG file back in.

**What it does / 做什么:**

1. 复用 `canny.strokes`（原始像素点）和 `canny.simplify()`（精简），得到每条线精简后的像素坐标。
   Reuses `canny.strokes` (raw pixel points) and `canny.simplify()` to get each stroke's simplified pixel coordinates.
2. `map_to_canvas()`：把像素坐标等比缩放到物理画布尺寸，保持长宽比（取宽高缩放比中较小的一个），可选择坐标系原点朝上还是朝下。
   `map_to_canvas()`: scales pixel coordinates onto the physical canvas size, preserving aspect ratio (uses the smaller of the width/height scale factors), with a choice of Y-axis origin.
3. `order_strokes()`（来自 `path_ordering.py`，两条技术路线共用）：贪心最近邻算法，从 `home_position` 出发，每次找离当前笔位置最近的下一条线（开放线条会按端点就近自动反向），减少抬笔空移的总距离。
   `order_strokes()` (from `path_ordering.py`, shared by both technical routes): greedy nearest-neighbor — starting from `home_position`, repeatedly picks whichever remaining stroke has an endpoint closest to the pen's current position (open strokes may be reversed), to cut down total pen-up travel.
4. `build_sketch_robot_path()` 把以上两步串起来，是给其他代码调用的主入口。
   `build_sketch_robot_path()` chains the two steps together and is the main entry point for other code to call.

**Output / 输出:** 返回 Python 数据结构，直接运行时也会顺手写一份调试用的 JSON（还没接具体的机器人协议 / 硬件画布尺寸，这两个待硬件那边定了再接）：
Returns a Python data structure; running the script directly also writes a debug JSON dump (not yet wired to a specific robot protocol / real canvas size, both still pending from hardware):

```python
{
    "canvas_size": (width, height),
    "tools": [
        {"kind": "line", "color": "black", "strokes": list[list[(x, y)]]},
    ],
}
# 只有一个 tool（单色笔），和 mondrian_robot_path.py 的输出用同一套外层结构
# only one tool (single monochrome pen); same top-level shape as mondrian_robot_path.py's output
```

直接运行 `python scripts/sketch_robot_path.py` 会打印统计信息，并写出 `output/sketch_robot_path.json`：

```
strokes: 428
total points: 1761
baseline pen-up travel: 26312.4
optimized pen-up travel: 1498.2
reduction: 94.3%
wrote .../output/sketch_robot_path.json
```

**Calling it from other code / 在自己代码里调用:**

```python
from sketch_robot_path import build_sketch_robot_path

result = build_sketch_robot_path(
    canvas_size=(300.0, 300.0),   # (width, height)，单位和原点待硬件确认 / unit + origin TBD from hardware
    origin="top-left",            # or "bottom-left"
    home_position=(0.0, 0.0),     # 笔的起始位置 / pen's starting position, same units as canvas_size
)
waypoints = result["tools"][0]["strokes"]
```

**Options / 可调参数:**

| Parameter | Effect |
| --- | --- |
| `canvas_size=(width, height)` | 目标物理画布尺寸（占位值，等硬件确认实际尺寸和单位）。Target physical canvas size — currently a placeholder until hardware confirms the real size/unit. |
| `origin="top-left"` / `"bottom-left"` | 画布坐标系原点位置；`"bottom-left"` 会翻转 Y 轴（常见于绘图机/G-code 习惯）。Canvas coordinate origin; `"bottom-left"` flips the Y axis (common for plotters/G-code). |
| `home_position=(x, y)` | 笔的起始位置，影响排序算法第一条线怎么选。Pen's starting position — affects which stroke the ordering algorithm picks first. |

**Still open / 待办:** 具体输出协议（G-code、自定义 JSON 等）和画布真实物理尺寸/单位，等硬件那边定下来后再对接。
Concrete output protocol (G-code, custom JSON, etc.) and the real physical canvas size/unit are still pending from the hardware side.

---

## path_ordering.py

Shared greedy nearest-neighbor stroke ordering, used by both
`sketch_robot_path.py` (sketch route) and `mondrian_robot_path.py`
(Mondrian route) so the algorithm only lives in one place.

被 `sketch_robot_path.py`（素描路线）和 `mondrian_robot_path.py`（Mondrian 路线）共用的贪心最近邻路径排序逻辑，避免两边各写一份。

Exposes `order_strokes(strokes_data, home_position)` and
`total_travel_distance(strokes_points, home_position)` — see the "How it
works" note under `sketch_robot_path.py` above for what the algorithm does.
Not meant to be run directly; import from it instead.

---

## mondrian_robot_path.py

Converts a `mondrian_generator.py` design into robot paths: traces the
black grid/border lines, and paints the color-filled cells. This is the
**Mondrian technical route** — see the note at the top of this file for
how it differs from the sketch/outline-tracing route.

把 `mondrian_generator.py` 生成的设计转换成机器人路径：描黑色网格/边框线，并给色块涂色。这是 **Mondrian 技术路线**，跟素描/描边路线是两码事（见本文件开头的说明）。

### Usage / 运行

```bash
python scripts/mondrian_robot_path.py
```

This `import`s `mondrian_generator` directly and calls
`generate_mondrian_design()` — it does not parse the SVG file, and (unlike
`mondrian_generator.py`'s CLI) does not currently take a `--seed` flag;
edit the `seed=None` argument in the `__main__` block if you need a
reproducible design.
直接 `import mondrian_generator` 并调用 `generate_mondrian_design()`，不解析 SVG 文件；目前 `__main__` 里没有 `--seed` 命令行参数，需要复现某个设计的话直接改 `__main__` 里的 `seed=None`。

**What it does / 做什么:**

1. `trace_line()`：黑色网格/边框线走中心线一趟描完（笔宽和设计线宽基本一致，不用多趟扫）。
   `trace_line()`: each grid/border line is drawn in a single centerline pass (brush width and the design's line width are close enough that one pass suffices).
2. `fill_rect()`：给色块生成锯齿形（boustrophedon）扫描路径——四周先各内缩 `brush_width_mm / 2`，避免和已经画好的黑线重叠出界；行距等于笔宽；整块色一次落笔、一次抬笔画完。
   `fill_rect()`: generates a boustrophedon scan path for a color cell — insets each edge by `brush_width_mm / 2` so the fill doesn't paint over the already-drawn grid lines, row spacing equals the brush width, and the whole cell is one continuous pen-down stroke.
3. `build_mondrian_robot_path()`：调用 `mondrian_generator.generate_mondrian_design()` 拿到矩形/线条数据（复用现有数据，不重新解析 SVG），黑线单独一组，色块按颜色分组（对应"每种颜色专用画笔"的现实约束，见 `docs/Rob_Ross_Discuss.md`），组内用 `path_ordering.order_strokes()` 排序减少空移。
   `build_mondrian_robot_path()`: calls `mondrian_generator.generate_mondrian_design()` to get rectangle/line data (reusing existing data, no SVG re-parsing), groups the black lines separately from each fill color (matching the "dedicated brush per color" constraint in `docs/Rob_Ross_Discuss.md`), and orders strokes within each group with `path_ordering.order_strokes()` to cut down travel.

**Output / 输出:**

```python
{
    "canvas_size_mm": float,
    "tools": [
        {"kind": "line", "color": "black", "strokes": list[list[(x, y)]]},
        {"kind": "fill", "color": "#d62828", "strokes": list[list[(x, y)]]},
        ...
    ],
}
```

一组是"黑线描边"，其余每组对应一种色块颜色（涂色时按组切换画笔/颜料，组内顺序已经排好）。直接运行会为每组打印统计信息，并写出调试用的 `output/mondrian_robot_path.json`：
One group is the black line trace, and each remaining group is one fill color (switch brush/paint between groups when painting; strokes within a group are already ordered). Running the script directly prints per-group stats and writes a debug `output/mondrian_robot_path.json`:

```
[line] color=black strokes=12 points=24 travel=689.7mm
[fill] color=#f7c600 strokes=1 points=26 travel=266.7mm
[fill] color=#1d4ed8 strokes=1 points=26 travel=210.4mm
wrote .../output/mondrian_robot_path.json
```

**Options / 可调参数:**

| Parameter | Effect |
| --- | --- |
| `line_brush_width_mm` (default 6.0) | 描黑线用的笔宽，决定要不要多趟扫（当前固定一趟）。Brush width for the line trace — currently always a single pass regardless of this value. |
| `fill_brush_width_mm` (default 6.0) | 涂色笔宽，决定扫描行距和四周内缩量。Fill brush width — drives scan row spacing and the inset margin from each cell edge. |
| `home_position=(x, y)` | 笔的起始位置，影响每组排序算法第一条线怎么选。Pen's starting position — affects which stroke each group's ordering picks first. |

**Still open / 待办:** 如果笔宽和某次设计随机出的 `stroke_width` 差异变大，黑线可能需要多趟平行扫线才能画满，目前按"一趟中心线"处理。具体机器人执行协议同样待硬件确认。
If the brush width and a given design's randomized `stroke_width` diverge significantly, grid lines may need multiple parallel passes to fill cleanly — currently handled as a single centerline pass. The concrete robot execution protocol is likewise still pending from hardware.
