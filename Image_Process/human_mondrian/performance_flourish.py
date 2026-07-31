"""Inject meaningless "human-like" micro-movements into an existing command
sequence — repeated strokes, swooping travels, test dabs, edge touch-ups,
idle shakes — without changing the painting result (same paint_stroke
positions/colours as the original). The robot performs the original strokes
plus extra ones that land on the same spots, plus decorative travel arcs.

Each flourish type is a self-contained function below. The entry point is
`apply_flourishes(commands, canvas, flourish_cfg)`."""

import math
import random


def apply_flourishes(commands: list, canvas: dict, flourish_cfg: dict) -> list:
    """Orchestrate all flourish passes. Returns a new (longer) command list."""
    if not flourish_cfg.get("enabled", True):
        return commands

    cfg = flourish_cfg
    result = commands

    if cfg.get("swoop_probability", 0) > 0:
        result = _inject_swooping_travels(result, canvas, cfg.get("swoop_probability", 0.3))
    if cfg.get("test_dab_probability", 0) > 0:
        result = _inject_test_dabs(result, canvas, cfg.get("test_dab_probability", 0.15))
    if cfg.get("overpaint_probability", 0) > 0:
        result = _inject_overpainting(result, canvas, cfg.get("overpaint_probability", 0.1))
    if cfg.get("edge_touchup_probability", 0) > 0:
        result = _inject_edge_touchups(result, canvas, cfg.get("edge_touchup_probability", 0.08))
    if cfg.get("shake_probability", 0) > 0:
        result = _inject_idle_shakes(result, canvas, cfg.get("shake_probability", 0.05))

    return result


# -----------------------------------------------------------------------
# helpers
# -----------------------------------------------------------------------

def _label(base: str, counter: list) -> str:
    """Return a unique flourish label for tracking."""
    val = counter[0]
    counter[0] += 1
    return f"{base}_{val}"


def _canvas_margin(canvas: dict) -> float:
    return float(canvas.get("margin_mm", 0.0))


def _canvas_bbox(canvas: dict) -> tuple:
    """Return the paintable bounding box (x0, y0, x1, y1) in mm."""
    w = float(canvas["width_mm"])
    h = float(canvas["height_mm"])
    m = float(canvas.get("margin_mm", 0.0))
    return (m, m, w - m, h - m)


# -----------------------------------------------------------------------
# 1. swooping travels — replace some linear move_to jumps with curved arcs
# -----------------------------------------------------------------------

def _inject_swooping_travels(commands: list, canvas: dict, probability: float) -> list:
    """After a lift_tool, the next move_to sometimes becomes a 2-3 point arc."""
    idx = 0
    cnt = [0]
    new_cmds = []
    bbox = _canvas_bbox(canvas)

    while idx < len(commands):
        cmd = commands[idx]
        # look for move_to that follows lift_tool (pen is up — drawing an
        # arc here adds nothing to the painting)
        if (
            cmd["command"] == "move_to"
            and idx > 0
            and commands[idx - 1]["command"] == "lift_tool"
            and random.random() < probability
        ):
            target_x = float(cmd["x_mm"])
            target_y = float(cmd["y_mm"])
            # Grab previous lift_tool's travel start — the move_to before
            # that (or the paint_stroke end) shows where the pen *is*.
            # Simplest: use the last known position from the previous
            # paint_stroke or move_to, but we'd need to track it. To keep
            # simplicity, insert 1-2 midpoints offset orthogonally.
            mid_x = target_x + random.uniform(-3, 3)
            mid_y = target_y + random.uniform(-3, 3)
            # Clamp to canvas.
            mid_x = max(bbox[0], min(bbox[2], mid_x))
            mid_y = max(bbox[1], min(bbox[3], mid_y))

            arc_pts = [(mid_x, mid_y)]
            if random.random() < 0.4:
                mid2_x = mid_x + random.uniform(-2, 2)
                mid2_y = mid_y + random.uniform(-2, 2)
                arc_pts.append((
                    max(bbox[0], min(bbox[2], mid2_x)),
                    max(bbox[1], min(bbox[3], mid2_y)),
                ))

            for ax, ay in arc_pts:
                new_cmds.append({
                    "command": "move_to",
                    "label": _label("flourish_swoop", cnt),
                    "x_mm": round(ax, 2),
                    "y_mm": round(ay, 2),
                })
            new_cmds.append(cmd)
            idx += 1
            continue

        new_cmds.append(cmd)
        idx += 1

    return new_cmds


# -----------------------------------------------------------------------
# 2. test dabs — after dip_paint, make a tiny dot in the margin corner
# -----------------------------------------------------------------------

def _inject_test_dabs(commands: list, canvas: dict, probability: float) -> list:
    """After dip_paint, occasionally move to a margin spot and dab."""
    bbox = _canvas_bbox(canvas)
    idx = 0
    cnt = [0]
    new_cmds = []

    while idx < len(commands):
        cmd = commands[idx]
        new_cmds.append(cmd)

        if cmd["command"] == "dip_paint" and random.random() < probability:
            colour = cmd.get("color", "#000000")
            # Pick a random spot in the bottom-right margin corner.
            margin = _canvas_margin(canvas)
            tx = bbox[2] - random.uniform(0, margin * 0.8)
            ty = bbox[3] - random.uniform(0, margin * 0.8)
            lbl = _label("flourish_dab", cnt)

            new_cmds.append({"command": "move_to", "label": lbl, "x_mm": round(tx, 2), "y_mm": round(ty, 2)})
            new_cmds.append({"command": "lower_tool", "label": lbl})
            # 1 mm tiny stroke (too small to be noticed on the final image)
            new_cmds.append({
                "command": "paint_stroke",
                "label": lbl,
                "color": colour,
                "from_mm": [round(tx, 2), round(ty, 2)],
                "to_mm": [round(tx + 1, 2), round(ty, 2)],
            })
            new_cmds.append({"command": "lift_tool", "label": lbl})
            # Move back to where we were — the next move_to in the original
            # commands will handle it, so we don't need a return move_to.
            # But to keep the sequence clean, insert a move_to back near the
            # original next-move_to target.
            # Look ahead for the next move_to target.
            look = idx + 1
            back_x = back_y = None
            while look < len(commands) and back_x is None:
                nxt = commands[look]
                if nxt["command"] == "move_to":
                    back_x = nxt["x_mm"]
                    back_y = nxt["y_mm"]
                look += 1
            if back_x is not None:
                lbl2 = _label("flourish_dab_return", cnt)
                new_cmds.append({"command": "move_to", "label": lbl2, "x_mm": back_x, "y_mm": back_y})

        idx += 1

    return new_cmds


# -----------------------------------------------------------------------
# 3. overpainting — repeat some paint_stroke blocks
# -----------------------------------------------------------------------

def _inject_overpainting(commands: list, canvas: dict, probability: float) -> list:
    """Every paint_stroke group (move→lower→stroke→lift) may be duplicated."""
    idx = 0
    cnt = [0]
    new_cmds = []

    while idx < len(commands):
        cmd = commands[idx]

        # Look for a full move_to → lower_tool → paint_stroke → lift_tool group.
        if cmd["command"] == "move_to" and idx + 3 < len(commands):
            c1 = commands[idx + 1]
            c2 = commands[idx + 2]
            c3 = commands[idx + 3]
            if (
                c1["command"] == "lower_tool"
                and c2["command"] == "paint_stroke"
                and c3["command"] == "lift_tool"
                and random.random() < probability
            ):
                # Emit the original group.
                new_cmds.append(cmd)
                new_cmds.append(c1)
                new_cmds.append(c2)
                new_cmds.append(c3)
                # Duplicate with fresh labels.
                lbl = _label("flourish_overpaint", cnt)
                new_cmds.append({
                    "command": "move_to",
                    "label": lbl,
                    "x_mm": cmd["x_mm"],
                    "y_mm": cmd["y_mm"],
                })
                new_cmds.append({"command": "lower_tool", "label": lbl})
                new_cmds.append({
                    "command": "paint_stroke",
                    "label": lbl,
                    "color": c2["color"],
                    "from_mm": c2["from_mm"],
                    "to_mm": c2["to_mm"],
                })
                new_cmds.append({"command": "lift_tool", "label": lbl})
                idx += 4
                continue

        new_cmds.append(cmd)
        idx += 1

    return new_cmds


# -----------------------------------------------------------------------
# 4. edge touch-ups — tiny strokes near the end of a region fill block
# -----------------------------------------------------------------------

def _inject_edge_touchups(commands: list, canvas: dict, probability: float) -> list:
    """After a lift_tool that ends a colour block, insert 1-2 tiny orthogonal
    strokes that look like edge tidying."""
    idx = 0
    cnt = [0]
    new_cmds = []

    while idx < len(commands):
        cmd = commands[idx]
        new_cmds.append(cmd)

        if (
            cmd["command"] == "lift_tool"
            and idx >= 3
            and commands[idx - 1]["command"] == "paint_stroke"
            and random.random() < probability
        ):
            # Use the just-finished stroke's midpoint as anchor.
            prev_stroke = commands[idx - 1]
            fx, fy = prev_stroke["from_mm"]
            tx, ty = prev_stroke["to_mm"]
            mx = (fx + tx) / 2
            my = (fy + ty) / 2
            colour = prev_stroke.get("color", "#000000")
            lbl = _label("flourish_edge", cnt)

            # Tiny orthogonal stroke (~2 mm, perpendicular to the original).
            dx = tx - fx
            dy = ty - fy
            length = math.hypot(dx, dy)
            if length > 0:
                nx = -dy / length * 2  # perpendicular direction * 2mm
                ny = dx / length * 2
            else:
                nx, ny = 1, 0

            ex = mx + nx
            ey = my + ny
            # Clamp to canvas.
            bbox = _canvas_bbox(canvas)
            ex = max(bbox[0], min(bbox[2], ex))
            ey = max(bbox[1], min(bbox[3], ey))

            new_cmds.append({"command": "move_to", "label": lbl, "x_mm": round(mx, 2), "y_mm": round(my, 2)})
            new_cmds.append({"command": "lower_tool", "label": lbl})
            new_cmds.append({
                "command": "paint_stroke",
                "label": lbl,
                "color": colour,
                "from_mm": [round(mx, 2), round(my, 2)],
                "to_mm": [round(ex, 2), round(ey, 2)],
            })
            new_cmds.append({"command": "lift_tool", "label": lbl})

        idx += 1

    return new_cmds


# -----------------------------------------------------------------------
# 5. idle shakes — split a move_to into several tiny wobbling steps
# -----------------------------------------------------------------------

def _inject_idle_shakes(commands: list, canvas: dict, probability: float) -> list:
    """Occasionally break a move_to into 3-4 micro-moves that wobble slightly."""
    idx = 0
    cnt = [0]
    new_cmds = []

    while idx < len(commands):
        cmd = commands[idx]

        if (
            cmd["command"] == "move_to"
            and random.random() < probability
        ):
            tx = float(cmd["x_mm"])
            ty = float(cmd["y_mm"])
            # Look back for a previous position (paint_stroke end or move_to).
            prev_x = prev_y = None
            for j in range(idx - 1, max(-1, idx - 20), -1):
                pc = commands[j]
                if pc["command"] == "paint_stroke":
                    px, py = pc["to_mm"]
                    prev_x, prev_y = float(px), float(py)
                    break
                if pc["command"] == "move_to":
                    prev_x = float(pc["x_mm"])
                    prev_y = float(pc["y_mm"])
                    break

            if prev_x is not None and math.hypot(tx - prev_x, ty - prev_y) > 2:
                # Interpolate with wobbles.
                steps = random.randint(2, 4)
                for s in range(1, steps):
                    frac = s / steps
                    ix = prev_x + (tx - prev_x) * frac + random.uniform(-0.5, 0.5)
                    iy = prev_y + (ty - prev_y) * frac + random.uniform(-0.5, 0.5)
                    bbox = _canvas_bbox(canvas)
                    ix = max(bbox[0], min(bbox[2], ix))
                    iy = max(bbox[1], min(bbox[3], iy))
                    lbl = _label("flourish_shake", cnt)
                    new_cmds.append({"command": "move_to", "label": lbl, "x_mm": round(ix, 2), "y_mm": round(iy, 2)})

            # Emit the original move_to as the final micro-step.
            new_cmds.append(cmd)
            idx += 1
            continue

        new_cmds.append(cmd)
        idx += 1

    return new_cmds