def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def opposite_dir(direction):
    direction = direction.lower()
    if direction == "cw":
        return "ccw"
    if direction == "ccw":
        return "cw"
    raise ValueError(f"Unsupported direction: {direction}")


def round_speed(speed, step):
    return int(round(speed / step) * step)


def iou_xywh(a, b):
    ax, ay, aw, ah = a
    bx, by, bw, bh = b
    ax2, ay2 = ax + aw, ay + ah
    bx2, by2 = bx + bw, by + bh
    ix1, iy1 = max(ax, bx), max(ay, by)
    ix2, iy2 = min(ax2, bx2), min(ay2, by2)
    iw = max(0, ix2 - ix1)
    ih = max(0, iy2 - iy1)
    inter = iw * ih
    union = aw * ah + bw * bh - inter
    return inter / union if union > 0 else 0.0


def dist2(p1, p2):
    dx = p1[0] - p2[0]
    dy = p1[1] - p2[1]
    return dx * dx + dy * dy


def center_of_bbox(b):
    x, y, w, h = b
    return int(x + w / 2), int(y + h / 2)


def in_deadband(cx, cy, fx, fy, bw, bh):
    return abs(cx - fx) <= bw // 2 and abs(cy - fy) <= bh // 2


def quadrant(cx, cy, fx, fy, bw, bh):
    if in_deadband(cx, cy, fx, fy, bw, bh):
        return "CENTER"
    if cx < fx and cy < fy:
        return "TOP-LEFT"
    if cx >= fx and cy < fy:
        return "TOP-RIGHT"
    if cx < fx and cy >= fy:
        return "BOTTOM-LEFT"
    return "BOTTOM-RIGHT"
