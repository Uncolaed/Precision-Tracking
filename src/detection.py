from . import config
from .utils import iou_xywh, dist2, center_of_bbox


class DetectionSmoother:
    def __init__(self, ttl: int = 3, match_dist: int = 60):
        self._ttl = ttl
        self._match_dist2 = match_dist ** 2
        self._tracked = []

    def update(self, fresh: list) -> list:
        used = [False] * len(fresh)

        for item in self._tracked:
            best_idx = -1
            best_dist = self._match_dist2

            for i, det in enumerate(fresh):
                if used[i]:
                    continue
                if det["class_id"] != item["det"]["class_id"]:
                    continue
                dx = det["cx"] - item["det"]["cx"]
                dy = det["cy"] - item["det"]["cy"]
                d2 = dx * dx + dy * dy
                if d2 < best_dist:
                    best_dist = d2
                    best_idx = i

            if best_idx >= 0:
                item["det"] = fresh[best_idx]
                item["ttl"] = self._ttl
                item["hits"] += 1
                used[best_idx] = True
            else:
                item["ttl"] -= 1

        self._tracked = [item for item in self._tracked if item["ttl"] > 0]

        for i, det in enumerate(fresh):
            if not used[i]:
                self._tracked.append({"det": det, "ttl": self._ttl, "hits": 1})

        self._tracked.sort(key=lambda item: (-item["hits"], -item["det"]["conf"]))
        return [item["det"] for item in self._tracked]

    def stable_only(self) -> list:
        return [item["det"] for item in self._tracked if item["hits"] >= 2]

    def reset(self):
        self._tracked.clear()


def detect(model, frame):
    result = model.predict(frame, conf=config.CONF, imgsz=config.IMGSZ, verbose=False)[0]
    detections = []

    if result.boxes is None or len(result.boxes) == 0:
        return detections

    xyxy = result.boxes.xyxy.cpu().numpy()
    confs = result.boxes.conf.cpu().numpy()
    clss = result.boxes.cls.cpu().numpy().astype(int)

    for (x1, y1, x2, y2), conf, cid in zip(xyxy, confs, clss):
        label = model.names[int(cid)]
        if config.TARGET_CLASS and label != config.TARGET_CLASS:
            continue
        x1, y1, x2, y2 = map(int, [x1, y1, x2, y2])
        w = x2 - x1
        h = y2 - y1
        detections.append({
            "bbox": (x1, y1, w, h),
            "cx": x1 + w // 2,
            "cy": y1 + h // 2,
            "conf": float(conf),
            "label": label,
            "class_id": int(cid),
        })

    return detections


def pick_center_target(dets, frame_cx, frame_cy):
    if not dets:
        return None
    return min(dets, key=lambda d: (d["cx"] - frame_cx) ** 2 + (d["cy"] - frame_cy) ** 2)


def pick_best_detection_for_reference(detections, ref_bbox, ref_class_id=None):
    if not detections or ref_bbox is None:
        return None

    rcx, rcy = center_of_bbox(ref_bbox)
    same_class = [d for d in detections if d["class_id"] == ref_class_id] if ref_class_id is not None else []
    candidates = same_class if same_class else detections
    max_d2 = config.REACQUIRE_MAX_DIST * config.REACQUIRE_MAX_DIST

    best = None
    best_score = -1e9

    for d in candidates:
        this_iou = iou_xywh(d["bbox"], ref_bbox)
        this_d2 = dist2((d["cx"], d["cy"]), (rcx, rcy))

        if this_iou < config.REACQUIRE_MIN_IOU and this_d2 > max_d2:
            continue

        score = 2.8 * this_iou + 1.0 / (1.0 + this_d2 / float(max_d2)) + 0.25 * d["conf"]
        if score > best_score:
            best_score = score
            best = d

    return best
