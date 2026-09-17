"""Show the actual asynchronous worker masks beside the controller overlay."""
import json
from pathlib import Path
import threading
import time

import cv2
import numpy as np


class ProcessingView:
    def __init__(self, config):
        self.rois = config.rois
        self.lock = threading.Lock()
        self.masks = {}
        self.started = False
        colors = json.loads((Path(__file__).parents[2] / "tools/color_ranges.json").read_text())
        self.labels = {}
        for name in ("BLACK", "RED", "GREEN", "MAGENTA", "ORANGE", "BLUE"):
            method = colors[name + "_COLOR_SPACE"]
            lower = colors["LOWER_" + name + ("" if method == "LAB" else "_HSV")]
            self.labels[(tuple(lower), method)] = name.lower()

    def record(self, roi, lower, method, mask):
        # Called on detector threads. Rendering stays on the main thread.
        region = next((name for name, value in self.rois.items()
                       if isinstance(value, list) and list(roi) == value), "ROI")
        color = self.labels.get((tuple(lower), method), method)
        with self.lock:
            self.masks[(region, color)] = (mask.copy(), time.monotonic())

    def compose(self, debug_frame):
        height, width = debug_frame.shape[:2]
        panel = np.full((height, 480, 3), 24, np.uint8)
        with self.lock:
            masks = sorted(self.masks.items())[:8]
        cv2.putText(panel, "Actual worker masks - latest available", (8, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, .45, (235, 235, 235), 1)
        if not masks:
            cv2.putText(panel, "Waiting for detector results...", (10, 55),
                        cv2.FONT_HERSHEY_SIMPLEX, .45, (200, 200, 200), 1)
        for i, ((region, color), (mask, timestamp)) in enumerate(masks):
            x = (i % 2) * 240
            y = 32 + (i // 2) * 110
            age = time.monotonic() - timestamp
            cv2.putText(panel, f"{region}/{color} {age:.1f}s", (x + 7, y + 15),
                        cv2.FONT_HERSHEY_SIMPLEX, .4, (200, 220, 245), 1)
            scale = min(224 / mask.shape[1], 78 / mask.shape[0])
            tile = cv2.resize(mask, (max(1, int(mask.shape[1] * scale)), max(1, int(mask.shape[0] * scale))),
                              interpolation=cv2.INTER_NEAREST)
            tile = cv2.cvtColor(tile, cv2.COLOR_GRAY2BGR)
            panel[y + 23:y + 23 + tile.shape[0], x + 7:x + 7 + tile.shape[1]] = tile
        return np.hstack((debug_frame, panel))

    def show(self, debug_frame):
        title = "Robot processing - ROIs, contours and masks"
        if not self.started:
            cv2.namedWindow(title, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(title, 1120, 480)
            self.started = True
        cv2.imshow(title, self.compose(debug_frame))


def install(config):
    import img_processing_functions as processing
    view = ProcessingView(config)
    processing.mask_observer = view.record
    processing.debug_frame_observer = view.show
    return view
