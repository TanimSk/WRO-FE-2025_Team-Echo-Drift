"""Native desktop vehicle and track editors with PyBullet previews."""
from __future__ import annotations

import argparse
import copy
import json
import math
import subprocess
import sys
import tempfile
from dataclasses import fields
from pathlib import Path

import cv2
import numpy as np
from PySide6 import QtCore, QtGui, QtWidgets as Q

from .profiles import (apply, default_path, payload, save, standard_config,
                       track_scale, validate, valid_post, parking_geometry, start_pose)
from .world import PyBulletWorld
from .virtual_camera import VirtualCamera


LABELS = {
    "mass": "Total mass (kg)", "length": "Chassis length (m)", "width": "Chassis width (m)",
    "wheel_radius": "Wheel radius (m)", "wheelbase": "Wheelbase (m)",
    "track_width": "Wheel centre spacing (m)", "max_speed_mps": "Maximum speed (m/s)",
    "max_steering_deg": "Maximum steering (degrees)", "drive_force": "Wheel motor torque (N m)",
    "servo_center_deg": "Servo centre (degrees)", "servo_span_deg": "Servo command half-range (degrees)",
    "ticks_per_revolution": "Encoder ticks / revolution", "tire_friction": "Tire friction coefficient",
    "steering_force": "Steering torque (N m)", "steering_rate": "Steering rate (rad/s)",
    "mount_height": "Camera height above chassis COM (m)", "mount_forward": "Camera forward offset (m)",
    "horizontal_fov_deg": "Horizontal field of view (degrees)", "pitch_deg": "Camera pitch (degrees)",
    "barrel_k1": "Radial distortion k1", "exposure_gain": "Image exposure multiplier",
    "outer_size": "Outer width (m)", "island_size": "Island width (m)",
    "length_scale": "Rectangle length / width", "ambient": "Ambient light (0..1)",
    "diffuse": "Directional light (0..1)", "light_z": "Light height (m)",
    "start_zone_half_width": "Start zone half-width X (m)",
    "start_zone_half_height": "Start zone half-height Y (m)",
    "lap_interval_seconds": "Minimum lap interval (s)",
    "drift_alpha": "Drift smoothing alpha (0..1)",
    "max_drift_threshold": "Drift acceptance threshold (m)",
    "estimated_wheel_radius": "Estimator wheel radius (m)",
    "estimated_ticks_per_rev": "Estimator ticks / revolution",
    "estimated_gear_ratio": "Estimator gear ratio",
    "custom": "Use custom parking / start pose",
    "center_x": "Bay centre X (m)", "center_y": "Bay centre Y (m)",
    "depth": "Bay depth (m)", "rotation_deg": "Bay rotation (degrees)",
    "start_offset_x": "Start offset along bay X (m)",
    "start_offset_y": "Start offset along bay Y (m)",
    "heading_deg": "Start heading relative to bay (degrees)",
}


class ImageView(Q.QLabel):
    clicked = QtCore.Signal(float, float)

    def __init__(self):
        super().__init__()
        self.setMinimumSize(400, 250)
        self.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.setStyleSheet("background:#e8edf2;border:1px solid #cbd5e1;border-radius:6px")
        self.image_size = (1, 1)

    def show_array(self, rgb):
        rgb = np.ascontiguousarray(rgb, dtype=np.uint8)
        h, w = rgb.shape[:2]
        image = QtGui.QImage(rgb.data, w, h, rgb.strides[0], QtGui.QImage.Format.Format_RGB888).copy()
        pixmap = QtGui.QPixmap.fromImage(image).scaled(self.size(), QtCore.Qt.AspectRatioMode.KeepAspectRatio,
                                                     QtCore.Qt.TransformationMode.SmoothTransformation)
        self.image_size = (pixmap.width(), pixmap.height())
        self.setPixmap(pixmap)

    def mousePressEvent(self, event):
        w, h = self.image_size
        x = (event.position().x() - (self.width() - w) / 2) / w
        y = (event.position().y() - (self.height() - h) / 2) / h
        if 0 <= x <= 1 and 0 <= y <= 1:
            self.clicked.emit(x, y)


class Editor(Q.QMainWindow):
    def __init__(self, kind, mode):
        super().__init__()
        self.kind, self.mode = kind, mode
        self.config = standard_config(mode)
        self.path = default_path(kind, mode)
        self.world = None
        self.temp = tempfile.TemporaryDirectory(prefix="vehicle-editor-")
        self.controls = {}
        self.dirty = False
        self.setWindowTitle(f"{'Vehicle configuration' if kind == 'vehicle' else 'Track setup'} - {mode}")
        self.resize(1320, 900)
        for group in ("vehicle", "track"):
            path = default_path(group, mode)
            if path.exists():
                apply(self.config, json.loads(path.read_text()), group)
        self.build_ui()
        self.rebuild()
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.start(100)

    def build_ui(self):
        central = Q.QWidget()
        self.setCentralWidget(central)
        root = Q.QVBoxLayout(central)
        title = Q.QLabel("Vehicle configuration" if self.kind == "vehicle" else "Track setup")
        title.setStyleSheet("font-size:26px;font-weight:600;color:#16324f")
        root.addWidget(title)
        row = Q.QHBoxLayout()
        for label, callback in (("Load profile", self.load), ("Save", self.save),
                                ("Save as", self.save_as), ("Standard values", self.defaults),
                                ("Launch driving view", self.launch)):
            button = Q.QPushButton(label)
            button.clicked.connect(callback)
            row.addWidget(button)
        root.addLayout(row)
        split = Q.QSplitter()
        root.addWidget(split, 1)
        self.tabs = Q.QTabWidget()
        self.tabs.setMinimumWidth(390)
        split.addWidget(self.tabs)
        groups = ("vehicle", "camera", "noise") if self.kind == "vehicle" else ("track", "parking", "odometry")
        for group in groups:
            scroll = Q.QScrollArea()
            scroll.setWidgetResizable(True)
            widget = Q.QWidget()
            form = Q.QFormLayout(widget)
            form.setSpacing(12)
            if group == "parking":
                note = Q.QLabel("Physical parking bay and robot spawn, not the odometry zone. "
                    "Enable custom placement to edit. Coordinates are world metres, with "
                    "(0, 0) at the track centre. The U-shaped bay opens along its local +Y. "
                    "Start offsets and heading rotate with the bay. Custom heading is explicit "
                    "and does not auto-flip with driving direction. Save and restart to apply.")
                note.setWordWrap(True)
                form.addRow(note)
            if group == "odometry":
                note = Q.QLabel(
                    "Saved with the track profile. Applied on the next simulator launch.\n\n"
                    "Start zone is centred on odometry (0, 0), the starting pose. "
                    "Full size is twice each half-size. It affects lap counting.\n\n"
                    "Drift settings affect open challenge only. Threshold 0 disables "
                    "its existing lap correction.\n\n"
                    "Estimator calibration does not change physical wheel geometry "
                    "or sensor noise (Vehicle editor)."
                )
                note.setWordWrap(True)
                form.addRow(note)
            for field in fields(getattr(self.config, group)):
                name = field.name
                if name == "posts" or (group == "camera" and name in ("width", "height", "output_bgr")):
                    continue
                value = getattr(getattr(self.config, group), name)
                if isinstance(value, bool):
                    control = Q.QCheckBox()
                    control.setChecked(value)
                    control.toggled.connect(self.changed)
                elif isinstance(value, str):
                    control = Q.QComboBox()
                    control.addItems(["square", "rectangle"])
                    control.setCurrentText(value)
                    control.currentTextChanged.connect(self.changed)
                else:
                    control = Q.QDoubleSpinBox()
                    control.setDecimals(4)
                    control.setRange(-10000, 10000)
                    control.setSingleStep(0.01 if abs(value) < 1 else 1)
                    if group == "parking":
                        control.setSingleStep(5 if name in ("rotation_deg", "heading_deg") else .01)
                    control.setValue(value)
                    control.valueChanged.connect(self.changed)
                self.controls[(group, name)] = control
                label = "Bay width (m)" if group == "parking" and name == "width" else LABELS.get(name, name.replace('_', ' ').capitalize())
                form.addRow(label, control)
            if group == "camera":
                form.addRow(Q.QLabel("Perception resolution: 640 x 480 (fixed to algorithm ROIs)"))
            scroll.setWidget(widget)
            self.tabs.addTab(scroll, group.title())
        if self.kind == "vehicle":
            self.roi_table = Q.QTableWidget(len(self.config.rois), 4)
            self.roi_table.setHorizontalHeaderLabels(["x1", "y1", "x2", "y2"])
            self.roi_table.setVerticalHeaderLabels(list(self.config.rois))
            self.roi_table.horizontalHeader().setSectionResizeMode(Q.QHeaderView.ResizeMode.Stretch)
            self.roi_table.itemChanged.connect(self.changed)
            self.tabs.addTab(self.roi_table, "ROIs")
        else:
            page = Q.QWidget()
            layout = Q.QVBoxLayout(page)
            layout.addWidget(Q.QLabel("Click the plan to place a post. Coordinates are metres.\nPosts are active in OBSTACLE mode."))
            self.post_color = Q.QComboBox()
            self.post_color.addItems(["red", "green"])
            layout.addWidget(self.post_color)
            self.post_table = Q.QTableWidget(0, 3)
            self.post_table.setHorizontalHeaderLabels(["X (m)", "Y (m)", "Colour"])
            self.post_table.horizontalHeader().setSectionResizeMode(Q.QHeaderView.ResizeMode.Stretch)
            self.post_table.itemChanged.connect(self.changed)
            layout.addWidget(self.post_table)
            delete = Q.QPushButton("Remove selected post")
            delete.clicked.connect(self.delete_post)
            layout.addWidget(delete)
            self.direction = Q.QComboBox()
            self.direction.addItems(["ccw", "cw"])
            self.direction.setCurrentText(self.config.direction)
            self.direction.currentTextChanged.connect(self.changed)
            layout.addWidget(Q.QLabel("Driving direction"))
            layout.addWidget(self.direction)
            self.random_posts = Q.QSpinBox()
            self.random_posts.setRange(0, 30)
            self.random_posts.setValue(self.config.random_posts)
            self.random_posts.valueChanged.connect(self.changed)
            layout.addWidget(Q.QLabel("Additional random posts"))
            layout.addWidget(self.random_posts)
            self.seed = Q.QSpinBox()
            self.seed.setRange(0, 1000000)
            self.seed.setValue(self.config.seed)
            self.seed.valueChanged.connect(self.changed)
            layout.addWidget(Q.QLabel("Random seed"))
            layout.addWidget(self.seed)
            self.tabs.addTab(page, "Posts / run")
        right = Q.QWidget()
        preview = Q.QVBoxLayout(right)
        preview.addWidget(Q.QLabel("Model preview" if self.kind == "vehicle" else "Track plan - click to add posts"))
        self.model_view = ImageView()
        preview.addWidget(self.model_view, 1)
        self.plan_status = Q.QLabel()
        self.plan_status.setWordWrap(True)
        if self.kind == "track":
            preview.addWidget(self.plan_status)
        if self.kind == "vehicle":
            orbit = Q.QHBoxLayout()
            self.orbit_yaw = Q.QSpinBox()
            self.orbit_yaw.setRange(-180, 180)
            self.orbit_yaw.setValue(45)
            self.orbit_pitch = Q.QSpinBox()
            self.orbit_pitch.setRange(-85, -5)
            self.orbit_pitch.setValue(-28)
            orbit.addWidget(Q.QLabel("Model view yaw"))
            orbit.addWidget(self.orbit_yaw)
            orbit.addWidget(Q.QLabel("Model view pitch"))
            orbit.addWidget(self.orbit_pitch)
            preview.addLayout(orbit)
        if self.kind == "track":
            self.model_view.clicked.connect(self.add_post)
        preview.addWidget(Q.QLabel("Robot camera - ROI overlay"))
        self.pov = ImageView()
        preview.addWidget(self.pov, 1)
        self.show_mask = Q.QCheckBox("Show calibrated black-wall mask")
        preview.addWidget(self.show_mask)
        manual = Q.QHBoxLayout()
        self.servo = Q.QDoubleSpinBox()
        self.servo.setRange(20, 170)
        self.servo.setValue(95)
        self.speed = Q.QDoubleSpinBox()
        self.speed.setRange(-100, 100)
        self.speed.setSuffix(" %")
        manual.addWidget(Q.QLabel("Preview servo"))
        manual.addWidget(self.servo)
        manual.addWidget(Q.QLabel("Motor"))
        manual.addWidget(self.speed)
        reset = Q.QPushButton("Reset pose")
        reset.clicked.connect(self.rebuild)
        manual.addWidget(reset)
        preview.addLayout(manual)
        preview.addWidget(Q.QLabel("Manual motor and servo values affect this preview only.\nWheel radius and encoder ticks must match odometry calibration (0.046 m / 2220 ticks)."))
        split.addWidget(right)
        split.setSizes([450, 820])
        self.status = Q.QLabel()
        self.status.setWordWrap(True)
        root.addWidget(self.status)
        self.debounce = QtCore.QTimer(self)
        self.debounce.setSingleShot(True)
        self.debounce.timeout.connect(self.rebuild)
        self.fill_tables()

    def fill_tables(self):
        table = self.roi_table if self.kind == "vehicle" else self.post_table
        table.blockSignals(True)
        rows = [list(v.values()) if isinstance(v, dict) else v for v in self.config.rois.values()] if self.kind == "vehicle" else self.config.track.posts
        table.setRowCount(len(rows))
        for i, row in enumerate(rows):
            for j, value in enumerate(row):
                table.setItem(i, j, Q.QTableWidgetItem(str(value)))
        table.blockSignals(False)

    def changed(self, *_):
        self.dirty = True
        if self.kind == "track":
            self.draw_plan()
        self.debounce.start(450)

    def collect(self, check=True):
        candidate = copy.deepcopy(self.config)
        for (group, name), widget in self.controls.items():
            previous = getattr(getattr(candidate, group), name)
            value = widget.isChecked() if isinstance(widget, Q.QCheckBox) else widget.currentText() if isinstance(widget, Q.QComboBox) else widget.value()
            setattr(getattr(candidate, group), name, int(value) if name in ("ticks_per_revolution", "estimated_ticks_per_rev") else value)
        if self.kind == "vehicle":
            for i, (name, region) in enumerate(candidate.rois.items()):
                row = [int(self.roi_table.item(i, j).text()) for j in range(4)]
                candidate.rois[name] = dict(zip(("x1", "y1", "x2", "y2"), row)) if isinstance(region, dict) else row
        else:
            candidate.track.posts = [[float(self.post_table.item(i, 0).text()), float(self.post_table.item(i, 1).text()),
                                      self.post_table.item(i, 2).text().lower()] for i in range(self.post_table.rowCount())]
            candidate.direction = self.direction.currentText()
            candidate.random_posts = self.random_posts.value()
            candidate.seed = self.seed.value()
        if check:
            validate(candidate)
        return candidate

    def rebuild(self):
        try:
            candidate = self.collect()
            candidate.gui = False
            candidate.preview = True
            if self.world is not None:
                self.world.close()
                self.world = None
            self.config = candidate
            self.world = PyBulletWorld(candidate, self.mode, Path(self.temp.name))
            self.camera = VirtualCamera(self.world, candidate.camera)
            self.camera.start()
            self.status.setText(f"{'Unsaved changes - ' if self.dirty else ''}{self.path}")
            self.tick()
        except (ValueError, TypeError, AttributeError) as exc:
            self.status.setText(f"Check configuration: {exc}")

    def tick(self):
        if self.world is None:
            return
        self.world.vehicle.command(self.speed.value(), self.servo.value())
        frame = self.camera.capture_array()
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        if self.show_mask.isChecked():
            colors = json.loads((Path(__file__).parents[2] / "tools/color_ranges.json").read_text())
            lab = colors["BLACK_COLOR_SPACE"] == "LAB"
            converted = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB if lab else cv2.COLOR_BGR2HSV)
            suffix = "" if lab else "_HSV"
            mask = cv2.inRange(converted, np.array(colors["LOWER_BLACK" + suffix]), np.array(colors["UPPER_BLACK" + suffix]))
            rgb = cv2.cvtColor(mask, cv2.COLOR_GRAY2RGB)
        for i, (name, roi) in enumerate(self.config.rois.items()):
            color = ((40 + i * 53) % 230, (130 + i * 37) % 230, (240 - i * 29) % 230)
            if isinstance(roi, dict):
                cv2.line(rgb, (roi['x1'], roi['y1']), (roi['x2'], roi['y2']), color, 2)
            else:
                x1, y1, x2, y2 = roi
                cv2.rectangle(rgb, (x1, y1), (x2, y2), color, 1)
                cv2.putText(rgb, name, (x1 + 2, max(12, y1 - 3)), cv2.FONT_HERSHEY_SIMPLEX, .35, color, 1)
        self.pov.show_array(rgb)
        if self.kind == "track":
            self.draw_plan()
        else:
            p = self.world.p
            position, _ = self.world.vehicle.pose()
            view = p.computeViewMatrixFromYawPitchRoll(position, max(.65, self.config.vehicle.length * 2),
                                                      self.orbit_yaw.value(), self.orbit_pitch.value(), 0, 2)
            projection = p.computeProjectionMatrixFOV(45, 1.6, .01, 20)
            image = p.getCameraImage(640, 400, view, projection, renderer=p.ER_TINY_RENDERER,
                                     physicsClientId=self.world.client_id)
            self.model_view.show_array(np.asarray(image[2]).reshape(400, 640, 4)[..., :3])

    def draw_plan(self):
        # Render edited values even when they cannot be built into a valid world.
        # The physics/camera preview stays at the last valid configuration.
        try:
            draft = self.collect(check=False)
        except (ValueError, TypeError, AttributeError):
            self.plan_status.setText("Finish entering numeric values to update the plan.")
            return
        for (group, name), control in self.controls.items():
            if group == "parking" and name != "custom":
                control.setEnabled(draft.parking.custom)
        problem = ""
        try:
            validate(draft)
        except ValueError as exc:
            problem = str(exc)
        self.plan_status.setStyleSheet("color:#b42318" if problem else "color:#16324f")
        self.plan_status.setText(
            f"Invalid draft: {problem}. Red bay/spawn show requested placement; camera keeps last valid setup."
            if problem else ("Live draft: magenta = bay, blue = starting footprint and heading. Save to keep changes."
                             if draft.parking.custom else "Default parking placement. Enable 'Use custom parking / start pose' to edit."))
        canvas = np.full((500, 700, 3), 245, np.uint8)
        t = draft.track
        bx, by, width, depth, yaw = parking_geometry(draft)
        spawn, heading = start_pose(draft, self.mode)
        self.plan_status.setText(self.plan_status.text() +
            f"\nBay ({bx:.2f}, {by:.2f}) m, size {width:.2f} x {depth:.2f} m. "
            f"Start ({spawn[0]:.2f}, {spawn[1]:.2f}) m, heading {math.degrees(heading):.1f} degrees.")
        self.plan_extent = max(1, t.outer_size/2, t.outer_size*track_scale(draft)/2,
                               abs(bx)+abs(width)/2+abs(depth)/2,
                               abs(by)+abs(width)/2+abs(depth)/2,
                               abs(spawn[0])+.3, abs(spawn[1])+.3) + .3
        def pixel(x, y):
            return (int(350 + x / (2 * self.plan_extent) * 500), int((.5 - y / (2 * self.plan_extent)) * 500))
        for size in (t.outer_size, t.island_size):
            cv2.rectangle(canvas, pixel(-size / 2, size * track_scale(draft) / 2),
                          pixel(size / 2, -size * track_scale(draft) / 2), (35, 45, 55), 4)
        posts = t.posts
        if (self.world is not None and self.mode == "OBSTACLE" and
            draft.track == self.config.track and draft.parking == self.config.parking and
            draft.random_posts == self.config.random_posts and draft.seed == self.config.seed and
            draft.direction == self.config.direction):
            posts = self.world.post_positions
        for x, y, color in posts:
            cv2.circle(canvas, pixel(x, y), max(5, int(t.post_radius / (2*self.plan_extent)*700)),
                       (210, 40, 40) if color == 'red' else (30, 150, 85), -1)
        corners = [pixel(bx + math.cos(yaw)*x-math.sin(yaw)*y,
                         by + math.sin(yaw)*x+math.cos(yaw)*y)
                   for x,y in [(-width/2,depth/2),(-width/2,-depth/2),
                               (width/2,-depth/2),(width/2,depth/2)]]
        cv2.polylines(canvas, [np.array(corners)], False, (210,40,40) if problem else (185,40,150), 3)
        footprint = [pixel(spawn[0]+math.cos(heading)*x-math.sin(heading)*y,
                           spawn[1]+math.sin(heading)*x+math.cos(heading)*y)
                     for x,y in [(-draft.vehicle.length/2,-draft.vehicle.track_width/2),
                                 (draft.vehicle.length/2,-draft.vehicle.track_width/2),
                                 (draft.vehicle.length/2,draft.vehicle.track_width/2),
                                 (-draft.vehicle.length/2,draft.vehicle.track_width/2)]]
        cv2.polylines(canvas, [np.array(footprint)], True, (210,40,40) if problem else (20,80,200), 2)
        cv2.arrowedLine(canvas, pixel(*spawn[:2]),
                       pixel(spawn[0]+.2*math.cos(heading), spawn[1]+.2*math.sin(heading)),
                       (20,80,200), 2, tipLength=.3)
        cv2.circle(canvas, pixel(0, 0), 3, (100,100,100), -1)
        cv2.putText(canvas, f"{t.outer_size:.2f} m x {t.outer_size*track_scale(draft):.2f} m", (15, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, .6, (30, 40, 50), 1)
        self.model_view.show_array(canvas)

    def add_post(self, nx, ny):
        x, y = round((nx - .5) * 2 * self.plan_extent * 700 / 500, 3), round((.5 - ny) * 2 * self.plan_extent, 3)
        try:
            config = self.collect()
            if not valid_post(config, x, y, [post[:2] for post in config.track.posts]):
                raise ValueError("That location overlaps a wall, parking bay, start or another post")
            config.track.posts.append([x, y, self.post_color.currentText()])
            self.config = config
            self.fill_tables()
            self.changed()
        except ValueError as exc:
            self.status.setText(str(exc))

    def delete_post(self):
        row = self.post_table.currentRow()
        if row >= 0:
            self.post_table.removeRow(row)
            self.changed()

    def refresh_controls(self):
        for (group, name), widget in self.controls.items():
            value = getattr(getattr(self.config, group), name)
            widget.blockSignals(True)
            if isinstance(widget, Q.QCheckBox): widget.setChecked(value)
            elif isinstance(widget, Q.QComboBox): widget.setCurrentText(value)
            else: widget.setValue(value)
            widget.blockSignals(False)
        if self.kind == "track":
            self.direction.setCurrentText(self.config.direction)
            self.random_posts.setValue(self.config.random_posts)
            self.seed.setValue(self.config.seed)
        self.fill_tables()
        self.rebuild()

    def defaults(self):
        apply(self.config, payload(standard_config(self.mode), self.kind), self.kind)
        self.dirty = True
        self.refresh_controls()

    def load(self):
        filename, _ = Q.QFileDialog.getOpenFileName(self, "Load profile", str(self.path.parent), "JSON (*.json)")
        if filename:
            try:
                candidate = copy.deepcopy(self.config)
                apply(candidate, json.loads(Path(filename).read_text()), self.kind)
                validate(candidate)
                self.config, self.path, self.dirty = candidate, Path(filename), False
                self.refresh_controls()
            except (ValueError, OSError) as exc:
                self.status.setText(str(exc))

    def save(self):
        try:
            candidate = self.collect()
            save(candidate, self.kind, self.path)
            self.config = candidate
            self.dirty = False
            self.status.setText(f"Saved {self.path}")
            return True
        except (ValueError, OSError) as exc:
            self.status.setText(str(exc))
            return False

    def save_as(self):
        filename, _ = Q.QFileDialog.getSaveFileName(self, "Save profile", str(self.path), "JSON (*.json)")
        if filename:
            self.path = Path(filename)
            self.save()

    def launch(self):
        if not self.save(): return
        script = Path(__file__).parents[1] / "simulator.py"
        subprocess.Popen([sys.executable, str(script), "--mode", "obstacle" if self.mode == "OBSTACLE" else "open",
                          f"--{self.kind}-profile", str(self.path)])

    def closeEvent(self, event):
        if self.dirty:
            choice = Q.QMessageBox.question(self, "Unsaved settings", "Save changes before closing?",
                Q.QMessageBox.StandardButton.Save | Q.QMessageBox.StandardButton.Discard | Q.QMessageBox.StandardButton.Cancel)
            if choice == Q.QMessageBox.StandardButton.Cancel or (choice == Q.QMessageBox.StandardButton.Save and not self.save()):
                event.ignore()
                return
        self.timer.stop()
        if self.world: self.world.close()
        self.temp.cleanup()
        event.accept()


def main(kind):
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=["open", "obstacle"], default="open")
    parser.add_argument("--screenshot", help="Render one preview to a PNG and exit (GUI QA)")
    args = parser.parse_args()
    application = Q.QApplication.instance() or Q.QApplication(sys.argv[:1])
    application.setStyle("Fusion")
    window = Editor(kind, "OBSTACLE" if args.mode == "obstacle" else "NO_OBSTACLE")
    window.show()
    if args.screenshot:
        def capture():
            window.grab().save(args.screenshot)
            window.dirty = False
            window.close()
            application.quit()
        QtCore.QTimer.singleShot(1200, capture)
    return application.exec()
