"""Track construction, fixed-step physics, and run logging."""

from __future__ import annotations

import csv
import json
import math
import random
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

from hardware import SimulatorRestartRequested
from .ui import SimulatorUI
from .vehicle import SimVehicle


class PyBulletWorld:
    def __init__(self, config, mode: str, output_root: Path):
        try:
            import pybullet as p
        except ImportError as exc:
            raise RuntimeError("PyBullet is required for --sim. Install sim/requirements.txt") from exc
        self.p = p
        self.config = config
        self.mode = mode
        self.client_id = p.connect(p.GUI if config.gui else p.DIRECT)
        p.setGravity(0, 0, -9.81, physicsClientId=self.client_id)
        p.setTimeStep(1.0 / config.physics_hz, physicsClientId=self.client_id)
        p.setPhysicsEngineParameter(numSolverIterations=80, physicsClientId=self.client_id)
        if config.gui:
            # Keep PyBullet's right-side debug panel visible. SimulatorUI uses this
            # native panel for sliders and push buttons.
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1, physicsClientId=self.client_id)
            p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0, physicsClientId=self.client_id)

        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.output_directory = output_root / stamp
        self.output_directory.mkdir(parents=True, exist_ok=True)
        self._ground_file = (self.output_directory / "ground_truth.csv").open("w", newline="")
        self._ground_writer = csv.writer(self._ground_file)
        self._ground_writer.writerow(("time_s", "x_m", "y_m", "yaw_deg"))
        (self.output_directory / "run_config.json").write_text(json.dumps(_as_json(config), indent=2))

        self._rng = random.Random(config.seed)
        self.colors = self._load_colors()
        self._build_track()
        start, yaw = self._start_pose(mode, config.direction)
        self.vehicle = SimVehicle(p, self.client_id, config.vehicle, start, yaw)
        self.running = not config.gui
        self._events = set()
        self.ui = SimulatorUI(p, self.client_id, config) if config.gui else None
        if self.ui is not None:
            print("Simulator ready - use the right-side panel and press START / RESUME.")
        self.start_yaw = yaw
        self.start_wall_time = time.monotonic()
        self.last_wall_time = self.start_wall_time
        self.accumulator = 0.0
        self.sim_time = 0.0
        self._last_ground_log = -1.0
        self._last_follow_update = -1.0
        self._closed = False

    def _load_colors(self):
        ranges_path = Path(__file__).parents[2] / "tools" / "color_ranges.json"
        data = json.loads(ranges_path.read_text())
        result = {"white": (0.95, 0.95, 0.95, 1.0), "black": (0.025, 0.025, 0.025, 1.0)}
        for name in ("RED", "GREEN", "ORANGE", "BLUE", "MAGENTA"):
            lo = np.array(data[f"LOWER_{name}_HSV"], dtype=np.uint8)
            hi = np.array(data[f"UPPER_{name}_HSV"], dtype=np.uint8)
            hsv = ((lo.astype(int) + hi.astype(int)) // 2).astype(np.uint8).reshape(1, 1, 3)
            expected_rgb = cv2.cvtColor(hsv, cv2.COLOR_HSV2RGB)[0, 0]
            rendered_rgb = expected_rgb[::-1] / 255.0
            result[name.lower()] = (*rendered_rgb.tolist(), 1.0)
        return result

    def _box(self, half_extents, position, color, collision=True, yaw=0.0):
        collision_id = self.p.createCollisionShape(self.p.GEOM_BOX, halfExtents=half_extents, physicsClientId=self.client_id) if collision else -1
        visual_id = self.p.createVisualShape(self.p.GEOM_BOX, halfExtents=half_extents, rgbaColor=color, physicsClientId=self.client_id)
        return self.p.createMultiBody(
            baseMass=0, baseCollisionShapeIndex=collision_id, baseVisualShapeIndex=visual_id,
            basePosition=position, baseOrientation=self.p.getQuaternionFromEuler((0, 0, yaw)),
            physicsClientId=self.client_id,
        )

    def _build_track(self):
        t = self.config.track
        self._box((1.8, 1.8, 0.015), (0, 0, -0.015), self.colors["white"])
        wall_half = t.wall_height / 2
        outer = t.outer_size / 2
        thick = t.wall_thickness / 2
        for position, extent in [
            ((0, outer, wall_half), (outer + thick, thick, wall_half)),
            ((0, -outer, wall_half), (outer + thick, thick, wall_half)),
            ((outer, 0, wall_half), (thick, outer + thick, wall_half)),
            ((-outer, 0, wall_half), (thick, outer + thick, wall_half)),
        ]:
            self._box(extent, position, self.colors["black"])
        island = t.island_size / 2
        for position, extent in [
            ((0, island, wall_half), (island, thick, wall_half)),
            ((0, -island, wall_half), (island, thick, wall_half)),
            ((island, 0, wall_half), (thick, island, wall_half)),
            ((-island, 0, wall_half), (thick, island, wall_half)),
        ]:
            self._box(extent, position, self.colors["black"])

        # Thin floor markers do not affect vehicle collisions.
        for x, y, yaw, name in [
            (-1.0, 0.55, math.radians(-35), "blue"), (-1.0, 0.62, math.radians(35), "orange"),
            (1.0, 0.55, math.radians(35), "blue"), (1.0, 0.62, math.radians(-35), "orange"),
            (-1.0, -0.55, math.radians(35), "blue"), (-1.0, -0.62, math.radians(-35), "orange"),
            (1.0, -0.55, math.radians(-35), "blue"), (1.0, -0.62, math.radians(35), "orange"),
        ]:
            self._box((0.38, 0.012, 0.002), (x, y, 0.003), self.colors[name], collision=False, yaw=yaw)
        bay_x = -1.05 if self.config.direction == "ccw" else 1.05
        self._box((0.28, 0.018, 0.003), (bay_x, -1.26, 0.004), self.colors["magenta"], collision=False)
        self._box((0.018, 0.19, 0.003), (bay_x - 0.28, -1.075, 0.004), self.colors["magenta"], collision=False)
        self._box((0.018, 0.19, 0.003), (bay_x + 0.28, -1.075, 0.004), self.colors["magenta"], collision=False)

        if self.mode == "OBSTACLE":
            defaults = [(-1.0, 0.85, "red"), (1.0, 0.85, "green"), (-1.0, -0.65, "green"), (1.0, -0.65, "red")]
            positions = [(x, y) for x, y, _ in defaults]
            for x, y, color in defaults:
                self._post(x, y, color)
            self._add_random_posts(positions)

    def _post(self, x, y, color):
        t = self.config.track
        collision = self.p.createCollisionShape(self.p.GEOM_CYLINDER, radius=t.post_radius, height=t.post_height, physicsClientId=self.client_id)
        visual = self.p.createVisualShape(self.p.GEOM_CYLINDER, radius=t.post_radius, length=t.post_height, rgbaColor=self.colors[color], physicsClientId=self.client_id)
        self.p.createMultiBody(0, collision, visual, (x, y, t.post_height / 2), physicsClientId=self.client_id)

    def _add_random_posts(self, occupied):
        candidates = []
        for _ in range(500):
            side = self._rng.randrange(4)
            along = self._rng.uniform(-1.18, 1.18)
            across = self._rng.uniform(-1.12, -0.88)
            if side == 0: point = (along, -across)
            elif side == 1: point = (-across, along)
            elif side == 2: point = (along, across)
            else: point = (across, along)
            if min((point[0] - x) ** 2 + (point[1] - y) ** 2 for x, y in occupied) < 0.12:
                continue
            if point[1] < -1.0 and abs(point[0] - (-1.05 if self.config.direction == "ccw" else 1.05)) < 0.45:
                continue
            occupied.append(point)
            candidates.append(point)
            if len(candidates) >= self.config.random_posts:
                break
        for index, (x, y) in enumerate(candidates):
            self._post(x, y, "red" if index % 2 == 0 else "green")

    def _start_pose(self, mode, direction):
        if mode == "OBSTACLE":
            x = -1.05 if direction == "ccw" else 1.05
            yaw = 0.0 if direction == "ccw" else math.pi
            return (x, -1.16, 0.0), yaw
        return ((-1.0 if direction == "ccw" else 1.0), -1.0, 0.0), (0.0 if direction == "ccw" else math.pi)

    def advance(self):
        if self._closed:
            return
        now = time.monotonic()
        if self.ui is not None:
            if not self.p.isConnected(self.client_id):
                raise KeyboardInterrupt
            try:
                events = self.ui.poll()
            except self.p.error:
                # PyBullet's macOS GUI can briefly reject debug-parameter reads
                # while its Metal window is being initialized. Only stop after the
                # physics client has actually disconnected.
                if not self.p.isConnected(self.client_id):
                    raise KeyboardInterrupt
                events = set()
            if "restart" in events:
                raise SimulatorRestartRequested
            if "stop" in events:
                self.running = False
                self.vehicle.command(0.0, 95.0)
                self._events.add("stop")
            if "start" in events:
                self.running = True
                self._events.add("start")
                print("START / RESUME pressed - simulator running.")
        elapsed = min(now - self.last_wall_time, 0.25)
        self.last_wall_time = now
        self.accumulator += max(0.0, elapsed)
        dt = 1.0 / self.config.physics_hz
        while self.accumulator >= dt:
            self.p.stepSimulation(physicsClientId=self.client_id)
            self.accumulator -= dt
            self.sim_time += dt
        position, yaw = self.vehicle.pose()
        if self.sim_time - self._last_ground_log >= 0.04:
            self._ground_writer.writerow((f"{self.sim_time:.4f}", *[f"{v:.6f}" for v in position[:2]], f"{math.degrees(yaw):.4f}"))
            self._last_ground_log = self.sim_time
        if self.config.gui and self.sim_time - self._last_follow_update > 0.1:
            self.p.resetDebugVisualizerCamera(2.3, 42, -48, position, physicsClientId=self.client_id)
            self._last_follow_update = self.sim_time
        if self.config.duration_seconds is not None and now - self.start_wall_time >= self.config.duration_seconds:
            raise KeyboardInterrupt

    def consume_event(self, name: str) -> bool:
        if name not in self._events:
            return False
        self._events.remove(name)
        return True

    def close(self):
        if self._closed:
            return
        self._closed = True
        if self.ui is not None:
            self.ui.save()
        self._ground_file.close()
        if self.p.isConnected(self.client_id):
            self.p.disconnect(self.client_id)


def _as_json(value):
    if hasattr(value, "__dataclass_fields__"):
        return {name: _as_json(getattr(value, name)) for name in value.__dataclass_fields__}
    return value
