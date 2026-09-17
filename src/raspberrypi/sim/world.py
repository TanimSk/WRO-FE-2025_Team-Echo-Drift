"""Track construction, fixed-step physics, and run logging."""

from __future__ import annotations

import csv
import json
import math
import random
import time
import tempfile
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

from hardware import SimulatorRestartRequested
from .ui import SimulatorUI
from .vehicle import SimVehicle
from .profiles import track_scale, valid_post, parking_geometry, start_pose


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
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, int(config.legacy_controls), physicsClientId=self.client_id)
            p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, int(config.track.shadows), physicsClientId=self.client_id)

        stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        output_root.mkdir(parents=True, exist_ok=True)
        self.output_directory = Path(tempfile.mkdtemp(prefix=stamp + "_", dir=output_root))
        self._ground_file = (self.output_directory / "ground_truth.csv").open("w", newline="")
        self._ground_writer = csv.writer(self._ground_file)
        self._ground_writer.writerow(("time_s", "x_m", "y_m", "yaw_deg", "obstacle_contacts"))
        (self.output_directory / "run_config.json").write_text(json.dumps(_as_json(config), indent=2))

        self._rng = random.Random(config.seed)
        self.colors = self._load_colors()
        self.post_positions = []
        self._build_track()
        start, yaw = self._start_pose(mode, config.direction)
        self.vehicle = SimVehicle(p, self.client_id, config.vehicle, start, yaw)
        self.running = not config.legacy_controls
        self._events = set()
        self.ui = SimulatorUI(p, self.client_id, config) if config.gui and config.legacy_controls else None
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
        sy = track_scale(self.config)
        position = (position[0], position[1] * sy, position[2])
        half_extents = (half_extents[0], half_extents[1] * sy, half_extents[2])
        collision_id = self.p.createCollisionShape(self.p.GEOM_BOX, halfExtents=half_extents, physicsClientId=self.client_id) if collision else -1
        visual_id = self.p.createVisualShape(self.p.GEOM_BOX, halfExtents=half_extents, rgbaColor=color, physicsClientId=self.client_id)
        return self.p.createMultiBody(
            baseMass=0, baseCollisionShapeIndex=collision_id, baseVisualShapeIndex=visual_id,
            basePosition=position, baseOrientation=self.p.getQuaternionFromEuler((0, 0, yaw)),
            physicsClientId=self.client_id,
        )

    def _build_track(self):
        t = self.config.track
        self.floor_body = self._box((t.outer_size / 2 + .3, t.outer_size / 2 + .3, 0.015), (0, 0, -0.015), self.colors["white"])
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
            scale = t.outer_size / 3
            self._box((0.38 * scale, 0.012, 0.002), (x * scale, y * scale, 0.003), self.colors[name], collision=False, yaw=yaw)
        bx, by, width, depth, yaw = parking_geometry(self.config)
        sy = track_scale(self.config)
        self.parking_bodies = []
        for dx, dy, hx, hy in [(0, -depth/2, width/2, .018),
                              (-width/2, 0, .018, depth/2),
                              (width/2, 0, .018, depth/2)]:
            x = bx + math.cos(yaw)*dx - math.sin(yaw)*dy
            y = by + math.sin(yaw)*dx + math.cos(yaw)*dy
            self.parking_bodies.append(self._box((hx, hy/sy, .003), (x, y/sy, .004),
                                                 self.colors["magenta"], collision=False, yaw=yaw))

        if self.mode == "OBSTACLE":
            defaults = self.config.track.posts
            positions = [(x, y) for x, y, _ in defaults]
            for x, y, color in defaults:
                self._post(x, y, color)
            self._add_random_posts(positions)

    def _post(self, x, y, color):
        self.post_positions.append((x, y, color))
        t = self.config.track
        collision = self.p.createCollisionShape(self.p.GEOM_CYLINDER, radius=t.post_radius, height=t.post_height, physicsClientId=self.client_id)
        visual = self.p.createVisualShape(self.p.GEOM_CYLINDER, radius=t.post_radius, length=t.post_height, rgbaColor=self.colors[color], physicsClientId=self.client_id)
        self.p.createMultiBody(0, collision, visual, (x, y, t.post_height / 2), physicsClientId=self.client_id)

    def _add_random_posts(self, occupied):
        if self.config.random_posts <= 0:
            return
        candidates = []
        outer = self.config.track.outer_size / 2
        for _ in range(5000):
            point = (self._rng.uniform(-outer, outer), self._rng.uniform(-outer, outer) * track_scale(self.config))
            if not valid_post(self.config, *point, occupied):
                continue
            occupied.append(point)
            candidates.append(point)
            if len(candidates) >= self.config.random_posts:
                break
        for index, (x, y) in enumerate(candidates):
            self._post(x, y, "red" if index % 2 == 0 else "green")
        if len(candidates) != self.config.random_posts:
            raise ValueError("Not enough free lane space for requested random posts")

    def _start_pose(self, mode, direction):
        return start_pose(self.config, mode)

    def advance(self):
        if self._closed:
            return
        now = time.monotonic()
        if self.config.gui and not self.p.isConnected(self.client_id):
            raise KeyboardInterrupt
        if self.config.gui and not self.config.legacy_controls:
            keys = self.p.getKeyboardEvents(physicsClientId=self.client_id)
            if keys.get(ord('r'), 0) & self.p.KEY_WAS_TRIGGERED:
                raise SimulatorRestartRequested
            if keys.get(ord(' '), 0) & self.p.KEY_WAS_TRIGGERED:
                self.running = not self.running
                self._events.add("start" if self.running else "stop")
                if not self.running:
                    self.vehicle.command(0, self.config.vehicle.servo_center_deg)
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
            contacts = self.p.getContactPoints(bodyA=self.vehicle.body_id, physicsClientId=self.client_id)
            obstacles = sum(contact[2] not in (self.floor_body, self.vehicle.body_id) for contact in contacts)
            self._ground_writer.writerow((f"{self.sim_time:.4f}", *[f"{v:.6f}" for v in position[:2]], f"{math.degrees(yaw):.4f}", obstacles))
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
