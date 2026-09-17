"""Independent vehicle and track profiles, shared by editors and runtime."""
from __future__ import annotations

import ast
import copy
import json
import math
from dataclasses import asdict, fields
from pathlib import Path

ROOT = Path(__file__).resolve().parent
ROI_NAMES = {"left": "LEFT_REGION", "right": "RIGHT_REGION", "lap": "LAP_REGION",
             "obstacle": "OBS_REGION", "reverse": "REVERSE_REGION",
             "front_wall": "FRONT_WALL_REGION", "parking": "PARKING_LOT_REGION"}


def default_path(kind, mode):
    return ROOT / "profiles" / f"{kind}-{mode.lower()}.json"


def standard_rois(mode):
    """Read calibration constants without executing a hardware entry point."""
    filename = "obstacle_challenge.py" if mode == "OBSTACLE" else "open_challenge.py"
    constants = {}
    for node in ast.parse((ROOT.parent / filename).read_text()).body:
        if isinstance(node, ast.Assign) and isinstance(node.targets[0], ast.Name):
            expression = copy.deepcopy(node.value)
            class Resolve(ast.NodeTransformer):
                def visit_Name(self, item):
                    if item.id in constants:
                        return ast.parse(repr(constants[item.id]), mode="eval").body
                    return item
                def visit_IfExp(self, item):
                    test = self.visit(item.test)
                    if isinstance(test, ast.Compare) and len(test.ops) == 1 and isinstance(test.ops[0], ast.Eq):
                        match = ast.literal_eval(test.left) == ast.literal_eval(test.comparators[0])
                        return self.visit(item.body if match else item.orelse)
                    return item
                def visit_Subscript(self, item):
                    value = ast.literal_eval(self.visit(item.value))
                    index = ast.literal_eval(item.slice)
                    return ast.parse(repr(value[index]), mode="eval").body
            try:
                constants[node.targets[0].id] = ast.literal_eval(Resolve().visit(expression))
            except (ValueError, TypeError, SyntaxError, KeyError, IndexError):
                pass
    result = {key: list(constants[name]) for key, name in ROI_NAMES.items()}
    result["left_danger"], result["right_danger"] = constants["DANGER_ZONE_POINTS"]
    return result


def standard_config(mode):
    from .config import SimConfig
    config = SimConfig(gui=False, rois=standard_rois(mode))
    config.noise.encoder_scale = 1.0
    config.camera.render_scale = 1.0
    config.camera.horizontal_fov_deg = 110.0
    config.camera.pitch_deg = -5.0
    config.camera.mount_height = 0.08
    config.camera.mount_forward = 0.16
    config.camera.barrel_k1 = 0.0
    config.rois["reverse"] = [263, 340, 377, 365]
    config.vehicle.track_width = 0.122
    if mode == "OBSTACLE":
        config.vehicle.max_steering_deg = 25.0
    return config


def payload(config, kind):
    groups = ("vehicle", "camera", "noise") if kind == "vehicle" else ("track", "odometry", "parking")
    data = {"version": 2, "kind": kind}
    data.update({group: asdict(getattr(config, group)) for group in groups})
    if kind == "vehicle":
        data["rois"] = copy.deepcopy(config.rois)
    else:
        data.update(direction=config.direction, seed=config.seed, random_posts=config.random_posts)
    return data


def apply(config, data, kind):
    if not isinstance(data, dict) or data.get("kind") != kind:
        raise ValueError(f"Expected a {kind} profile")
    # Old track profiles have no odometry section: restore original defaults.
    if kind == "track" and "odometry" not in data:
        from .config import OdometryConfig
        config.odometry = OdometryConfig()
    if kind == "track" and "parking" not in data:
        from .config import ParkingConfig
        config.parking = ParkingConfig()
    for group in (("vehicle", "camera", "noise") if kind == "vehicle" else ("track", "odometry", "parking")):
        target = getattr(config, group)
        allowed = {f.name for f in fields(target)}
        for key, value in data.get(group, {}).items():
            if key not in allowed:
                raise ValueError(f"Unknown setting: {group}.{key}")
            if group != "camera" or key not in ("width", "height", "output_bgr"):
                setattr(target, key, value)
    if kind == "vehicle":
        for name, value in data.get("rois", {}).items():
            if name in config.rois:
                target = config.rois[name]
                if isinstance(target, list):
                    target[:] = value
                else:
                    target.update(value)
    else:
        for key in ("direction", "seed", "random_posts"):
            if key in data:
                setattr(config, key, data[key])


def apply_profiles(config, required_vehicle=False, required_track=False):
    for kind, required in (("vehicle", required_vehicle), ("track", required_track)):
        path = Path(getattr(config, f"{kind}_profile"))
        if path.is_file():
            apply(config, json.loads(path.read_text()), kind)
        elif required:
            raise FileNotFoundError(path)
    validate(config)


def save(config, kind, path):
    validate(config)
    path = Path(path).expanduser().resolve()
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(".json.tmp")
    temporary.write_text(json.dumps(payload(config, kind), indent=2) + "\n")
    temporary.replace(path)


def validate(c):
    for group in (c.vehicle, c.camera, c.noise, c.track, c.odometry, c.parking):
        for value in asdict(group).values():
            if isinstance(value, (int, float)) and not math.isfinite(value):
                raise ValueError("All numeric settings must be finite")
    o = c.odometry
    for name in ("start_zone_half_width", "start_zone_half_height", "lap_interval_seconds",
                 "estimated_wheel_radius", "estimated_ticks_per_rev", "estimated_gear_ratio"):
        if getattr(o, name) <= 0:
            raise ValueError(f"Odometry {name} must be positive")
    if int(o.estimated_ticks_per_rev) != o.estimated_ticks_per_rev:
        raise ValueError("Odometry ticks per revolution must be an integer")
    if not 0 <= o.drift_alpha <= 1 or o.max_drift_threshold < 0:
        raise ValueError("Odometry drift alpha must be 0..1 and threshold nonnegative")
    v, t = c.vehicle, c.track
    for name in ("length", "width", "wheel_radius", "wheelbase", "track_width",
                 "mass", "drive_force", "servo_span_deg", "steering_rate", "steering_force"):
        if getattr(v, name) <= 0:
            raise ValueError(f"Vehicle {name} must be positive")
    if v.mass <= 0.09 or v.wheelbase >= v.length or v.track_width <= v.width:
        raise ValueError("Mass must exceed 0.09 kg; wheelbase < length; track width > chassis width")
    if not 0 < v.max_speed_mps <= 3 or not 0 < v.max_steering_deg <= 45:
        raise ValueError("Speed must be 0..3 m/s and steering 0..45 degrees")
    if v.ticks_per_revolution < 1 or v.tire_friction < 0:
        raise ValueError("Invalid encoder ticks or tire friction")
    if not 5 <= c.camera.fps <= 120 or not 30 <= c.camera.horizontal_fov_deg <= 170:
        raise ValueError("Camera FPS must be 5..120 and FOV 30..170")
    if not 0.25 <= c.camera.render_scale <= 1 or c.camera.near <= 0 or c.camera.far <= c.camera.near:
        raise ValueError("Invalid camera scale or clipping distances")
    if c.camera.mount_height <= 0 or c.camera.exposure_gain <= 0:
        raise ValueError("Camera height and exposure must be positive")
    if c.noise.encoder_scale <= 0 or c.noise.telemetry_hz <= 0 or c.noise.watchdog_seconds <= 0:
        raise ValueError("Encoder scale, telemetry rate and watchdog must be positive")
    if any(getattr(c.noise, k) < 0 for k in ("encoder_noise_std_ticks", "gyro_noise_std_deg", "gyro_bias_walk_std_deg_sqrt_s")):
        raise ValueError("Noise deviations cannot be negative")
    if t.shape not in ("square", "rectangle") or not 0.6 <= t.length_scale <= 2:
        raise ValueError("Track shape must be square or rectangle; length scale 0.6..2")
    if t.outer_size < 2 or not 0 < t.island_size < t.outer_size - 0.8:
        raise ValueError("Track must be at least 2 m with at least 0.4 m lanes")
    scale = t.outer_size / 3
    if not c.parking.custom and t.island_size / 2 + v.width / 2 + .1 >= scale:
        raise ValueError("Island leaves insufficient clearance for the standard vehicle start")
    if min(t.wall_height, t.wall_thickness, t.post_radius, t.post_height) <= 0:
        raise ValueError("Track geometry dimensions must be positive")
    if not 0 <= t.ambient <= 1 or not 0 <= t.diffuse <= 1 or t.light_z <= 0:
        raise ValueError("Lighting coefficients must be 0..1 and light height positive")
    if c.direction not in ("cw", "ccw") or not 0 <= c.random_posts <= 30:
        raise ValueError("Invalid direction or random post count (0..30)")
    for name, region in c.rois.items():
        values = list(region.values()) if isinstance(region, dict) else region
        if len(values) != 4 or any(not isinstance(n, int) for n in values):
            raise ValueError(f"ROI {name} needs four integer coordinates")
        x1, y1, x2, y2 = values
        if not (0 <= x1 <= c.camera.width and 0 <= x2 <= c.camera.width and
                0 <= y1 <= c.camera.height and 0 <= y2 <= c.camera.height):
            raise ValueError(f"ROI {name} is outside the image")
        if isinstance(region, list) and not (x1 < x2 and y1 < y2):
            raise ValueError(f"ROI {name} must have x1 < x2 and y1 < y2")
    if c.parking.width <= .036 or c.parking.depth <= .036:
        raise ValueError("Parking width and depth must exceed 0.036 m")
    if c.parking.custom:
        x, y, width, depth, yaw = parking_geometry(c)
        _validate_lane_box(c, x, y, width + .036, depth + .036, yaw, "Parking bay")
        pos, heading = start_pose(c, "OBSTACLE")
        _validate_lane_box(c, pos[0], pos[1], v.length + .04,
                           v.track_width + .04, heading, "Vehicle start")
    positions = []
    for post in t.posts:
        if len(post) != 3 or post[2] not in ("red", "green") or not valid_post(c, *post[:2], positions):
            raise ValueError(f"Post overlaps a wall, vehicle start, parking bay or another post: {post}")
        positions.append(post[:2])


def track_scale(c):
    return c.track.length_scale if c.track.shape == "rectangle" else 1.0


def valid_post(c, x, y, occupied=()):
    t = c.track
    sy = track_scale(c)
    margin = t.post_radius + t.wall_thickness / 2 + 0.03
    if not math.isfinite(x) or not math.isfinite(y):
        return False
    if abs(x) >= t.outer_size / 2 - margin or abs(y) >= t.outer_size * sy / 2 - margin:
        return False
    if abs(x) <= t.island_size / 2 + margin and abs(y) <= t.island_size * sy / 2 + margin:
        return False
    if c.parking.custom:
        bx, by, width, depth, yaw = parking_geometry(c)
        dx, dy = x - bx, y - by
        lx = math.cos(yaw) * dx + math.sin(yaw) * dy
        ly = -math.sin(yaw) * dx + math.cos(yaw) * dy
        if abs(lx) < width / 2 + margin and abs(ly) < depth / 2 + margin:
            return False
        pos, _ = start_pose(c, "OBSTACLE")
        if math.hypot(x-pos[0], y-pos[1]) < math.hypot(c.vehicle.length, c.vehicle.track_width)/2 + margin:
            return False
        return all(math.hypot(x-px, y-py) > 2*t.post_radius + .06 for px, py in occupied)
    # Reserve the complete start/parking rectangle on the bottom lane.
    scale = t.outer_size / 3
    bay_x = (-1.05 if c.direction == "ccw" else 1.05) * scale
    if abs(x - bay_x) < 0.45 * scale + t.post_radius and y < -0.8 * scale * sy:
        return False
    return all(math.hypot(x - px, y - py) > 2 * t.post_radius + 0.06 for px, py in occupied)


def parking_geometry(c):
    """Bay centre, full dimensions and yaw, all in world metres/radians."""
    p = c.parking
    if p.custom:
        return p.center_x, p.center_y, p.width, p.depth, math.radians(p.rotation_deg)
    scale, sy = c.track.outer_size / 3, track_scale(c)
    return ((-1.05 if c.direction == "ccw" else 1.05)*scale,
            -1.075*scale*sy, .56*scale, .38*scale*sy, 0.0)


def start_pose(c, mode):
    if c.parking.custom:
        p = c.parking
        x, y, _, _, yaw = parking_geometry(c)
        return ((x + math.cos(yaw)*p.start_offset_x - math.sin(yaw)*p.start_offset_y,
                 y + math.sin(yaw)*p.start_offset_x + math.cos(yaw)*p.start_offset_y, 0.0),
                yaw + math.radians(p.heading_deg))
    scale, sy = c.track.outer_size/3, track_scale(c)
    x, y = (1.05, -1.16) if mode == "OBSTACLE" else (1.0, -1.0)
    return ((-x if c.direction == "ccw" else x)*scale, y*scale*sy, 0.0), (0.0 if c.direction == "ccw" else math.pi)


def _validate_lane_box(c, x, y, width, depth, yaw, label):
    # Conservative axis-aligned envelope guarantees clearance even when rotated.
    hx = abs(math.cos(yaw))*width/2 + abs(math.sin(yaw))*depth/2
    hy = abs(math.sin(yaw))*width/2 + abs(math.cos(yaw))*depth/2
    t, sy = c.track, track_scale(c)
    margin = t.wall_thickness/2 + .01
    if (abs(x)+hx >= t.outer_size/2-margin or
        abs(y)+hy >= t.outer_size*sy/2-margin or
        (abs(x)-hx <= t.island_size/2+margin and abs(y)-hy <= t.island_size*sy/2+margin)):
        raise ValueError(f"{label} overlaps a wall or island (conservative rotated bounds)")
