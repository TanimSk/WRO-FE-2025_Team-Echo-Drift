"""Rear-wheel-drive, front-steered PyBullet vehicle."""

from __future__ import annotations

import math
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path


class SimVehicle:
    def __init__(self, bullet, client_id: int, config, start_position, start_yaw: float):
        self.p = bullet
        self.client_id = client_id
        self.config = config
        urdf = Path(__file__).with_name("assets") / "vehicle.urdf"
        model = ET.parse(urdf)
        root = model.getroot()
        base = root.find("./link[@name='base_link']")
        mass = config.mass - 0.09  # four wheels and two knuckles
        base.find("./inertial/mass").set("value", str(mass))
        inertia = base.find("./inertial/inertia")
        for name, value in zip(("ixx", "iyy", "izz"), (
            mass * (config.width**2 + 0.07**2) / 12,
            mass * (config.length**2 + 0.07**2) / 12,
            mass * (config.length**2 + config.width**2) / 12,
        )):
            inertia.set(name, str(value))
        for box in base.findall(".//box"):
            box.set("size", f"{config.length} {config.width} 0.07")
        for origin in base.findall("./*/origin"):
            origin.set("xyz", f"0 0 {config.wheel_radius + 0.019}")
        for cylinder in root.findall(".//cylinder"):
            cylinder.set("radius", str(config.wheel_radius))
        for link in root.findall("./link"):
            if link.get("name", "").endswith("_wheel"):
                tensor = link.find("./inertial/inertia")
                transverse = 0.02 * (3 * config.wheel_radius**2 + 0.018**2) / 12
                for key, value in (("ixx", transverse), ("izz", transverse),
                                   ("iyy", 0.01 * config.wheel_radius**2)):
                    tensor.set(key, str(value))
        for joint in root.findall("./joint"):
            origin = joint.find("origin")
            if origin is not None:
                name = joint.get("name")
                x = config.wheelbase / 2 * (1 if name.startswith("front") else -1)
                y = config.track_width / 2 * (1 if "left" in name else -1)
                origin.set("xyz", f"{x} {y} {config.wheel_radius}")
            limit = joint.find("limit")
            if limit is not None:
                limit.set("lower", "-1.2")
                limit.set("upper", "1.2")
        with tempfile.NamedTemporaryFile(suffix=".urdf") as temporary:
            model.write(temporary.name)
            self.body_id = self.p.loadURDF(
                temporary.name, start_position,
                self.p.getQuaternionFromEuler((0, 0, start_yaw)),
                useFixedBase=False, physicsClientId=client_id,
            )
        self.joints = {}
        for index in range(self.p.getNumJoints(self.body_id, physicsClientId=client_id)):
            name = self.p.getJointInfo(self.body_id, index, physicsClientId=client_id)[1].decode()
            self.joints[name] = index

        self.rear_joints = [self.joints["rear_left_wheel_joint"], self.joints["rear_right_wheel_joint"]]
        self.front_wheel_joints = [self.joints["front_left_wheel_joint"], self.joints["front_right_wheel_joint"]]
        self.steer_joints = [self.joints["front_left_steer_joint"], self.joints["front_right_steer_joint"]]
        for joint in self.rear_joints + self.front_wheel_joints:
            self.p.setJointMotorControl2(self.body_id, joint, self.p.VELOCITY_CONTROL, force=0, physicsClientId=client_id)
            self.p.changeDynamics(self.body_id, joint, lateralFriction=config.tire_friction, rollingFriction=0.002, physicsClientId=client_id)
        self.command(0.0, 95.0)

    def command(self, speed_percent: float, servo_angle: float) -> None:
        speed_percent = max(-100.0, min(100.0, speed_percent))
        wheel_velocity = (speed_percent / 100.0) * self.config.max_speed_mps / self.config.wheel_radius
        for joint in self.rear_joints:
            self.p.setJointMotorControl2(
                self.body_id, joint, self.p.VELOCITY_CONTROL,
                targetVelocity=wheel_velocity, force=self.config.drive_force,
                physicsClientId=self.client_id,
            )

        normalized = max(-1.0, min(1.0, (servo_angle - self.config.servo_center_deg) / self.config.servo_span_deg))
        center = -normalized * math.radians(self.config.max_steering_deg)
        left, right = self._ackermann(center)
        for joint, target in zip(self.steer_joints, (left, right)):
            self.p.setJointMotorControl2(
                self.body_id, joint, self.p.POSITION_CONTROL,
                targetPosition=target, force=self.config.steering_force, maxVelocity=self.config.steering_rate,
                physicsClientId=self.client_id,
            )

    def _ackermann(self, center: float) -> tuple[float, float]:
        if abs(center) < 1e-5:
            return 0.0, 0.0
        radius = self.config.wheelbase / math.tan(abs(center))
        inner = math.atan(self.config.wheelbase / max(0.02, radius - self.config.track_width / 2))
        outer = math.atan(self.config.wheelbase / (radius + self.config.track_width / 2))
        if center > 0:
            return inner, outer
        return -outer, -inner

    def rear_wheel_revolutions(self) -> float:
        angles = [self.p.getJointState(self.body_id, j, physicsClientId=self.client_id)[0] for j in self.rear_joints]
        return sum(angles) / (len(angles) * 2.0 * math.pi)

    def pose(self):
        position, orientation = self.p.getBasePositionAndOrientation(self.body_id, physicsClientId=self.client_id)
        yaw = self.p.getEulerFromQuaternion(orientation)[2]
        return position, yaw
