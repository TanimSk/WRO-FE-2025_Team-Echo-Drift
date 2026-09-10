"""Rear-wheel-drive, front-steered PyBullet vehicle."""

from __future__ import annotations

import math
from pathlib import Path


class SimVehicle:
    def __init__(self, bullet, client_id: int, config, start_position, start_yaw: float):
        self.p = bullet
        self.client_id = client_id
        self.config = config
        urdf = Path(__file__).with_name("assets") / "vehicle.urdf"
        self.body_id = self.p.loadURDF(
            str(urdf), start_position,
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
            self.p.changeDynamics(self.body_id, joint, lateralFriction=1.2, rollingFriction=0.002, physicsClientId=client_id)
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

        normalized = max(-1.0, min(1.0, (servo_angle - 95.0) / 55.0))
        center = -normalized * math.radians(self.config.max_steering_deg)
        left, right = self._ackermann(center)
        for joint, target in zip(self.steer_joints, (left, right)):
            self.p.setJointMotorControl2(
                self.body_id, joint, self.p.POSITION_CONTROL,
                targetPosition=target, force=3.0, maxVelocity=5.0,
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
