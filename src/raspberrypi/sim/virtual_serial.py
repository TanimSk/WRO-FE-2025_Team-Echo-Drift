"""pyserial-compatible Arduino protocol adapter for the simulated vehicle."""

from __future__ import annotations

import csv
import math
import random
import threading
import time
from collections import deque


class VirtualSerial:
    def __init__(self, world, noise, vehicle_config, seed: int):
        self.world = world
        self.noise = noise
        self.vehicle_config = vehicle_config
        self._rng = random.Random(seed)
        self._lines = deque([b"START\n"] if world.running else [])
        self._start_sent = world.running
        self._lock = threading.Lock()
        self._closed = False
        self._speed = 0.0
        self._angle = 95.0
        self._step_target = None
        self._encoder_origin_revs = world.vehicle.rear_wheel_revolutions()
        self._last_telemetry = -1.0
        self._last_sensor_time = world.sim_time
        self._last_yaw = world.vehicle.pose()[1]
        self._unwrapped_heading = 0.0
        self._gyro_bias = 0.0
        self._last_command_wall = time.monotonic()
        self._received_command = False
        self._sensor_file = (world.output_directory / "sensor.csv").open("w", newline="")
        self._sensor_writer = csv.writer(self._sensor_file)
        self._sensor_writer.writerow(("time_s", "encoder_ticks", "gyro_deg"))
        self._command_file = (world.output_directory / "commands.csv").open("w", newline="")
        self._command_writer = csv.writer(self._command_file)
        self._command_writer.writerow(("time_s", "speed_percent", "steps", "servo_angle"))

    @property
    def in_waiting(self) -> int:
        self._update()
        with self._lock:
            return sum(len(line) for line in self._lines)

    def write(self, data: bytes) -> int:
        self._update()
        try:
            text = data.decode("ascii").strip()
            fields = text.split(",")
            if len(fields) != 3:
                raise ValueError
            speed = float(fields[0])
            steps = int(float(fields[1]))
            angle = max(20.0, min(170.0, float(fields[2])))
        except (UnicodeDecodeError, ValueError):
            return len(data)

        self._speed, self._angle = speed, angle
        self._last_command_wall = time.monotonic()
        self._received_command = True
        self._command_writer.writerow((f"{self.world.sim_time:.4f}", f"{speed:.3f}", steps, f"{angle:.2f}"))
        self._command_file.flush()
        if steps > 0:
            self._encoder_origin_revs = self.world.vehicle.rear_wheel_revolutions()
            direction = 1 if speed >= 0 else -1
            self._step_target = direction * steps
        else:
            self._step_target = None
        if self.world.running:
            self.world.vehicle.command(speed, angle)
        return len(data)

    def readline(self) -> bytes:
        self._update()
        with self._lock:
            return self._lines.popleft() if self._lines else b""

    def _update(self):
        if self._closed:
            return
        self.world.advance()
        if self.world.consume_event("start"):
            if not self._start_sent:
                # Start must take priority over any telemetry accumulated while
                # the simulator was paused.
                self._enqueue(b"START\n", priority=True)
                self._start_sent = True
        if self.world.consume_event("stop"):
            self._speed = 0.0
            self._step_target = None
        current_revs = self.world.vehicle.rear_wheel_revolutions()
        true_ticks = int(round((current_revs - self._encoder_origin_revs) * self.vehicle_config.ticks_per_revolution))

        _, yaw = self.world.vehicle.pose()
        delta = (yaw - self._last_yaw + math.pi) % (2 * math.pi) - math.pi
        self._unwrapped_heading += math.degrees(delta)
        self._last_yaw = yaw
        dt = max(0.0, self.world.sim_time - self._last_sensor_time)
        self._last_sensor_time = self.world.sim_time
        if dt:
            self._gyro_bias += self._rng.gauss(0.0, self.noise.gyro_bias_walk_std_deg_sqrt_s * math.sqrt(dt))

        if self._step_target is not None:
            reached = true_ticks >= self._step_target if self._step_target > 0 else true_ticks <= self._step_target
            if reached:
                self.world.vehicle.command(0.0, self._angle)
                self._speed = 0.0
                self._step_target = None
                self._enqueue(b"DONE\n")

        interval = 1.0 / self.noise.telemetry_hz
        if self.world.sim_time - self._last_telemetry >= interval:
            self._last_telemetry = self.world.sim_time
            measured_ticks = int(round(true_ticks * self.noise.encoder_scale + self._rng.gauss(0.0, self.noise.encoder_noise_std_ticks)))
            raw_ticks = -measured_ticks
            gyro = self._unwrapped_heading + self._gyro_bias + self._rng.gauss(0.0, self.noise.gyro_noise_std_deg)
            line = f"{raw_ticks},{gyro:.2f}\n".encode("ascii")
            self._enqueue(line)
            self._sensor_writer.writerow((f"{self.world.sim_time:.4f}", raw_ticks, f"{gyro:.4f}"))
            self._sensor_file.flush()

        if self.world.running and self._received_command and self._step_target is None and self._speed != 0 and time.monotonic() - self._last_command_wall > self.noise.watchdog_seconds:
            self._speed = 0.0
            self._angle = 95.0
            self.world.vehicle.command(0.0, 95.0)
            self._enqueue(b"No serial input - resetting to 95 degrees\n")

    def _enqueue(self, line: bytes, priority: bool = False):
        with self._lock:
            if priority:
                self._lines.appendleft(line)
            else:
                self._lines.append(line)

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        if self.world.p.isConnected(self.world.client_id):
            self.world.vehicle.command(0.0, self.vehicle_config.servo_center_deg)
        self._sensor_file.close()
        self._command_file.close()
        self.world.close()
