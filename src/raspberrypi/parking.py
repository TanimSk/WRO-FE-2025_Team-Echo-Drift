import cv2
from serial import Serial
from contour_workers import ContourResult
from img_processing_functions import (
    get_min_x_coord,
    get_max_x_coord,
    get_overall_centroid,
)
from simple_pid import PID
from collections import deque
import numpy as np
import time


class Parking:
    def __init__(
        self,
        arduino: Serial,
        parking_speed: int,
        camera_width: int,
        camera_height: int,
        parking_lot_region: list,
        maxLeft: int,
        maxRight: int,
        STRAIGHT_CONST: int,
        MAX_OFFSET_DEGREE: int,
        REVERSE_REGION: list,
    ):
        self.arduino = arduino
        self.parking_speed = parking_speed
        self.camera_width = camera_width
        self.camera_height = camera_height
        self.parking_lot_region = parking_lot_region

        # steering constraints
        self.maxLeft = maxLeft
        self.maxRight = maxRight
        self.STRAIGHT_CONST = STRAIGHT_CONST
        self.MAX_OFFSET_DEGREE = MAX_OFFSET_DEGREE

        # states
        self.has_parked_out = False
        # self.last_wall_count = 0
        # self.stop_tolerance = 5

        self.seen_parking_lot = "NOT_SEEN"
        self.parking_lot_side = "left"
        self.last_seen_time = time.time()
        self.wait_after_seen = 0.6  # seconds

        self.SMOOTH_WINDOW = 3
        self.left_buf = deque(maxlen=self.SMOOTH_WINDOW)
        self.right_buf = deque(maxlen=self.SMOOTH_WINDOW)

        # reverse
        self.REVERSE_REGION = REVERSE_REGION
        self.reverse_start_time = 0
        self.reverse_duration = 0.2  # seconds
        self.is_reversing = False

        # parking out instructions
        self.parking_out_instructions = {
            # left -> exit to left
            # right -> exit to right
            "left": [
                # speed, steps, angle                
                (-self.parking_speed, 500, 160),
                (self.parking_speed, 2000, 25),
                (self.parking_speed, 700, 95),
                (self.parking_speed, 2600, 145),
                (-self.parking_speed, 1500, 65),
                (-self.parking_speed, 1500, 135),                
            ],
            "right": [
                # speed, steps, angle
                (-self.parking_speed, 500, 25),
                (self.parking_speed, 2000, 160),
                (self.parking_speed, 700, 95),
                (self.parking_speed, 2600, 45),
                (-self.parking_speed, 1500, 125),
                (-self.parking_speed, 1500, 65),                        
            ],
        }
        self.parking_out_odometry_history = []


        self.parking_in_instructions = {
            # right -> parking lot is on the right
            # left -> parking lot is on the left
            "left": [
                # speed, steps, angle
                (self.parking_speed, 1200, 95),
                (-self.parking_speed, 3500, 20),
                (-self.parking_speed, 1250, 95),
                (-self.parking_speed, 2300, 170),
                # (self.parking_speed, 800, 20),
                (self.parking_speed, 800, 95),
            ],
            "right": [
                # speed, steps, angle
                (self.parking_speed, 2200, 150),
                (self.parking_speed, 1000, 95),                
            ],
        }

    def process_parking_out(
        self, left_result: ContourResult, right_result: ContourResult
    ):
        left_area = 0
        right_area = 0
        avg_times = 5

        for _ in range(avg_times):
            if left_result and left_result.area > 800:
                left_area += left_result.area
            if right_result and right_result.area > 800:
                right_area += right_result.area
            time.sleep(0.01)

        left_area = left_area / avg_times
        right_area = right_area / avg_times

        if left_area > right_area:
            # exit to the right
            print(f"Parking Out | Left Area: {left_area}, Right Area: {right_area}")
            parking_out_instructions = self.parking_out_instructions["right"]
            self.parking_lot_side = "left" # parking lot is on the left
        else:
            # exit to the left
            print(f"Parking Out | Left Area: {left_area}, Right Area: {right_area}")
            parking_out_instructions = self.parking_out_instructions["left"]
            self.parking_lot_side = "right" # parking lot is on the right

        for speed, steps, angle in parking_out_instructions:
            print(f"Parking Out | Speed: {speed}, Steps: {steps}, Angle: {angle}")
            self.arduino.write(f"{speed},{steps},{angle}\n".encode())
            time.sleep(0.1)            

            # wait for arduino to respond
            while True:
                if self.arduino.in_waiting > 0:
                    line = self.arduino.readline().decode("utf-8").rstrip()
                    print(f"Arduino: {line}")
                    if "DONE" in line:
                        print("found DONE")
                        break
                    else:
                        parts = line.split(",")
                        if len(parts) == 2:
                            try:
                                encoder_ticks = int(parts[0])
                                gyro_angle = float(parts[1])
                                self.parking_out_odometry_history.append(
                                    (encoder_ticks, gyro_angle)
                                )
                            except ValueError:
                                pass
            
            # stop bot
            self.arduino.write(f"0,-1,95\n".encode())

                # if arduino.in_waiting > 0:
            #     line = arduino.readline().decode("utf-8").rstrip()
            #     print(f"Arduino: {line}")
            #     if not line == "START":
            #         startProcessing = True

    def process_parking(
        self,
        dir: str,
        # parking_result: ContourResult,
        # left_result: ContourResult,
        # right_result: ContourResult,        
        # pid: PID,
    ):
        # print(left_result.metadata)
        # print(right_result.metadata)

        # self.left_buf.append(left_result.area)
        # self.right_buf.append(right_result.area)
        # left_s = sum(self.left_buf) / len(self.left_buf)
        # right_s = sum(self.right_buf) / len(self.right_buf)
        # aDiff = right_s - left_s
        # aSum = left_s + right_s
        # error = aDiff / (aSum + 1e-6)  # normalized between roughly [-1,1]
        # normalized_angle_offset = pid(error)

        # angle = int(
        #     max(
        #         min(
        #             self.STRAIGHT_CONST
        #             + normalized_angle_offset * self.MAX_OFFSET_DEGREE,
        #             self.maxRight,
        #         ),
        #         self.maxLeft,
        #     )
        # )

        # self.arduino.write(f"{self.parking_speed},-1,{angle}\n".encode())

        # # # step 01
        # if parking_result and parking_result.area > 800:
        #     self.seen_parking_lot = "SEEN"
        #     self.last_seen_time = time.time()

        #     # determine which side the parking lot is on
        #     parking_x, _ = get_overall_centroid(parking_result.contours)
        #     if parking_x is not None:
        #         if parking_x < (self.camera_width // 2):
        #             self.parking_lot_side = "left"
        #         else:
        #             self.parking_lot_side = "right"

        #     print("seen parking lot")
        #     return angle

        # # step 02
        for speed, steps, angle in self.parking_in_instructions[dir]:
            self.arduino.write(f"{speed},{steps},{angle}\n".encode())
            time.sleep(0.1)
            print(f"Parking In | Speed: {speed}, Steps: {steps}, Angle: {angle}")

            # wait for arduino to respond
            while True:
                if self.arduino.in_waiting > 0:
                    line = self.arduino.readline().decode("utf-8").rstrip()
                    print(f"Arduino: {line}")
                    if "DONE" in line:
                        print("found DONE")
                        break

        #     return (None, None)
        #     print("Parking | No longer see parking lot, stopping")
        #     self.arduino.write(f"0,-1,{self.STRAIGHT_CONST}\n".encode())

        #     for speed, steps, angle in self.parking_in_instructions:
        #         self.arduino.write(f"{speed},{steps},{angle}\n".encode())
        #         time.sleep(0.1)
        #         print(f"Parking In | Speed: {speed}, Steps: {steps}, Angle: {angle}")

        #         # wait for arduino to respond
        #         while True:
        #             if self.arduino.in_waiting > 0:
        #                 line = self.arduino.readline().decode("utf-8").rstrip()
        #                 print(f"Arduino: {line}")
        #                 if "DONE" in line:
        #                     print("found DONE")
        #                     break

        # parking_walls = []
        # current_wall_count = 0
        # obstacle_wall_pivot = (None, None)

        # if parking_result and parking_result.area > 800:
        #     for contour in parking_result.contours:
        #         x, y, w, h = cv2.boundingRect(contour)
        #         area = w * h

        #         if area > 1500:
        #             current_wall_count += 1
        #             parking_walls.append((x, y, w, h))

        # if self.last_wall_count == 2 and current_wall_count < 2:
        #     self.is_reversing = True
        #     self.reverse_start_time = time.time()

        # if self.is_reversing:
        #     if (time.time() - self.reverse_start_time) < self.reverse_duration:
        #         self.arduino.write(
        #             f"{-self.parking_speed}, {self.STRAIGHT_CONST}\n".encode()
        #         )
        #         return parking_walls, 2, obstacle_wall_pivot
        #     else:
        #         self.is_reversing = False
        #         self.last_wall_count = 0

        # self.last_wall_count = current_wall_count
        # if current_wall_count == 2:
        #     wall_1_x, wall_1_y, wall_1_w, wall_1_h = parking_walls[0]
        #     wall_1_cx, wall_1_cy = wall_1_x + (wall_1_w // 2), wall_1_y + (
        #         wall_1_h // 2
        #     )
        #     wall_2_x, wall_2_y, wall_2_w, wall_2_h = parking_walls[1]
        #     wall_2_cx, wall_2_cy = wall_2_x + (wall_2_w // 2), wall_2_y + (
        #         wall_2_h // 2
        #     )

        #     # check if the centroid y are in the reverse region
        #     if (
        #         ((wall_1_y + self.parking_lot_region[1] + wall_1_h) > self.REVERSE_REGION[1])
        #         and ((wall_1_cx + self.parking_lot_region[0]) > self.REVERSE_REGION[0])
        #         and ((wall_1_cx + self.parking_lot_region[0]) < self.REVERSE_REGION[2])
        #     ) or (
        #         ((wall_2_y + self.parking_lot_region[1] + wall_2_h) > self.REVERSE_REGION[1])
        #         and ((wall_2_cx + self.parking_lot_region[0]) > self.REVERSE_REGION[0])
        #         and ((wall_2_cx + self.parking_lot_region[0]) < self.REVERSE_REGION[2])
        #     ):
        #         print("Entering Reverse Region")
        #         self.is_reversing = True
        #         self.reverse_start_time = time.time()
        #         return parking_walls, 2, obstacle_wall_pivot

        #     print(
        #         f"wall 1: X: {wall_1_cx + self.parking_lot_region[0]}, Y: {wall_1_cy + self.parking_lot_region[1] + wall_1_h} | REVERSE_REGION: {self.REVERSE_REGION}"
        #     )
        #     print(
        #         f"wall 2: X: {wall_2_cx + self.parking_lot_region[0]}, Y: {wall_2_cy + self.parking_lot_region[1] + wall_2_h} | REVERSE_REGION: {self.REVERSE_REGION}"
        #     )
        #     # print(
        #     #     f"wall 2: area: {wall_2_w * wall_2_h}, pos: ({wall_2_cx}, {wall_2_cy})"
        #     # )

        #     # transform to global coordinates
        #     wall_1_cx += self.parking_lot_region[0]
        #     wall_1_cy += self.parking_lot_region[1]
        #     wall_2_cx += self.parking_lot_region[0]
        #     wall_2_cy += self.parking_lot_region[1]

        #     # compute how far is the bot from the object and walls middle point
        #     offset_x = ((wall_1_cx + wall_2_cx) // 2) - (self.camera_width // 2)

        #     # show a circle dot on the middle of the object and wall
        #     obstacle_wall_pivot = (
        #         (wall_1_cx + wall_2_cx) // 2,
        #         (wall_1_cy + wall_2_cy) // 2,
        #     )

        #     obj_error = offset_x / (self.camera_width // 2)
        #     obj_error = -np.clip(obj_error * 10, -1, 1)
        #     normalized_angle_offset = pid(obj_error)

        #     # --- Map normalized control to servo angle ---
        #     angle = int(
        #         max(
        #             min(
        #                 self.STRAIGHT_CONST
        #                 + normalized_angle_offset * self.MAX_OFFSET_DEGREE,
        #                 self.maxRight,
        #             ),
        #             self.maxLeft,
        #         )
        #     )

        #     self.arduino.write(f"{self.parking_speed}, {angle}\n".encode())

        #     # if abs(offset_x) < self.stop_tolerance:
        #     #     print("Parking | Stopped")
        #     #     print(f"offset_x: {offset_x}, obj_error: {obj_error}, angle: {angle}")
        #     #     while True:
        #     #         self.arduino.write(f"0, {self.STRAIGHT_CONST}\n".encode())
        #     #         time.sleep(0.1)

        #     # print(f"Parking | Speed: {self.parking_speed}, Angle: {angle}")

        # return parking_walls, current_wall_count, obstacle_wall_pivot
