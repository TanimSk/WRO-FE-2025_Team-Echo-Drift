import os
import json
import sys
import time
import cv2
import numpy as np
import threading
import time
from collections import deque
from img_processing_functions import (
    display_roi,
    get_max_y_coord,
    get_min_x_coord,
    display_debug_screen,
    get_max_x_coord,
    get_overall_centroid,
    point_position,
)
from contour_workers import ContourWorkers
from parking import Parking
from simple_pid import PID
import copy
from odometry import OdometryTracker, OdometryVisualizer, clamp_angle
import math
from hardware import create_hardware, run_entrypoint

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

# debug flag parsing
DEBUG = "--debug" in sys.argv[1:]
SIMULATED = "--sim" in sys.argv[1:]

# DEBUG = True
print("-- DEBUG MODE --" if DEBUG else "-- PRODUCTION --")

# Simulated camera settings
MODE = "NO_OBSTACLE"
CAM_WIDTH = 640
CAM_HEIGHT = 480
MAX_SPEED = 100
MIN_SPEED = 90

# Intersections
TOTAL_LAPS = 3

# Region of Interest coordinates
LEFT_REGION = [0, 165, 230, 230]  # left
RIGHT_REGION = [410, 165, 640, 230]  # right
LAP_REGION = [215, 260, 415, 305]  # lap detection
OBS_REGION = [77, 110, 563, 355]  # obstacle detection
REVERSE_REGION = [223, 255, 427, 273]  # reverse trigger area
FRONT_WALL_REGION = [300, 160, 340, 180]  # front wall detection
PARKING_LOT_REGION = [0, 185, CAM_WIDTH, 400]  # parking lot detection
# DANGER_ZONE_POINTS = [175, OBS_REGION[1], 465, OBS_REGION[3]]  # area to check for obstacles
DANGER_ZONE_POINTS = [
    {
        "x1": 207,
        "y1": OBS_REGION[1],
        "x2": 77,
        "y2": OBS_REGION[3],
    },
    {
        "x1": 423,
        "y1": OBS_REGION[1],
        "x2": 563,
        "y2": OBS_REGION[3],
    },
]

BLACK_WALL_DETECTOR_AREA = (LEFT_REGION[2] - LEFT_REGION[0]) * (
    LEFT_REGION[3] - LEFT_REGION[1]
)
# OBSTACLE_DETECTOR_X = OBS_REGION[2] - OBS_REGION[0]
# OBSTACLE_DETECTOR_Y = OBS_REGION[3] - OBS_REGION[1]
obstacle_wall_pivot = (None, None)

# Color ranges
# Load color ranges from file
color_ranges = {}
try:
    script_dir = os.path.dirname(os.path.abspath(__file__))
    tools_dir = os.path.join(script_dir, "..", "tools")
    tools_dir = os.path.abspath(tools_dir)
    with open(os.path.join(tools_dir, "color_ranges.json"), "r") as f:
        color_ranges = json.load(f)
except FileNotFoundError:
    print("Color ranges file not found. Using default values.")

# Boundary
LOWER_BLACK = (
    np.array(color_ranges["LOWER_BLACK"])
    if color_ranges["BLACK_COLOR_SPACE"] == "LAB"
    else np.array(color_ranges["LOWER_BLACK_HSV"])
)
UPPER_BLACK = (
    np.array(color_ranges["UPPER_BLACK"])
    if color_ranges["BLACK_COLOR_SPACE"] == "LAB"
    else np.array(color_ranges["UPPER_BLACK_HSV"])
)

# Lane color ranges
LOWER_ORANGE = (
    np.array(color_ranges["LOWER_ORANGE"])
    if color_ranges["ORANGE_COLOR_SPACE"] == "LAB"
    else np.array(color_ranges["LOWER_ORANGE_HSV"])
)
UPPER_ORANGE = (
    np.array(color_ranges["UPPER_ORANGE"])
    if color_ranges["ORANGE_COLOR_SPACE"] == "LAB"
    else np.array(color_ranges["UPPER_ORANGE_HSV"])
)
LOWER_BLUE = (
    np.array(color_ranges["LOWER_BLUE"])
    if color_ranges["BLUE_COLOR_SPACE"] == "LAB"
    else np.array(color_ranges["LOWER_BLUE_HSV"])
)
UPPER_BLUE = (
    np.array(color_ranges["UPPER_BLUE"])
    if color_ranges["BLUE_COLOR_SPACE"] == "LAB"
    else np.array(color_ranges["UPPER_BLUE_HSV"])
)

# Obstacle color ranges
LOWER_RED = (
    np.array(color_ranges["LOWER_RED"])
    if color_ranges["RED_COLOR_SPACE"] == "LAB"
    else np.array(color_ranges["LOWER_RED_HSV"])
)
UPPER_RED = (
    np.array(color_ranges["UPPER_RED"])
    if color_ranges["RED_COLOR_SPACE"] == "LAB"
    else np.array(color_ranges["UPPER_RED_HSV"])
)
LOWER_GREEN = (
    np.array(color_ranges["LOWER_GREEN"])
    if color_ranges["GREEN_COLOR_SPACE"] == "LAB"
    else np.array(color_ranges["LOWER_GREEN_HSV"])
)
UPPER_GREEN = (
    np.array(color_ranges["UPPER_GREEN"])
    if color_ranges["GREEN_COLOR_SPACE"] == "LAB"
    else np.array(color_ranges["UPPER_GREEN_HSV"])
)

# Parking color ranges
LOWER_MAGENTA = (
    np.array(color_ranges["LOWER_MAGENTA"])
    if color_ranges["MAGENTA_COLOR_SPACE"] == "LAB"
    else np.array(color_ranges["LOWER_MAGENTA_HSV"])
)
UPPER_MAGENTA = (
    np.array(color_ranges["UPPER_MAGENTA"])
    if color_ranges["MAGENTA_COLOR_SPACE"] == "LAB"
    else np.array(color_ranges["UPPER_MAGENTA_HSV"])
)

# reverse_black
LOWER_REVERSE_BLACK = LOWER_BLACK
UPPER_REVERSE_BLACK = UPPER_BLACK

contour_workers = ContourWorkers(
    # mode="NO_OBSTACLE",
    color_ranges=color_ranges,
    mode=MODE,
    has_parked_out=False,
    # color ranges
    lower_blue=LOWER_BLUE,
    upper_blue=UPPER_BLUE,
    lower_black=LOWER_BLACK,
    upper_black=UPPER_BLACK,
    lower_reverse_black=LOWER_REVERSE_BLACK,
    upper_reverse_black=UPPER_REVERSE_BLACK,
    lower_orange=LOWER_ORANGE,
    upper_orange=UPPER_ORANGE,
    lower_red=LOWER_RED,
    upper_red=UPPER_RED,
    lower_green=LOWER_GREEN,
    upper_green=UPPER_GREEN,
    upper_magenta=UPPER_MAGENTA,
    lower_magenta=LOWER_MAGENTA,
    # regions
    left_region=LEFT_REGION,
    right_region=RIGHT_REGION,
    lap_region=LAP_REGION,
    obs_region=OBS_REGION,
    front_wall_region=FRONT_WALL_REGION,
    reverse_region=REVERSE_REGION,
    parking_lot_region=PARKING_LOT_REGION,
)

contour_workers.parking_mode = False
STRAIGHT_CONST = 95
MAX_OFFSET_DEGREE = 55
maxRight = STRAIGHT_CONST + MAX_OFFSET_DEGREE
maxLeft = STRAIGHT_CONST - MAX_OFFSET_DEGREE

# PID controller constants
kp = 1.6
ki = 0.0
kd = 0.07
pid = PID(Kp=kp, Ki=ki, Kd=kd, setpoint=0)
pid.output_limits = (-1, 1)  # limit output to -1 to 1
pid.sample_time = 0.02

# Start/reverse/Stopping logic
speed = 0
normalized_angle_offset = 0
trigger_reverse = False
reverse_start_time = 0
reverse_duration = 0.6  # seconds
reverse_angle = STRAIGHT_CONST
reverse_pause_time = (
    1.3  # seconds to wait after reversing, will not initiate reverse during this period
)
last_reverse_end_time = 0

start_processing = False
stop_flag = False

# Serial communication
hardware = create_hardware(
    simulated=SIMULATED,
    mode=MODE,
    width=CAM_WIDTH,
    height=CAM_HEIGHT,
    argv=sys.argv[1:],
    rois={
        "left": LEFT_REGION,
        "right": RIGHT_REGION,
        "lap": LAP_REGION,
        "obstacle": OBS_REGION,
        "reverse": REVERSE_REGION,
        "front_wall": FRONT_WALL_REGION,
        "parking": PARKING_LOT_REGION,
        "left_danger": DANGER_ZONE_POINTS[0],
        "right_danger": DANGER_ZONE_POINTS[1],
    },
)
# Persisted ROI dimensions are loaded by the simulator hardware boundary.
BLACK_WALL_DETECTOR_AREA = (LEFT_REGION[2] - LEFT_REGION[0]) * (
    LEFT_REGION[3] - LEFT_REGION[1]
)
arduino = hardware.serial
if hardware.output_directory is not None:
    os.chdir(hardware.output_directory)
    print(f"Simulator logs: {hardware.output_directory}")
time.sleep(hardware.serial_warmup_seconds)
arduino.write(b"0,-1,95\n")

# parking
parking = Parking(
    parking_speed=20,
    arduino=arduino,
    camera_width=CAM_WIDTH,
    camera_height=CAM_HEIGHT,
    parking_lot_region=PARKING_LOT_REGION,
    maxLeft=maxLeft,
    maxRight=maxRight,
    STRAIGHT_CONST=STRAIGHT_CONST,
    MAX_OFFSET_DEGREE=MAX_OFFSET_DEGREE,
    REVERSE_REGION=REVERSE_REGION,
)
parking.has_parked_out = False


# -- Odometry --
# init odometry tracker and visualizer
start_zone_rect = [0.75, 1.5]  # meters x, y
tracker = OdometryTracker(
    wheel_radius=0.046, ticks_per_rev=2220, gear_ratio=1.0, debug=DEBUG
)
visualizer = OdometryVisualizer(
    title="Odometry Path (Single Encoder + Gyro)",
    start_zone_rect=start_zone_rect,
    debug=DEBUG,
)

encoder_ticks = 0
gyro_angle = 0.0
prev_encoder_ticks = 0
prev_gyro_angle = 0.0
last_odometry_time = time.time()
last_lap_time = time.time()
current_lap = 0
odometry_lap_samples = {}
DRIFT_ALPHA = 0.3  # between 0.05–0.3, depending on how noisy your drift is
MAX_DRIFT_THRESHOLD = 0.5
LAP_COUNT_INTERVAL = 6  # seconds
smoothed_drift_x = 0.0
smoothed_drift_y = 0.0


# Threading variables - separate queues for each detection task
def main():
    global stop_flag, speed, trigger_reverse, reverse_angle, last_reverse_end_time, normalized_angle_offset
    global last_odometry_time, last_lap_time, current_lap, odometry_lap_samples
    global encoder_ticks, gyro_angle, prev_encoder_ticks, prev_gyro_angle
    global start_processing, obstacle_wall_pivot, reverse_start_time
    global smoothed_drift_x, smoothed_drift_y

    # Camera object is physical or virtual, selected at the hardware boundary.
    picam2 = hardware.camera
    config = picam2.create_preview_configuration(
        main={"format": "RGB888", "size": (CAM_WIDTH, CAM_HEIGHT)}
    )
    picam2.configure(config)
    # load camera settings from file
    try:
        tools_dir = os.path.join(SCRIPT_DIR, "..", "tools")
        tools_dir = os.path.abspath(tools_dir)
        with open(os.path.join(tools_dir, "camera_settings.json"), "r") as f:
            settings = json.load(f)
            picam2.set_controls(settings)
            print("Loaded camera settings from file.")
    except FileNotFoundError:
        # Default settings if no file found
        print("No camera settings file found. Using default settings.")

    picam2.start()

    # Start all processing threads
    threads = []
    workers = [
        contour_workers.left_contour_worker,
        contour_workers.right_contour_worker,
        contour_workers.orange_contour_worker,
        contour_workers.blue_contour_worker,
        contour_workers.green_contour_worker,
        contour_workers.red_contour_worker,
        contour_workers.reverse_n_front_wall_contour_worker,
        contour_workers.parking_lot_contour_worker,
    ]

    for worker in workers:
        thread = threading.Thread(target=worker, daemon=True)
        thread.start()
        threads.append(thread)

    print(f"Started {len(threads)} processing threads")

    time.sleep(hardware.camera_warmup_seconds)
    angle = STRAIGHT_CONST

    try:
        while True:
            # dont start until start button pressed
            if arduino.in_waiting > 0:
                line = arduino.readline().decode("utf-8").rstrip()
                print(f"Arduino: {line}")
                if not start_processing and line == "START":
                    start_processing = True
                    last_lap_time = time.time()

                else:
                    # Get gyro data and encoder ticks
                    parts = line.split(",")
                    if len(parts) != 2:
                        print(f"Invalid data from Arduino: {line}")
                        continue
                    try:
                        encoder_ticks = -int(parts[0])
                        gyro_angle = float(parts[1])
                        print(f"Ticks: {encoder_ticks}, Gyro Angle: {gyro_angle}")
                    except ValueError:
                        print(f"Invalid number format from Arduino: {line}")
                        continue

            # ---- Draw odometry ----
            if (
                # (time.time() - last_odometry_time) >= 0.2 and
                start_processing
                and (abs(encoder_ticks - prev_encoder_ticks) > 300)
                # dont collect odometry data during parking maneuver
                and (parking.has_parked_out if MODE == "OBSTACLE" else True)
            ):
                # gyro_angle = clamp_angle(gyro_angle, threshold=7)
                # Update odometry tracker
                tracker.update(encoder_ticks, gyro_angle)
                # Get current position
                x, y, theta = tracker.get_position()
                # Update previous values
                prev_encoder_ticks = encoder_ticks
                prev_gyro_angle = gyro_angle
                last_odometry_time = time.time()

                if odometry_lap_samples.get(current_lap) is None:
                    odometry_lap_samples[current_lap] = []
                odometry_lap_samples[current_lap].append((x, y))

                # Drift correction using previous lap data
                if current_lap > 0:
                    # skip correction for first few samples
                    if len(odometry_lap_samples[current_lap]) > 6:
                        # get closest point to (x, y) from previous lap, because robot starts from a different position
                        prev_lap_points = odometry_lap_samples[current_lap - 1]
                        closest_point = min(
                            prev_lap_points,
                            key=lambda p: math.sqrt((p[0] - x) ** 2 + (p[1] - y) ** 2),
                        )
                        # Raw drift measurement
                        new_drift_x = x - closest_point[0]
                        new_drift_y = y - closest_point[1]
                        drift_mag = math.sqrt(new_drift_x**2 + new_drift_y**2)
                        if drift_mag < MAX_DRIFT_THRESHOLD:

                            # Smoothed drift (exponential moving average)
                            smoothed_drift_x = (
                                DRIFT_ALPHA * new_drift_x
                                + (1 - DRIFT_ALPHA) * smoothed_drift_x
                            )
                            smoothed_drift_y = (
                                DRIFT_ALPHA * new_drift_y
                                + (1 - DRIFT_ALPHA) * smoothed_drift_y
                            )

                            # Apply partial correction
                            correction_rate = 0.25
                            tracker.x -= smoothed_drift_x * correction_rate
                            tracker.y -= smoothed_drift_y * correction_rate

                # Update visualization
                visualizer.update_plot(
                    tracker.get_position_history(),
                    auto_fit=False,
                    current_angle=gyro_angle,
                )

                # current_intersections = round(abs(gyro_angle) / 90)
                if abs(x) < start_zone_rect[0] and abs(y) < start_zone_rect[1]:
                    if (time.time() - last_lap_time) >= LAP_COUNT_INTERVAL:
                        current_lap += 1

                    # keep resetting lap time only when inside start zone
                    last_lap_time = time.time()

                print(
                    f"Position: x={x:.3f}m, y={y:.3f}m, θ={math.degrees(theta):.1f}° | lap: {current_lap}"
                )

            # Capture frame
            frame = picam2.capture_array()

            # Distribute frame to all processing threads (non-blocking)
            frame_copy = frame.copy()
            contour_workers.put_frames_in_queues(frame_copy)

            # Retrieve all results from queues (non-blocking)
            (
                left_result,
                right_result,
                # orange_result,
                # blue_result,
                green_result,
                red_result,
                reverse_result,
                front_wall_result,
                parking_result,
            ) = contour_workers.collect_results()            

            # Use the latest processing results
            left_area = left_result.area
            right_area = right_result.area
            # orange_area = orange_result.area
            # blue_area = blue_result.area
            green_area = green_result.area
            red_area = red_result.area
            reverse_area = reverse_result.area
            front_wall_area = front_wall_result.area

            # Debug view
            if DEBUG:
                display_debug_screen(
                    mode=MODE,
                    frame=frame,
                    CAM_WIDTH=CAM_WIDTH,
                    CAM_HEIGHT=CAM_HEIGHT,
                    left_result=left_result,
                    right_result=right_result,
                    # orange_result=orange_result,
                    # blue_result=blue_result,
                    green_result=green_result,
                    red_result=red_result,
                    reverse_result=reverse_result,
                    LEFT_REGION=LEFT_REGION,
                    RIGHT_REGION=RIGHT_REGION,
                    LAP_REGION=LAP_REGION,
                    OBS_REGION=OBS_REGION,
                    REVERSE_REGION=REVERSE_REGION,
                    FRONT_WALL_REGION=FRONT_WALL_REGION,
                    # show_front_wall=show_front_wall,
                    DANGER_ZONE_POINTS=DANGER_ZONE_POINTS,
                    front_wall_result=front_wall_result,
                    angle=angle,
                    current_lap=current_lap,
                    left_area=left_area,
                    right_area=right_area,
                    obstacle_wall_pivot=obstacle_wall_pivot,
                    parking_mode=contour_workers.parking_mode,
                    parking_lot_region=PARKING_LOT_REGION,
                    parking_result=parking_result,
                    gyro_angle=gyro_angle,
                )

            if not start_processing:
                if DEBUG and cv2.waitKey(1) & 0xFF == ord("q"):
                    break
                # Dont start moving until start signal received
                continue


            # --- Reversing logic ---
            if (
                trigger_reverse
                # avoid reversing again too soon after last reverse
                and (last_reverse_end_time + reverse_pause_time) < time.time()
            ):
                speed = -MIN_SPEED
                if (time.time() - reverse_start_time) > reverse_duration:
                    trigger_reverse = False
                    last_reverse_end_time = time.time()
                    speed = 20  # stop after reversing
                    reverse_angle = STRAIGHT_CONST
                arduino.write(f"{speed},-1,{reverse_angle}\n".encode())
                print(f"Reversing... Speed: {speed}, Angle: {reverse_angle}")

                if DEBUG and cv2.waitKey(1) & 0xFF == ord("q"):
                    break
                print("continue")
                continue

            if reverse_area > 800:
                print("Reverse trigger detected!")
                trigger_reverse = True
                reverse_start_time = time.time()
                speed = -5  # stop before reversing
                arduino.write(f"{speed},-1,{reverse_angle}\n".encode())
                if DEBUG and cv2.waitKey(1) & 0xFF == ord("q"):
                    break

                print("continue")
                continue            

            # Only wall following            
            obstacle_wall_pivot = (None, None)            

            # interpolate
            left_area_normalized = np.interp(
                left_area,
                [0, BLACK_WALL_DETECTOR_AREA / 2, BLACK_WALL_DETECTOR_AREA],
                [0, 0.7, 1] if MODE == "NO_OBSTACLE" else [0, 0.5, 0.7],
            )
            right_area_normalized = np.interp(
                right_area,
                [0, BLACK_WALL_DETECTOR_AREA / 2, BLACK_WALL_DETECTOR_AREA],
                [0, 0.7, 1] if MODE == "NO_OBSTACLE" else [0, 0.5, 0.7],
            )

            error = right_area_normalized - left_area_normalized

            normalized_angle_offset = pid(error)
            print(
                f"Wall following | Norm: {normalized_angle_offset} | Error: {error}"
            )

            # --- Map normalized control to servo angle ---
            angle = int(
                max(
                    min(
                        STRAIGHT_CONST + normalized_angle_offset * MAX_OFFSET_DEGREE,
                        maxRight,
                    ),
                    maxLeft,
                )
            )

            # map speed with angle
            speed = np.interp(
                angle,
                [maxLeft, STRAIGHT_CONST, maxRight],
                [MIN_SPEED, MAX_SPEED, MIN_SPEED],
            )

            # Send to Arduino
            arduino.write(f"{speed},-1,{angle}\n".encode())

            # Stopping logic
            if (
                # contour_workers.mode == "NO_OBSTACLE"
                # and
                stop_flag
                and (clamp_angle(gyro_angle, threshold=30) % 360 == 0)
                # and (int(time.time()) - stopTime) > stop_timer
            ):
                print("Lap completed!")
                arduino.write(f"-5,-1,{angle}\n".encode())
                print(angle)
                break

            if current_lap >= TOTAL_LAPS and not stop_flag:
                stop_flag = True
                # Enable parking mode if in obstacle mode
                contour_workers.parking_mode = True if MODE == "OBSTACLE" else False
                print("Preparing to stop...")

            if DEBUG and cv2.waitKey(1) & 0xFF == ord("q"):
                break

    except KeyboardInterrupt:
        print("Interrupted by user")
    finally:
        # Cleanup
        contour_workers.stop_processing.set()
        for thread in threads:
            thread.join(timeout=1.0)
        picam2.stop()
        arduino.close()
        cv2.destroyAllWindows()
        tracker.close()


if __name__ == "__main__":
    run_entrypoint(main, hardware)
