from __future__ import annotations

import cv2
import numpy as np
from collections import defaultdict

# Optional diagnostic sinks, installed only by the simulator's processing view.
# They observe masks and display frames; detector outputs are unchanged.
mask_observer = None
debug_frame_observer = None


def replace_closest(polygon, new_point):
    # Compute distances from new_point to each polygon vertex
    distances = np.linalg.norm(polygon - new_point, axis=1)
    # Index of the closest point
    idx = np.argmin(distances)
    # Replace that vertex
    polygon[idx] = new_point
    return polygon


def check_overlap(region, contours):
    """
    region: [x1, y1, x2, y2] -> rectangle region
    contours: list of contours from cv2.findContours
    return: True if any contour overlaps region, else False
    """

    # Convert region into a rectangle contour
    rect_pts = (
        np.array(
            [
                [region[0], region[1]],  # top-left
                [region[2], region[1]],  # top-right
                [region[2], region[3]],  # bottom-right
                [region[0], region[3]],  # bottom-left
            ]
        )
        .reshape((-1, 1, 2))
        .astype(np.int32)
    )

    for cnt in contours:
        area, _ = cv2.intersectConvexConvex(
            cnt.astype(np.float32), rect_pts.astype(np.float32)
        )
        if area > 0:  # overlap detected
            return True

    return False


def transform_danger_zone_xshift(points, steering_angle, sensitivity=2):
    transformed = points
    # for i, line in enumerate(points):
    #     x1, y1, x2, y2 = line["x1"], line["y1"], line["x2"], line["y2"]

    #     # shift top x1 only (keep y same, keep bottom fixed)
    #     if i == 0:  # left line
    #         x1_new = int(x1 - steering_angle * sensitivity)
    #     else:  # right line
    #         x1_new = int(x1 - steering_angle * sensitivity)

    #     transformed.append({"x1": x1_new, "y1": y1, "x2": x2, "y2": y2})
    return transformed


def create_mask(hsv, lower_limit, upper_limit, kernel):
    """Creates a mask with given HSV limits and applies morphological operations using a precomputed kernel."""
    mask = cv2.inRange(hsv, lower_limit, upper_limit)
    mask = cv2.dilate(mask, kernel, iterations=2)
    mask = cv2.erode(mask, kernel, iterations=2)
    return mask


def boxes_to_contours(boxes):
    contours = []
    for x, y, w, h in boxes:
        contour = np.array(
            [[[x, y]], [[x + w, y]], [[x + w, y + h]], [[x, y + h]]], dtype=np.int32
        )
        contours.append(contour)
    return contours


def find_color_signal_box(frame, lower_color, upper_color, roi, consider_area=3000):
    x1, y1, x2, y2 = roi
    roi_frame = frame[y1:y2, x1:x2]
    blurred = cv2.medianBlur(roi_frame, 1)  # Reduced from 15 to 7
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    kernel = np.ones((9, 9), np.uint8)
    mask = create_mask(hsv, lower_color, upper_color, kernel)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    # Process largest contours first
    contours = sorted(contours, key=cv2.contourArea, reverse=True)
    detected_boxes = []

    for cnt in contours:
        if cv2.contourArea(cnt) < consider_area:
            continue  # Skip small contours
        x, y, w, h = cv2.boundingRect(cnt)
        detected_boxes.append((x, y, w, h))  # Adjust coordinates to original frame

    return boxes_to_contours(detected_boxes)


def find_contours(
    frame,
    lower_color,
    upper_color,
    roi,
    direction=None,
    use_convex_hull=False,
    consider_area=None,
    blur=7,
    method="LAB",
):
    x1, y1, x2, y2 = roi
    roi_frame = frame[y1:y2, x1:x2]

    if method == "LAB":
        img = cv2.cvtColor(roi_frame, cv2.COLOR_RGB2Lab)        
    else:
        img = cv2.cvtColor(roi_frame, cv2.COLOR_RGB2HSV)

    # lighting normalization
    # l, a, b = cv2.split(labImg)
    # clahe = cv2.createCLAHE(clipLimit=1.0, tileGridSize=(8,8))
    # l = clahe.apply(l)
    # labImg = cv2.merge((l,a,b))

    # blur and mask
    img_blur = cv2.medianBlur(img, blur)
    # img_blur = cv2.bilateralFilter(labImg, d=7, sigmaColor=75, sigmaSpace=75)
    mask = cv2.inRange(img_blur, lower_color, upper_color)
    kernel = np.ones((13, 13), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    if mask_observer is not None:
        mask_observer(roi, lower_color, method, mask)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if consider_area is not None:
        contours = [cnt for cnt in contours if cv2.contourArea(cnt) >= consider_area]

    if use_convex_hull and len(contours) > 0:
        all_points = np.concatenate(contours, axis=0)
        hull = cv2.convexHull(all_points)
        contours = [hull]

    if direction is not None and contours:
        # if max_contour_area(contours)[0] < 1700:
        #     return contours

        # Combine all points into one array safely
        all_points = np.concatenate(contours, axis=0)
        hull = cv2.convexHull(all_points)
        hull = hull.reshape(-1, 2)

        if direction == "right":
            hull = replace_closest(hull, np.array([roi[2] - roi[0], 0]))
            hull = replace_closest(hull, np.array([roi[2] - roi[0], roi[3] - roi[1]]))

        elif direction == "left":
            hull = replace_closest(hull, np.array([0, 0]))
            hull = replace_closest(hull, np.array([0, roi[3] - roi[1]]))

        contours = [hull.reshape(-1, 1, 2)]

    return contours


def max_contour_area(contours):
    if len(contours) == 0:
        return (0, None)
    largest = max(contours, key=cv2.contourArea)
    return (cv2.contourArea(largest), largest)


def display_roi(frame, rois, color):
    for roi in rois:
        x1, y1, x2, y2 = roi
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
    return frame


def display_debug_screen(
    mode,
    CAM_WIDTH,
    CAM_HEIGHT,
    frame,
    LEFT_REGION,
    RIGHT_REGION,
    LAP_REGION,
    OBS_REGION,
    FRONT_WALL_REGION,
    # show_front_wall,
    front_wall_result,
    REVERSE_REGION,
    DANGER_ZONE_POINTS,
    left_result,
    right_result,
    # orange_result,
    # blue_result,
    green_result,
    red_result,
    reverse_result,
    angle,
    current_lap,
    left_area,
    right_area,
    obstacle_wall_pivot,
    parking_mode,
    parking_lot_region,
    parking_result,
    gyro_angle,
):
    debug_frame = frame.copy()
    debug_frame = display_roi(
        debug_frame,
        [LEFT_REGION, RIGHT_REGION, OBS_REGION, REVERSE_REGION],
        (255, 0, 255),
    )

    # if show_front_wall:
    debug_frame = display_roi(debug_frame, [FRONT_WALL_REGION], (0, 255, 255))

    if parking_mode:
        debug_frame = display_roi(debug_frame, [parking_lot_region], (0, 255, 0))

    if obstacle_wall_pivot != (None, None):
        print(f"Obstacle wall pivot: {obstacle_wall_pivot}")
        cv2.circle(
            debug_frame,
            obstacle_wall_pivot,
            5,
            (0, 0, 255),
        )

    cv2.line(
        debug_frame,
        (CAM_WIDTH // 2, 0),
        (CAM_WIDTH // 2, CAM_HEIGHT),
        (255, 0, 0),
        1,
    )  # blue line in the center

    # draw danger zone
    if mode == "OBSTACLE":
        cv2.line(
            debug_frame,
            (DANGER_ZONE_POINTS[0]["x1"], DANGER_ZONE_POINTS[0]["y1"]),
            (DANGER_ZONE_POINTS[0]["x2"], DANGER_ZONE_POINTS[0]["y2"]),
            (0, 0, 255),
            2,
        )
        cv2.line(
            debug_frame,
            (DANGER_ZONE_POINTS[1]["x1"], DANGER_ZONE_POINTS[1]["y1"]),
            (DANGER_ZONE_POINTS[1]["x2"], DANGER_ZONE_POINTS[1]["y2"]),
            (0, 0, 255),
            2,
        )

    # Draw contours using the latest results
    if left_result.contours:
        cv2.drawContours(
            debug_frame[
                LEFT_REGION[1] : LEFT_REGION[3], LEFT_REGION[0] : LEFT_REGION[2]
            ],
            left_result.contours,
            -1,
            (0, 255, 0),
            2,
        )
    if right_result.contours:
        cv2.drawContours(
            debug_frame[
                RIGHT_REGION[1] : RIGHT_REGION[3], RIGHT_REGION[0] : RIGHT_REGION[2]
            ],
            right_result.contours,
            -1,
            (0, 255, 0),
            2,
        )
    # if orange_result.contours:
    #     cv2.drawContours(
    #         debug_frame[LAP_REGION[1] : LAP_REGION[3], LAP_REGION[0] : LAP_REGION[2]],
    #         orange_result.contours,
    #         -1,
    #         (0, 165, 255),
    #         2,
    #     )
    # if blue_result.contours:
    #     cv2.drawContours(
    #         debug_frame[LAP_REGION[1] : LAP_REGION[3], LAP_REGION[0] : LAP_REGION[2]],
    #         blue_result.contours,
    #         -1,
    #         (0, 165, 255),
    #         2,
    #     )

    if green_result.contours:
        cv2.drawContours(
            debug_frame[OBS_REGION[1] : OBS_REGION[3], OBS_REGION[0] : OBS_REGION[2]],
            green_result.contours,
            -1,
            (0, 255, 0),
            2,
        )
    if red_result.contours:
        cv2.drawContours(
            debug_frame[OBS_REGION[1] : OBS_REGION[3], OBS_REGION[0] : OBS_REGION[2]],
            red_result.contours,
            -1,
            (0, 255, 255),
            2,
        )

    if reverse_result.contours:
        cv2.drawContours(
            debug_frame[
                REVERSE_REGION[1] : REVERSE_REGION[3],
                REVERSE_REGION[0] : REVERSE_REGION[2],
            ],
            reverse_result.contours,
            -1,
            (255, 0, 0),
            2,
        )

    if front_wall_result.contours:
        cv2.drawContours(
            debug_frame[
                FRONT_WALL_REGION[1] : FRONT_WALL_REGION[3],
                FRONT_WALL_REGION[0] : FRONT_WALL_REGION[2],
            ],
            front_wall_result.contours,
            -1,
            (0, 255, 0),
            2,
        )

    if parking_mode and parking_result.contours:
        cv2.drawContours(
            debug_frame[
                parking_lot_region[1] : parking_lot_region[3],
                parking_lot_region[0] : parking_lot_region[2],
            ],
            parking_result.contours,
            -1,
            (255, 255, 0),
            2,
        )

    status = f"Angle: {angle} | Lap: {current_lap} | L: {left_area} | R: {right_area} | Gyro: {gyro_angle}"
    cv2.putText(
        debug_frame,
        status,
        (10, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (255, 0, 0),
        2,
    )

    if debug_frame_observer is not None:
        debug_frame_observer(debug_frame)
    else:
        cv2.imshow("Debug View", debug_frame)


def get_max_y_coord(contours) -> tuple[int, int] | tuple[None, None]:
    if not contours:  # empty list
        return (None, None)

    all_points = np.vstack(contours).reshape(-1, 2)
    max_idx = np.argmax(all_points[:, 1])  # index of largest y
    return tuple(all_points[max_idx])  # (x, y)


def get_min_x_coord(contours) -> tuple[int, int] | tuple[None, None]:
    if not contours:  # empty list
        print("No contours found for min x coord")
        return (None, None)

    all_points = np.vstack(contours).reshape(-1, 2)
    min_idx = np.argmin(all_points[:, 0])  # index of smallest x
    return tuple(all_points[min_idx])  # (x, y)


def get_max_x_coord(contours) -> tuple[int, int] | tuple[None, None]:
    if not contours:  # empty list
        print("No contours found for max x coord")
        return (None, None)

    all_points = np.vstack(contours).reshape(-1, 2)
    max_idx = np.argmax(all_points[:, 0])  # index of largest x
    return tuple(all_points[max_idx])  # (x, y)


def get_overall_centroid(contours):
    """
    Calculate the single centroid of multiple contours combined.

    Args:
        contours (list): List of contours (each contour is a numpy array of points)

    Returns:
        tuple: (cx, cy) as the centroid of all contours combined, or (None, None) if invalid
    """
    # Compute moments for all contours together
    M = {"m00": 0, "m10": 0, "m01": 0}

    for contour in contours:
        m = cv2.moments(contour)
        M["m00"] += m["m00"]
        M["m10"] += m["m10"]
        M["m01"] += m["m01"]

    if M["m00"] != 0:
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        return (cx, cy)
    else:
        return (None, None)  # invalid if total area = 0


def point_position(x1, y1, x2, y2, target_x3, target_y3):
    val = (x2 - x1) * (target_y3 - y1) - (y2 - y1) * (target_x3 - x1)
    if val > 0:
        return "LEFT"
    elif val < 0:
        return "RIGHT"
    else:
        return "COLLINEAR"
