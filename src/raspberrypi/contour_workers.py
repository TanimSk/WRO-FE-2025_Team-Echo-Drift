from img_processing_functions import (
    find_contours,
    max_contour_area,
    find_color_signal_box,
)
import threading
from queue import Queue, Empty
import cv2
import numpy as np
from typing import Literal


class ContourResult:
    def __init__(self, area=0, contours=None, metadata=""):
        self.area = area
        self.contours = contours or []
        self.metadata = metadata


class ContourWorkers:
    def __init__(
        self,
        color_ranges: dict,
        mode: Literal["OBSTACLE", "NO_OBSTACLE"],
        has_parked_out: bool,
        # color ranges
        lower_blue: np.ndarray,
        upper_blue: np.ndarray,
        lower_black: np.ndarray,
        upper_black: np.ndarray,
        lower_reverse_black: np.ndarray,
        upper_reverse_black: np.ndarray,
        lower_orange: np.ndarray,
        upper_orange: np.ndarray,
        lower_red: np.ndarray,
        upper_red: np.ndarray,
        lower_green: np.ndarray,
        upper_green: np.ndarray,
        upper_magenta: np.ndarray,
        lower_magenta: np.ndarray,
        # regions of interest
        left_region: list,
        right_region: list,
        lap_region: list,
        obs_region: list,
        front_wall_region: list,
        reverse_region: list,
        parking_lot_region: list,
    ):
        self.color_ranges = color_ranges
        self.mode = mode
        self.parking_mode = False
        self.has_parked_out = has_parked_out

        # colors
        self.LOWER_BLUE = lower_blue
        self.UPPER_BLUE = upper_blue
        self.LOWER_BLACK = lower_black
        self.UPPER_BLACK = upper_black
        self.LOWER_ORANGE = lower_orange
        self.UPPER_ORANGE = upper_orange
        self.LOWER_RED = lower_red
        self.UPPER_RED = upper_red
        self.LOWER_GREEN = lower_green
        self.UPPER_GREEN = upper_green
        self.LOWER_MAGENTA = lower_magenta
        self.UPPER_MAGENTA = upper_magenta
        self.UPPER_REVERSE_BLACK = upper_reverse_black
        self.LOWER_REVERSE_BLACK = lower_reverse_black

        # regions of interest
        self.LEFT_REGION = left_region
        self.RIGHT_REGION = right_region
        self.LAP_REGION = lap_region
        self.OBS_REGION = obs_region
        self.FRONT_WALL_REGION = front_wall_region
        self.REVERSE_REGION = reverse_region
        self.PARKING_LOT_REGION = parking_lot_region

        # queues
        self.frame_queue_left = Queue(maxsize=2)
        self.frame_queue_right = Queue(maxsize=2)
        self.frame_queue_orange = Queue(maxsize=2)
        self.frame_queue_blue = Queue(maxsize=2)
        self.frame_queue_green = Queue(maxsize=2)
        self.frame_queue_red = Queue(maxsize=2)
        self.frame_queue_reverse = Queue(maxsize=2)
        self.frame_queue_front_wall = Queue(maxsize=2)
        self.frame_queue_parking_lot = Queue(maxsize=2)

        # result queues
        self.result_queue_left = Queue(maxsize=2)
        self.result_queue_right = Queue(maxsize=2)
        self.result_queue_orange = Queue(maxsize=2)
        self.result_queue_blue = Queue(maxsize=2)
        self.result_queue_green = Queue(maxsize=2)
        self.result_queue_red = Queue(maxsize=2)
        self.result_queue_reverse = Queue(maxsize=2)
        self.result_queue_front_wall = Queue(maxsize=2)
        self.result_queue_parking_lot = Queue(maxsize=2)

        # control event
        self.stop_processing = threading.Event()

        # results
        self.left_result = ContourResult()
        self.right_result = ContourResult()
        self.orange_result = ContourResult()
        self.blue_result = ContourResult()
        self.green_result = ContourResult()
        self.red_result = ContourResult()
        self.reverse_result = ContourResult()
        self.front_wall_result = ContourResult()
        self.parking_lot_result = ContourResult()


    def clear_all_queues(self):
        """Clear all frame and result queues"""
        queues_to_clear = [
            # Frame queues
            self.frame_queue_left,
            self.frame_queue_right,
            self.frame_queue_orange,
            self.frame_queue_blue,
            self.frame_queue_green,
            self.frame_queue_red,
            self.frame_queue_reverse,
            self.frame_queue_front_wall,
            self.frame_queue_parking_lot,
            # Result queues
            self.result_queue_left,
            self.result_queue_right,
            self.result_queue_orange,
            self.result_queue_blue,
            self.result_queue_green,
            self.result_queue_red,
            self.result_queue_reverse,
            self.result_queue_front_wall,
            self.result_queue_parking_lot,
        ]
        
        for queue in queues_to_clear:
            try:
                while not queue.empty():
                    queue.get_nowait()
                    queue.task_done()
            except Empty:
                pass
            except Exception as e:
                print(f"Error clearing queue: {e}")

    def put_frames_in_queues(self, frame_copy):
        try:
            self.frame_queue_left.put_nowait(frame_copy)
        except:
            pass  # Skip if queue full

        try:
            self.frame_queue_right.put_nowait(frame_copy)
        except:
            pass

        # try:
        #     self.frame_queue_orange.put_nowait(frame_copy)
        # except:
        #     pass

        # try:
        #     self.frame_queue_blue.put_nowait(frame_copy)
        # except:
        #     pass

        try:
            self.frame_queue_reverse.put_nowait(frame_copy)
        except:
            pass

        if self.mode == "OBSTACLE":
            try:
                self.frame_queue_green.put_nowait(frame_copy)
            except:
                pass

            try:
                self.frame_queue_red.put_nowait(frame_copy)
            except:
                pass

            try:
                self.frame_queue_front_wall.put_nowait(frame_copy)
            except:
                pass

        if self.parking_mode:
            try:
                self.frame_queue_parking_lot.put_nowait(frame_copy)
            except:
                pass

    def collect_results(self):
        try:
            while not self.result_queue_left.empty():
                self.left_result = self.result_queue_left.get_nowait()
        except Empty:
            pass

        try:
            while not self.result_queue_right.empty():
                self.right_result = self.result_queue_right.get_nowait()
        except Empty:
            pass

        try:
            while not self.result_queue_orange.empty():
                self.orange_result = self.result_queue_orange.get_nowait()
        except Empty:
            pass

        try:
            while not self.result_queue_blue.empty():
                self.blue_result = self.result_queue_blue.get_nowait()
        except Empty:
            pass

        try:
            while not self.result_queue_green.empty():
                self.green_result = self.result_queue_green.get_nowait()
        except Empty:
            pass

        try:
            while not self.result_queue_red.empty():
                self.red_result = self.result_queue_red.get_nowait()
        except Empty:
            pass

        try:
            while not self.result_queue_reverse.empty():
                self.reverse_result = self.result_queue_reverse.get_nowait()
        except Empty:
            pass

        try:
            while not self.result_queue_front_wall.empty():
                self.front_wall_result = self.result_queue_front_wall.get_nowait()
        except Empty:
            pass

        try:
            while not self.result_queue_parking_lot.empty():
                self.parking_lot_result = self.result_queue_parking_lot.get_nowait()
        except Empty:
            pass

        return (
            self.left_result,
            self.right_result,
            # self.orange_result,
            # self.blue_result,
            self.green_result,
            self.red_result,
            self.reverse_result,
            self.front_wall_result,
            self.parking_lot_result,
        )

    def left_contour_worker(self):
        """Worker thread for left ROI black line detection"""
        while not self.stop_processing.is_set():
            try:
                frame = self.frame_queue_left.get(timeout=0.1)
                contours = find_contours(
                    frame,
                    self.LOWER_BLACK,
                    self.UPPER_BLACK,
                    self.LEFT_REGION,
                    direction="left",
                    consider_area=400,
                    method=self.color_ranges["BLACK_COLOR_SPACE"],
                )
                black_area, _ = max_contour_area(contours)
                result = ContourResult(black_area, contours, "black_left")

                if self.mode == "OBSTACLE" and self.has_parked_out:
                    frame = self.frame_queue_left.get(timeout=0.1)
                    contours = find_contours(
                        frame,
                        self.LOWER_MAGENTA,
                        self.UPPER_MAGENTA,
                        self.LEFT_REGION,
                        direction="left",
                        use_convex_hull=False,
                        consider_area=400,
                        method=self.color_ranges["MAGENTA_COLOR_SPACE"],
                    )
                    magenta_area, _ = max_contour_area(contours)

                    if magenta_area > 0:
                        result = ContourResult(magenta_area, contours, "magenta_left")

                try:
                    self.result_queue_left.put_nowait(result)
                except:
                    try:
                        self.result_queue_left.get_nowait()
                        self.result_queue_left.put_nowait(result)
                    except Empty:
                        pass

                self.frame_queue_left.task_done()
            except Empty:
                continue
            except Exception as e:
                print(f"Left processing error: {e}")
                continue

    def right_contour_worker(self):
        """Worker thread for right ROI black line detection"""
        while not self.stop_processing.is_set():
            try:
                frame = self.frame_queue_right.get(timeout=0.1)
                contours = find_contours(
                    frame,
                    self.LOWER_BLACK,
                    self.UPPER_BLACK,
                    self.RIGHT_REGION,
                    direction="right",
                    consider_area=400,
                    method=self.color_ranges["BLACK_COLOR_SPACE"],
                )
                black_area, _ = max_contour_area(contours)
                result = ContourResult(black_area, contours, "black_right")

                if self.mode == "OBSTACLE" and self.has_parked_out:
                    frame = self.frame_queue_right.get(timeout=0.1)
                    contours = find_contours(
                        frame,
                        self.LOWER_MAGENTA,
                        self.UPPER_MAGENTA,
                        self.RIGHT_REGION,
                        direction="right",
                        use_convex_hull=True,
                        consider_area=400,
                        method=self.color_ranges["MAGENTA_COLOR_SPACE"],
                    )
                    magenta_area, _ = max_contour_area(contours)

                    if magenta_area > 0:
                        result = ContourResult(magenta_area, contours, "magenta_right")

                try:
                    self.result_queue_right.put_nowait(result)
                except:
                    try:
                        self.result_queue_right.get_nowait()
                        self.result_queue_right.put_nowait(result)
                    except Empty:
                        pass

                self.frame_queue_right.task_done()
            except Empty:
                continue
            except Exception as e:
                print(f"Right processing error: {e}")
                continue

    def orange_contour_worker(self):
        """Worker thread for orange marker detection"""
        while not self.stop_processing.is_set():
            try:
                frame = self.frame_queue_orange.get(timeout=0.1)
                contours = find_contours(
                    frame,
                    self.LOWER_ORANGE,
                    self.UPPER_ORANGE,
                    self.LAP_REGION,                    
                    method=self.color_ranges["ORANGE_COLOR_SPACE"],
                )
                area, _ = max_contour_area(contours)
                result = ContourResult(area, contours)

                try:
                    self.result_queue_orange.put_nowait(result)
                except:
                    try:
                        self.result_queue_orange.get_nowait()
                        self.result_queue_orange.put_nowait(result)
                    except Empty:
                        pass

                self.frame_queue_orange.task_done()
            except Empty:
                continue
            except Exception as e:
                print(f"Orange processing error: {e}")
                continue

    def blue_contour_worker(self):
        """Worker thread for blue marker detection"""
        while not self.stop_processing.is_set():
            try:
                frame = self.frame_queue_blue.get(timeout=0.1)
                contours = find_contours(
                    frame,
                    self.LOWER_BLUE,
                    self.UPPER_BLUE,
                    self.LAP_REGION,
                    method=self.color_ranges["BLUE_COLOR_SPACE"],
                )
                area, _ = max_contour_area(contours)
                result = ContourResult(area, contours)

                try:
                    self.result_queue_blue.put_nowait(result)
                except:
                    try:
                        self.result_queue_blue.get_nowait()
                        self.result_queue_blue.put_nowait(result)
                    except Empty:
                        pass

                self.frame_queue_blue.task_done()
            except Empty:
                continue
            except Exception as e:
                print(f"Blue processing error: {e}")
                continue

    def green_contour_worker(self):
        """Worker thread for green marker detection"""
        while not self.stop_processing.is_set():
            try:
                frame = self.frame_queue_green.get(timeout=0.1)
                contours = find_contours(
                    frame,
                    self.LOWER_GREEN,
                    self.UPPER_GREEN,
                    self.OBS_REGION,
                    consider_area=450,
                    blur=3,
                    method=self.color_ranges["GREEN_COLOR_SPACE"],
                )
                area, _ = max_contour_area(contours)
                result = ContourResult(area, contours, "green_pillar")

                try:
                    self.result_queue_green.put_nowait(result)
                except:
                    try:
                        self.result_queue_green.get_nowait()
                        self.result_queue_green.put_nowait(result)
                    except Empty:
                        pass

                self.frame_queue_green.task_done()
            except Empty:
                continue
            except Exception as e:
                print(f"Green processing error: {e}")
                continue

    def red_contour_worker(self):
        """Worker thread for red marker detection"""
        while not self.stop_processing.is_set():
            try:
                frame = self.frame_queue_red.get(timeout=0.1)
                contours = find_contours(
                    frame,
                    self.LOWER_RED,
                    self.UPPER_RED,
                    self.OBS_REGION,
                    consider_area=450,
                    blur=3,
                    method=self.color_ranges["RED_COLOR_SPACE"],
                )
                area, _ = max_contour_area(contours)
                result = ContourResult(area, contours, "red_pillar")

                try:
                    self.result_queue_red.put_nowait(result)
                except:
                    try:
                        self.result_queue_red.get_nowait()
                        self.result_queue_red.put_nowait(result)
                    except Empty:
                        pass

                self.frame_queue_red.task_done()
            except Empty:
                continue
            except Exception as e:
                print(f"Red processing error: {e}")
                continue

    def parking_lot_contour_worker(self):
        """Worker thread for parking lot marker detection"""
        while not self.stop_processing.is_set():
            try:
                frame = self.frame_queue_parking_lot.get(timeout=0.1)
                contours = find_contours(
                    frame,
                    self.LOWER_MAGENTA,
                    self.UPPER_MAGENTA,
                    self.PARKING_LOT_REGION,
                    use_convex_hull=True,
                    consider_area=800,
                    method=self.color_ranges["MAGENTA_COLOR_SPACE"],
                )
                area, _ = max_contour_area(contours)
                result = ContourResult(area, contours, "magenta_parking_lot")

                try:
                    self.result_queue_parking_lot.put_nowait(result)
                except:
                    try:
                        self.result_queue_parking_lot.get_nowait()
                        self.result_queue_parking_lot.put_nowait(result)
                    except Empty:
                        pass

                self.frame_queue_parking_lot.task_done()
            except Empty:
                continue
            except Exception as e:
                print(f"Parking lot processing error: {e}")
                continue

    def reverse_n_front_wall_contour_worker(self):
        """Worker thread for reverse trigger detection"""
        while not self.stop_processing.is_set():
            try:

                # reverse region processing
                frame = self.frame_queue_reverse.get(timeout=0.1)
                contours = find_contours(
                    frame,
                    self.LOWER_REVERSE_BLACK,
                    self.UPPER_REVERSE_BLACK,
                    self.REVERSE_REGION,
                    method=self.color_ranges["BLACK_COLOR_SPACE"],
                )
                area, _ = max_contour_area(contours)
                result = ContourResult(area, contours, "reverse_trigger")

                try:
                    self.result_queue_reverse.put_nowait(result)
                except:
                    try:
                        self.result_queue_reverse.get_nowait()
                        self.result_queue_reverse.put_nowait(result)
                    except Empty:
                        pass

                self.frame_queue_reverse.task_done()

                # front wall region processing
                frame_fw = self.frame_queue_front_wall.get(timeout=0.1)
                contours_fw = find_contours(
                    frame_fw,
                    self.LOWER_BLACK,
                    self.UPPER_BLACK,
                    self.FRONT_WALL_REGION,
                    use_convex_hull=True,
                    method=self.color_ranges["BLACK_COLOR_SPACE"],
                )
                area_fw, _ = max_contour_area(contours_fw)
                result_fw = ContourResult(area_fw, contours_fw, "front_wall_trigger")
                try:
                    self.result_queue_front_wall.put_nowait(result_fw)
                except:
                    try:
                        self.result_queue_front_wall.get_nowait()
                        self.result_queue_front_wall.put_nowait(result_fw)
                    except Empty:
                        pass
                self.frame_queue_front_wall.task_done()

            except Empty:
                continue
            except Exception as e:
                print(f"Reverse and front wall processing error: {e}")
                continue
