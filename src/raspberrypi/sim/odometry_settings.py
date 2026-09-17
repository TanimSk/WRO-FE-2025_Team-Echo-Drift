"""Apply editor calibration once, after controller initialization, before main.

Keep the existing tracker, plotter and controller algorithms intact. The explicit
namespace mapping is centralized at the hardware entry point for both challenges.
"""


def apply_odometry_settings(namespace, config):
    o = config.odometry
    zone = namespace["start_zone_rect"]
    zone[:] = [o.start_zone_half_width, o.start_zone_half_height]
    visualizer = namespace["visualizer"]
    visualizer.start_zone_rect_x, visualizer.start_zone_rect_y = zone
    tracker = namespace["tracker"]
    tracker.wheel_radius = o.estimated_wheel_radius
    tracker.ticks_per_rev = o.estimated_ticks_per_rev
    tracker.gear_ratio = o.estimated_gear_ratio
    namespace["LAP_COUNT_INTERVAL"] = o.lap_interval_seconds
    namespace["DRIFT_ALPHA"] = o.drift_alpha
    namespace["MAX_DRIFT_THRESHOLD"] = o.max_drift_threshold
