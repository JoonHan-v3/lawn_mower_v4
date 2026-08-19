#!/usr/bin/python3

"""Three-way comparison to tell apart the two reasons a mowed trail can look
messy while nav2 itself reports no errors:

  * controller tracking - the EKF estimate agrees with ground truth, but both
    stray from the planned path, so the controller is steering badly.
  * localization - the EKF estimate hugs the planned path while ground truth
    does not, so nav2 is tracking a pose that isn't where the robot really is.

Reports cross-track error of each against /coverage_path, plus the gap between
the two pose sources. Ctrl-C to print the summary.
"""

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from nav_msgs.msg import Odometry, Path
from tf2_msgs.msg import TFMessage


def cross_track_error(point, path_points):
    """Shortest distance from point to the polyline through path_points."""
    px, py = point
    best = float('inf')
    for (x0, y0), (x1, y1) in zip(path_points, path_points[1:]):
        dx, dy = x1 - x0, y1 - y0
        seg_sq = dx * dx + dy * dy
        if seg_sq < 1e-12:
            d = math.hypot(px - x0, py - y0)
        else:
            t = max(0.0, min(1.0, ((px - x0) * dx + (py - y0) * dy) / seg_sq))
            d = math.hypot(px - (x0 + t * dx), py - (y0 + t * dy))
        if d < best:
            best = d
    return best


class PathTrackingMonitor(Node):

    def __init__(self):
        super().__init__('path_tracking_monitor')

        self.declare_parameter('sample_rate', 5.0)
        sample_rate = self.get_parameter('sample_rate').value

        latched_qos = QoSProfile(depth=1)
        latched_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        self.path_points = None
        self.create_subscription(Path, '/coverage_path', self.on_path, latched_qos)

        self.truth_xy = None
        self.create_subscription(TFMessage, 'tf_ground_truth', self.on_ground_truth, 10)

        self.ekf_xy = None
        self.create_subscription(
            Odometry, '/odometry/filtered/global', self.on_ekf, 10)

        self.truth_errors = []
        self.ekf_errors = []
        self.estimate_gaps = []

        self.create_timer(1.0 / sample_rate, self.on_timer)
        self.get_logger().info('path_tracking_monitor started, waiting for /coverage_path')

    def on_path(self, msg: Path):
        self.path_points = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self.get_logger().info(f'coverage path received: {len(self.path_points)} points')

    def on_ground_truth(self, msg):
        # index 0 is the robot in this world's dynamic pose list, matching
        # the convention grass_mower.py already relies on
        if not msg.transforms:
            return
        t = msg.transforms[0].transform.translation
        self.truth_xy = (t.x, t.y)

    def on_ekf(self, msg: Odometry):
        p = msg.pose.pose.position
        self.ekf_xy = (p.x, p.y)

    def on_timer(self):
        if self.path_points is None or self.truth_xy is None or self.ekf_xy is None:
            return

        self.truth_errors.append(cross_track_error(self.truth_xy, self.path_points))
        self.ekf_errors.append(cross_track_error(self.ekf_xy, self.path_points))
        self.estimate_gaps.append(
            math.hypot(self.truth_xy[0] - self.ekf_xy[0],
                       self.truth_xy[1] - self.ekf_xy[1]))

    def report(self):
        if not self.truth_errors:
            self.get_logger().error(
                'no samples collected - was /coverage_path published and the robot running?')
            return

        def stats(values):
            ordered = sorted(values)
            mean = sum(ordered) / len(ordered)
            p95 = ordered[int(0.95 * (len(ordered) - 1))]
            return mean, p95, ordered[-1]

        t_mean, t_p95, t_max = stats(self.truth_errors)
        e_mean, e_p95, e_max = stats(self.ekf_errors)
        g_mean, g_p95, g_max = stats(self.estimate_gaps)

        self.get_logger().info(f'samples: {len(self.truth_errors)}')
        self.get_logger().info(
            f'ground truth  vs planned path: mean {t_mean:.3f}m  p95 {t_p95:.3f}m  max {t_max:.3f}m')
        self.get_logger().info(
            f'EKF estimate  vs planned path: mean {e_mean:.3f}m  p95 {e_p95:.3f}m  max {e_max:.3f}m')
        self.get_logger().info(
            f'EKF estimate  vs ground truth: mean {g_mean:.3f}m  p95 {g_p95:.3f}m  max {g_max:.3f}m')

        row_spacing = 0.42 * (1.0 - 0.15)
        if g_mean > 0.5 * row_spacing and e_mean < t_mean:
            verdict = ('LOCALIZATION: the EKF tracks the path better than ground truth does, '
                       'and the two poses disagree by more than half a row - nav2 is steering '
                       'to a pose the robot is not actually at.')
        elif t_mean > 0.5 * row_spacing:
            verdict = ('CONTROLLER TRACKING: both pose sources agree, but the robot is off the '
                       'planned path by more than half a row - the controller is steering badly '
                       '(lookahead vs row spacing is the usual cause).')
        else:
            verdict = ('Tracking is within half a row spacing on both sources - the messiness is '
                       'probably not cross-track error; look at path shape/coverage instead.')
        self.get_logger().info(f'verdict -> {verdict}')


def main():
    rclpy.init()
    node = PathTrackingMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.report()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
