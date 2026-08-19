#!/usr/bin/python3

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from geometry_msgs.msg import Point, PolygonStamped
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import Marker, MarkerArray
from shapely.geometry import Point as ShapelyPoint, Polygon
from shapely.ops import unary_union


def rotate_by_quaternion(v, q):
    """Rotate vector v=(x,y,z) by quaternion q=(x,y,z,w)."""
    vx, vy, vz = v
    qx, qy, qz, qw = q
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    rx = vx + qw * tx + (qy * tz - qz * ty)
    ry = vy + qw * ty + (qz * tx - qx * tz)
    rz = vz + qw * tz + (qx * ty - qy * tx)
    return rx, ry, rz


class CoverageChecker(Node):
    """Independently reconstructs the mowed-area union from ground truth
    and compares it against /mow_boundary, so 'full coverage' has a number
    (and a visible gap outline) instead of relying on eyeballing the
    mowed_patch_N discs in the Gazebo viewport."""

    def __init__(self):
        super().__init__('coverage_checker')

        self.declare_parameter('deck_offset_x', 0.16)
        self.declare_parameter('deck_offset_z', 0.055)
        self.declare_parameter('blade_joint', 'blade_joint')
        self.declare_parameter('blade_velocity_threshold', 0.1)
        self.declare_parameter('patch_radius', 0.24)
        self.declare_parameter('trail_spacing', 0.10)
        self.declare_parameter('coverage_threshold', 0.95)

        self.deck_local_offset = (
            self.get_parameter('deck_offset_x').value,
            0.0,
            self.get_parameter('deck_offset_z').value,
        )
        self.blade_joint = self.get_parameter('blade_joint').value
        self.blade_velocity_threshold = self.get_parameter('blade_velocity_threshold').value
        self.patch_radius = self.get_parameter('patch_radius').value
        self.trail_spacing = self.get_parameter('trail_spacing').value
        self.coverage_threshold = self.get_parameter('coverage_threshold').value

        latched_qos = QoSProfile(depth=1)
        latched_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        self.boundary_polygon = None
        self.boundary_area = 0.0
        self.create_subscription(
            PolygonStamped, '/mow_boundary', self.on_boundary, latched_qos)

        self.blade_velocity = 0.0
        self.create_subscription(JointState, 'joint_states', self.on_joint_state, 10)

        self.robot_pose = None
        self.create_subscription(TFMessage, 'tf_ground_truth', self.on_ground_truth, 10)

        self.last_xy = None
        self.trail_points = []

        self.gap_pub = self.create_publisher(MarkerArray, '/coverage_gaps', latched_qos)

        self.create_timer(1.0 / 5.0, self.on_timer)
        self.get_logger().info('coverage_checker started, waiting for /mow_boundary')

    def on_boundary(self, msg: PolygonStamped):
        coords = [(p.x, p.y) for p in msg.polygon.points]
        polygon = Polygon(coords)
        if not polygon.is_valid:
            polygon = polygon.buffer(0)
        self.boundary_polygon = polygon
        self.boundary_area = polygon.area
        self.get_logger().info(f'boundary received: {self.boundary_area:.1f} m^2')

    def on_joint_state(self, msg):
        try:
            idx = msg.name.index(self.blade_joint)
        except ValueError:
            return
        if idx < len(msg.velocity):
            self.blade_velocity = msg.velocity[idx]

    def on_ground_truth(self, msg):
        if not msg.transforms:
            return
        t = msg.transforms[0]
        p = t.transform.translation
        q = t.transform.rotation
        self.robot_pose = ((p.x, p.y, p.z), (q.x, q.y, q.z, q.w))

    def on_timer(self):
        if abs(self.blade_velocity) < self.blade_velocity_threshold:
            self.last_xy = None
            return
        if self.robot_pose is None:
            return

        position, orientation = self.robot_pose
        offset = rotate_by_quaternion(self.deck_local_offset, orientation)
        x = position[0] + offset[0]
        y = position[1] + offset[1]

        if self.last_xy is not None:
            dx, dy = x - self.last_xy[0], y - self.last_xy[1]
            if math.hypot(dx, dy) < self.trail_spacing:
                return

        self.last_xy = (x, y)
        self.trail_points.append((x, y))

    def report(self):
        if self.boundary_polygon is None:
            self.get_logger().error('coverage: no /mow_boundary received, cannot report')
            return
        if not self.trail_points:
            self.get_logger().error(
                'coverage: 0.0% - no trail points recorded (was the blade spinning?)')
            return

        circles = [ShapelyPoint(x, y).buffer(self.patch_radius) for x, y in self.trail_points]
        mowed = unary_union(circles)
        covered = mowed.intersection(self.boundary_polygon)
        pct = 100.0 * covered.area / self.boundary_area
        gap = self.boundary_polygon.difference(mowed)

        if pct >= self.coverage_threshold * 100.0:
            self.get_logger().info(
                f'COVERAGE PASS: {pct:.1f}% covered ({len(self.trail_points)} trail points, '
                f'gap {gap.area:.2f} m^2)')
        else:
            self.get_logger().error(
                f'COVERAGE FAIL: {pct:.1f}% covered, threshold {self.coverage_threshold * 100:.0f}%, '
                f'gap {gap.area:.2f} m^2')

        self.publish_gap_markers(gap)

    def publish_gap_markers(self, gap):
        if gap.is_empty:
            polys = []
        elif gap.geom_type == 'Polygon':
            polys = [gap]
        elif gap.geom_type in ('MultiPolygon', 'GeometryCollection'):
            polys = [g for g in gap.geoms if g.geom_type == 'Polygon']
        else:
            polys = []

        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        for i, poly in enumerate(polys):
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'coverage_gaps'
            m.id = i
            m.type = Marker.LINE_STRIP
            m.action = Marker.ADD
            m.scale.x = 0.05
            m.color.r = 1.0
            m.color.a = 1.0
            for x, y in poly.exterior.coords:
                m.points.append(Point(x=float(x), y=float(y), z=0.05))
            markers.markers.append(m)

        self.gap_pub.publish(markers)
        self.get_logger().info(f'published {len(polys)} gap outline(s) to /coverage_gaps')


def main():
    rclpy.init()
    node = CoverageChecker()
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
