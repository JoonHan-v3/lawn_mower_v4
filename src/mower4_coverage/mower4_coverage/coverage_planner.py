#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from geometry_msgs.msg import PolygonStamped, PoseStamped
from nav_msgs.msg import Path
from shapely.geometry import LineString, Polygon
from shapely.affinity import rotate as shapely_rotate

def longest_edge_angle(coords):
    """Angle of the Polygon's longest edge, so swaths align with 
    the field's dominant direction instead of a fixed world axis"""

    best_len, best_angle = -1.0, 0.0
    for (x0, y0), (x1, y1) in zip(coords, coords[1:] + coords[:1]):
        length = math.hypot(x1 - x0, y1 - y0)
        if length > best_len:
            best_len, best_angle = length, math.atan2(y1 - y0, x1 - x0)

    return best_angle


def generate_boustrophedon(polygon: Polygon, row_spacing: float):
    coords = list(polygon.exterior.coords)[:-1]
    angle = longest_edge_angle(coords)
    centroid = polygon.centroid
    aligned = shapely_rotate(polygon, -angle, origin=centroid, use_radians=True)

    minx, miny, maxx, maxy = aligned.bounds
    rows = []
    y = miny + row_spacing / 2.0

    while y <= maxy:
        line = LineString([(minx - 1.0, y), (maxx + 1.0, y)])
        clipped = line.intersection(aligned)

        if not clipped.is_empty:
            if clipped.geom_type == "LineString":
                segments = [clipped]
            elif clipped.geom_type in ("MultiLineString", "GeometryCollection"):
                segments = [g for g in clipped.geoms if g.geom_type == "LineString"]
            else:
                # tangent touch (Point/MultiPoint) at this row height: no usable swath
                segments = []
            for seg in segments:
                if seg.length > 1e-6:
                    rows.append(list(seg.coords))
        y += row_spacing

    # serpentine ordering: reverse every other row instead of jumping back
    ordered = [row if i % 2 == 0 else list(reversed(row)) for i, row in enumerate(rows)]
    points = [pt for row in ordered for pt in row]

    unrotated = shapely_rotate(LineString(points), angle, origin=centroid, use_radians=True)
    return list(unrotated.coords)


def densify(points, spacing):
    """Resample a corner-to-corner path so consecutive poses are `spacing` apart.

    The raw boustrophedon path has only the two endpoints per row, ~19m apart.
    RegulatedPurePursuitController clips the path it follows to the local
    costmap window (2.5m here), so a gap that large leaves it with a lookahead
    carrot on top of the robot and no usable heading: it stops dead at the row
    start. Controllers expect planner-resolution paths, so emit one.
    """
    if len(points) < 2:
        return list(points)

    dense = [points[0]]
    for (x0, y0), (x1, y1) in zip(points, points[1:]):
        seg_len = math.hypot(x1 - x0, y1 - y0)
        steps = max(1, int(math.ceil(seg_len / spacing)))
        for i in range(1, steps + 1):
            t = i / steps
            dense.append((x0 + (x1 - x0) * t, y0 + (y1 - y0) * t))

    return dense


class CoveragePlanner(Node):
    def __init__(self):
        super().__init__("coverage_planner")

        self.declare_parameter("cutting_width", 0.42)
        self.declare_parameter("overlap", 0.15)
        self.declare_parameter("boundary_inset", 0.5)
        self.declare_parameter("point_spacing", 0.2)
        self.declare_parameter("map_frame", "map")

        cutting_width = self.get_parameter("cutting_width").value
        overlap = self.get_parameter("overlap").value
        self.boundary_inset = self.get_parameter("boundary_inset").value
        self.point_spacing = self.get_parameter("point_spacing").value
        self.map_frame = self.get_parameter("map_frame").value
        self.row_spacing = cutting_width * (1.0 - overlap)

        latched_qos = QoSProfile(depth = 1)
        latched_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.path_pub = self.create_publisher(Path, "/coverage_path", latched_qos)
        self.create_subscription(PolygonStamped, "/mow_boundary", self.on_boundary, latched_qos)

    def on_boundary(self, msg: PolygonStamped):
        coords = [(p.x, p.y) for p in msg.polygon.points]
        if len(coords) < 3:
            self.get_logger().error("mow_boundary needs at least 3 points")
            return

        polygon = Polygon(coords)
        if not polygon.is_valid:
            polygon = polygon.buffer(0)

        coverage_polygon = polygon
        if self.boundary_inset > 0.0:
            coverage_polygon = polygon.buffer(-self.boundary_inset)
            if coverage_polygon.is_empty:
                self.get_logger().error(
                    "boundary_inset is larger than the mow boundary can support"
                )
                return

        corners = generate_boustrophedon(coverage_polygon, self.row_spacing)
        if len(corners) < 2:
            self.get_logger().error("empty coverage path - check cutting_width/overlap and the boundary")
            return

        points = densify(corners, self.point_spacing)

        path = Path()
        path.header.frame_id = self.map_frame
        path.header.stamp = self.get_clock().now().to_msg()
        yaw = 0.0

        for i, (x, y) in enumerate(points):
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = x
            pose.pose.position.y = y

            if i + 1 < len(points):
                yaw = math.atan2(points[i + 1][1] - y, points[i + 1][0] - x)

            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            path.poses.append(pose)

        self.path_pub.publish(path)
        self.get_logger().info(
            f"published coverage path: {len(points)} points "
            f"({len(corners)} row corners at {self.point_spacing:.2f}m spacing), "
            f"{self.row_spacing:.2f}m row spacing, {self.boundary_inset:.2f}m boundary inset"
        )

def main():
    rclpy.init()
    node = CoveragePlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
