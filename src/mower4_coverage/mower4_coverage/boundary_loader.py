#!/usr/bin/env python3

import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from geometry_msgs.msg import Point32, PolygonStamped
from pymap3d import geodetic2enu

class BoundaryLoader(Node):
    def __init__(self):
        super().__init__("boundary_loader")

        self.declare_parameter("boundary_file", "")
        self.declare_parameter("map_frame", "map")
        boundary_file = self.get_parameter("boundary_file").value
        self.map_frame = self.get_parameter("map_frame").value

        with open(boundary_file) as file:
            data = yaml.safe_load(file)

        datum = data["datum"]

        latched_qos = QoSProfile(depth=1)
        latched_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.pub = self.create_publisher(PolygonStamped, "/mow_boundary", latched_qos)

        msg = PolygonStamped()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()

        for c in data["corners"]:
            x, y, _ = geodetic2enu(
                c['lat'], c['lon'], 0.0, 
                datum['lat'], datum['lon'], 0.0)   

            msg.polygon.points.append(Point32(x = float(x), y = float(y), z = 0.0)) 

        self.pub.publish(msg)
        corner_count = len(data["corners"])
        self.get_logger().info(f"published mow boundary: {corner_count} corners")

def main():
    rclpy.init()
    node = BoundaryLoader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()    