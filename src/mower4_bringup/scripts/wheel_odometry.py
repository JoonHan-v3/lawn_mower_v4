#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion


class WheelOdometry(Node):
    def __init__(self):
        super().__init__("wheel_odometry")

        self.declare_parameter("wheel_separation", 0.36)
        self.declare_parameter("wheel_radius", 0.08)
        self.declare_parameter("left_wheel_joint", "left_wheel_joint")
        self.declare_parameter("right_wheel_joint", "right_wheel_joint")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("publish_rate", 50.0)

        self.wheel_separation = self.get_parameter("wheel_separation").value
        self.wheel_radius = self.get_parameter("wheel_radius").value
        self.left_joint = self.get_parameter("left_wheel_joint").value
        self.right_joint = self.get_parameter("right_wheel_joint").value
        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value

        publish_rate = self.get_parameter("publish_rate").value
        self.publish_period = 1.0 / publish_rate if publish_rate > 0.0 else 0.0

        self.sub = self.create_subscription(JointState, "/joint_states", self.joint, 10)
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)

        self.prev_time = None
        self.prev_left = None
        self.prev_right = None
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # /joint_states arrives far faster than any consumer needs (hundreds of
        # Hz from the bridge). Pose is integrated on every sample so nothing is
        # lost, but publishing is capped, and the twist is averaged over the
        # whole publish window instead of one tiny dt - which is both cheaper
        # and less noisy for the EKF than the raw per-sample rate.
        self.acc_dt = 0.0
        self.acc_s = 0.0
        self.acc_theta = 0.0

    def joint(self, msg):
        names = msg.name
        positions = msg.position

        try:
            l_idx = names.index(self.left_joint)
            r_idx = names.index(self.right_joint)
        except ValueError as e:
            self.get_logger().warn(f"Joint names not found: {e}")
            return

        left = positions[l_idx]
        right = positions[r_idx]
        now = self.get_clock().now()


        if self.prev_time is None:
            self.prev_time = now
            self.prev_left = left
            self.prev_right = right
            return

        dt = (now - self.prev_time).nanoseconds / 1e9
        self.prev_time = now

        if dt <= 0.0:
            return

        dl = self.wheel_radius * (left - self.prev_left)
        dr = self.wheel_radius * (right - self.prev_right)

        self.prev_left = left
        self.prev_right = right

        ds = 0.5 * (dl + dr)
        dtheta = (dr - dl) / self.wheel_separation

        self.x += ds * math.cos(self.theta + dtheta / 2.0)
        self.y += ds * math.sin(self.theta + dtheta / 2.0)
        self.theta = self.normalize_angle(self.theta + dtheta)

        self.acc_dt += dt
        self.acc_s += ds
        self.acc_theta += dtheta

        if self.acc_dt < self.publish_period:
            return

        window_dt = self.acc_dt
        window_s = self.acc_s
        window_theta = self.acc_theta
        self.acc_dt = 0.0
        self.acc_s = 0.0
        self.acc_theta = 0.0

        q = Quaternion()
        q.z = math.sin(self.theta / 2.0)
        q.w = math.cos(self.theta / 2.0)

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = q
        odom.pose.covariance = [
            0.05, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.05, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1e3, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1e3, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 1e3, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.10,
        ]

        odom.twist.twist.linear.x = window_s / window_dt
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = window_theta / window_dt
        odom.twist.covariance = [
            0.02, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.02, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1e3, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1e3, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 1e3, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.05,
        ]
        self.odom_pub.publish(odom)

    def normalize_angle(self, a):
        while a > math.pi:
            a -= 2.0 * math.pi
        while a < -math.pi:
            a += 2.0 * math.pi

        return a

    
def main():
    rclpy.init()
    node = WheelOdometry()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
