#!/usr/bin/python3

import math
import subprocess

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage

BATCH_SDF_TEMPLATE = (
    "<?xml version='1.0'?>"
    "<sdf version='1.9'>"
    "<model name='{name}'>"
    "<static>true</static>"
    "<link name='link'>"
    "{visuals}"
    "</link>"
    "</model>"
    "</sdf>"
)

PATCH_VISUAL_TEMPLATE = (
    "<visual name='v{index}'>"
    "<pose>{dx} {dy} 0 0 0 0</pose>"
    "<geometry><cylinder><radius>{radius}</radius><length>0.002</length></cylinder></geometry>"
    "<material>"
    "<ambient>0.35 0.55 0.20 1</ambient>"
    "<diffuse>0.45 0.65 0.25 1</diffuse>"
    "</material>"
    "</visual>"
)


def rotate_by_quaternion(v, q):
    """Rotate vector v=(x,y,z) by quaternion q=(x,y,z,w)."""
    vx, vy, vz = v
    qx, qy, qz, qw = q
    # t = 2 * cross(q_xyz, v)
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    # v' = v + w*t + cross(q_xyz, t)
    rx = vx + qw * tx + (qy * tz - qz * ty)
    ry = vy + qw * ty + (qz * tx - qx * tz)
    rz = vz + qw * tz + (qx * ty - qy * tx)
    return rx, ry, rz


class GrassMower(Node):

    def __init__(self):
        super().__init__('grass_mower')

        self.declare_parameter('world_name', 'lawn_world')
        self.declare_parameter('deck_offset_x', 0.16)
        self.declare_parameter('deck_offset_z', 0.055)
        self.declare_parameter('blade_joint', 'blade_joint')
        self.declare_parameter('blade_velocity_threshold', 0.1)
        self.declare_parameter('patch_radius', 0.24)
        # Painted patches overlap, so this is bounded by geometry rather than
        # taste: between two patches the band narrows to
        # 2*sqrt(patch_radius^2 - (trail_spacing/2)^2), which must stay wider
        # than the planner's row spacing (0.357m) or adjacent rows leave a
        # visible gap. That breaks down at ~0.32; 0.25 keeps ~5cm of margin
        # while cutting the rendered visual count 2.5x versus 0.10.
        self.declare_parameter('trail_spacing', 0.25)
        self.declare_parameter('update_rate', 5.0)
        self.declare_parameter('batch_size', 25)

        self.world_name = self.get_parameter('world_name').value
        self.deck_local_offset = (
            self.get_parameter('deck_offset_x').value,
            0.0,
            self.get_parameter('deck_offset_z').value,
        )
        self.blade_joint = self.get_parameter('blade_joint').value
        self.blade_velocity_threshold = self.get_parameter('blade_velocity_threshold').value
        self.patch_radius = self.get_parameter('patch_radius').value
        self.trail_spacing = self.get_parameter('trail_spacing').value
        update_rate = self.get_parameter('update_rate').value
        self.batch_size = self.get_parameter('batch_size').value

        self.blade_velocity = 0.0
        self.joint_state_sub = self.create_subscription(
            JointState, 'joint_states', self.on_joint_state, 10)

        self.robot_pose = None
        self.ground_truth_sub = self.create_subscription(
            TFMessage, 'tf_ground_truth', self.on_ground_truth, 10)

        self.last_xy = None
        self.batch_count = 0
        # One Gazebo entity per 0.10m of travel is thousands of entities over a
        # full mow, and each spawn used to block this callback on a `gz` CLI
        # round trip. Patches are accumulated and spawned as one model holding
        # many visuals, via a non-blocking subprocess that is reaped later.
        self.pending = []
        self.spawns = []

        self.timer = self.create_timer(1.0 / update_rate, self.on_timer)
        self.get_logger().info('grass_mower started, painting mowed trail behind the mower deck')

    def on_joint_state(self, msg):
        try:
            idx = msg.name.index(self.blade_joint)
        except ValueError:
            return
        if idx < len(msg.velocity):
            self.blade_velocity = msg.velocity[idx]

    def on_ground_truth(self, msg):
        # The bridged gz.msgs.Pose_V -> tf2_msgs/TFMessage conversion for
        # /world/.../dynamic_pose/info doesn't carry the per-pose names
        # through (frame_id/child_frame_id come out empty), unlike the
        # DiffDrive-published /model/.../tf topic. The robot model is always
        # the first entry in this world's dynamic pose list (verified: its
        # own links follow immediately after), so index 0 is used instead.
        if not msg.transforms:
            return
        t = msg.transforms[0]
        p = t.transform.translation
        q = t.transform.rotation
        self.robot_pose = ((p.x, p.y, p.z), (q.x, q.y, q.z, q.w))

    def on_timer(self):
        self.spawns = [p for p in self.spawns if p.poll() is None]

        if abs(self.blade_velocity) < self.blade_velocity_threshold:
            # blade isn't cutting: don't paint a trail, and don't count
            # movement that happened while it was off once it starts again
            self.last_xy = None
            # flush what's queued so the painted trail ends where cutting did
            self.flush()
            return

        if self.robot_pose is None:
            return

        position, orientation = self.robot_pose
        offset = rotate_by_quaternion(self.deck_local_offset, orientation)
        x = position[0] + offset[0]
        y = position[1] + offset[1]

        if self.last_xy is not None:
            dx = x - self.last_xy[0]
            dy = y - self.last_xy[1]
            if math.hypot(dx, dy) < self.trail_spacing:
                return

        self.last_xy = (x, y)
        self.pending.append((x, y))

        if len(self.pending) >= self.batch_size:
            self.flush()

    def flush(self):
        if not self.pending:
            return

        points = self.pending
        self.pending = []

        # the model is spawned at the first point; the rest are visuals offset
        # from it, so the painted result is identical to one model per patch
        origin_x, origin_y = points[0]
        visuals = ''.join(
            PATCH_VISUAL_TEMPLATE.format(
                index=i,
                dx=px - origin_x,
                dy=py - origin_y,
                radius=self.patch_radius,
            )
            for i, (px, py) in enumerate(points)
        )

        name = f'mowed_patch_batch_{self.batch_count}'
        self.batch_count += 1
        sdf = BATCH_SDF_TEMPLATE.format(name=name, visuals=visuals)
        req = (
            f"sdf: \"{sdf}\" "
            f"pose: {{position: {{x: {origin_x}, y: {origin_y}, z: 0.001}}}} "
            f"name: \"{name}\""
        )

        try:
            self.spawns.append(subprocess.Popen(
                [
                    'gz', 'service', '-s', f'/world/{self.world_name}/create',
                    '--reqtype', 'gz.msgs.EntityFactory',
                    '--reptype', 'gz.msgs.Boolean',
                    '--timeout', '300',
                    '--req', req,
                ],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            ))
        except FileNotFoundError:
            self.get_logger().error('gz executable not found; cannot spawn mowed patches')


def main():
    rclpy.init()
    node = GrassMower()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.flush()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
