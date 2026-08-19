#!/usr/bin/python3

"""Drops obstacles onto the lawn so the replanner has something to route around.

The coverage sweep is planned from the boundary polygon alone, so an obstacle
only becomes interesting once it is standing in the middle of a row the mower
has already committed to. Spawning them from here (rather than writing them
into worlds/lawn_field.sdf) keeps the default field clear and lets them appear
part-way through a run, which is the case coverage_executor.py exists for.
"""

import subprocess

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

OBSTACLE_SDF_TEMPLATE = (
    "<?xml version='1.0'?>"
    "<sdf version='1.9'>"
    "<model name='{name}'>"
    "<static>true</static>"
    "<link name='link'>"
    "<collision name='collision'>"
    "<geometry><cylinder><radius>{radius}</radius><length>{height}</length></cylinder></geometry>"
    "</collision>"
    "<visual name='visual'>"
    "<geometry><cylinder><radius>{radius}</radius><length>{height}</length></cylinder></geometry>"
    "<material>"
    "<ambient>0.20 0.12 0.06 1</ambient>"
    "<diffuse>0.35 0.22 0.10 1</diffuse>"
    "</material>"
    "</visual>"
    "</link>"
    "</model>"
    "</sdf>"
)


def parse_obstacles(spec):
    """Parse 'x,y[,radius[,height]]; x,y,...' into a list of tuples."""
    obstacles = []
    for i, chunk in enumerate(spec.split(";")):
        chunk = chunk.strip()
        if not chunk:
            continue
        fields = [float(f) for f in chunk.split(",")]
        if len(fields) < 2:
            raise ValueError(f"obstacle {i} needs at least an x and a y: '{chunk}'")
        x, y = fields[0], fields[1]
        radius = fields[2] if len(fields) > 2 else 0.35
        height = fields[3] if len(fields) > 3 else 0.8
        obstacles.append((x, y, radius, height))
    return obstacles


class ObstacleSpawner(Node):

    def __init__(self):
        super().__init__("obstacle_spawner")

        self.declare_parameter("world_name", "lawn_world")
        # three spots inside the default L-shaped boundary (10m x 10m with the
        # x>1, y>1 quadrant notched out), far enough apart to land on
        # different rows of the sweep whichever way round it runs
        self.declare_parameter("obstacles", "-1.5,-2.0,0.5; 2.5,-3.5,0.6; -3.0,2.0,0.5")
        self.declare_parameter("spawn_delay", 0.0)

        self.world_name = self.get_parameter("world_name").value
        self.obstacles = parse_obstacles(self.get_parameter("obstacles").value)
        delay = self.get_parameter("spawn_delay").value

        if not self.obstacles:
            self.get_logger().warn("no obstacles configured, nothing to spawn")
            return

        if delay > 0.0:
            self.get_logger().info(
                f"spawning {len(self.obstacles)} obstacle(s) in {delay:.0f}s")
            self.timer = self.create_timer(delay, self.on_timer)
        else:
            self.spawn_all()

    def on_timer(self):
        self.timer.cancel()
        self.spawn_all()

    def spawn_all(self):
        for i, (x, y, radius, height) in enumerate(self.obstacles):
            self.spawn(f"obstacle_{i}", x, y, radius, height)

    def spawn(self, name, x, y, radius, height):
        sdf = OBSTACLE_SDF_TEMPLATE.format(name=name, radius=radius, height=height)
        req = (
            f"sdf: \"{sdf}\" "
            f"pose: {{position: {{x: {x}, y: {y}, z: {height / 2.0}}}}} "
            f"name: \"{name}\""
        )

        try:
            result = subprocess.run(
                [
                    "gz", "service", "-s", f"/world/{self.world_name}/create",
                    "--reqtype", "gz.msgs.EntityFactory",
                    "--reptype", "gz.msgs.Boolean",
                    "--timeout", "3000",
                    "--req", req,
                ],
                capture_output=True,
                text=True,
            )
        except FileNotFoundError:
            self.get_logger().error("gz executable not found; cannot spawn obstacles")
            return

        if "true" in result.stdout.lower():
            self.get_logger().info(
                f"spawned {name}: r={radius:.2f}m h={height:.2f}m at ({x:.2f}, {y:.2f})")
        else:
            self.get_logger().error(
                f"failed to spawn {name}: {result.stdout.strip()} {result.stderr.strip()}")


def main():
    rclpy.init()
    node = ObstacleSpawner()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
