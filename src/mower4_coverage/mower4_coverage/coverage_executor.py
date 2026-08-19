#!/usr/bin/env python3

"""Drives the coverage path, routing around whatever the costmap says is in
the way instead of stopping at it.

The sweep from coverage_planner.py is computed once, before the run, from the
boundary polygon alone - it knows nothing about the bush that grew into the
lawn or the chair someone left on it. This node follows that sweep while
watching the part of it that is still ahead against the global costmap. When a
stretch comes up blocked it picks the first point past the obstacle where the
sweep is clear again, asks the global planner for a route from wherever the
robot is to that point, and follows the detour spliced onto the rest of the
sweep. Only the stretch the obstacle physically occupies is given up.
"""

import math

import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from geometry_msgs.msg import Point, PoseStamped
from nav2_msgs.action import BackUp, ComputePathToPose, FollowPath
from nav2_msgs.msg import Costmap, CostmapUpdate
from nav_msgs.msg import Path
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

# how close to the end of the sweep counts as having driven all of it
COMPLETION_TOLERANCE = 1.0


class CostmapGrid:
    """Random-access view of a nav2 costmap.

    Kept current from the full `costmap_raw` message plus the
    `costmap_raw_updates` patches that follow it - with the default
    `always_send_full_costmap: false`, nav2 sends the whole grid once and then
    only the changed window, so a node that reads `costmap_raw` alone sees an
    empty field forever.
    """

    def __init__(self, msg: Costmap):
        metadata = msg.metadata
        self.resolution = metadata.resolution
        self.size_x = metadata.size_x
        self.size_y = metadata.size_y
        self.origin_x = metadata.origin.position.x
        self.origin_y = metadata.origin.position.y
        self.data = bytearray(msg.data)

    def fits(self, msg: CostmapUpdate) -> bool:
        return (msg.x + msg.size_x <= self.size_x and msg.y + msg.size_y <= self.size_y)

    def apply(self, msg: CostmapUpdate):
        patch = bytes(msg.data)
        for row in range(msg.size_y):
            src = row * msg.size_x
            dst = (msg.y + row) * self.size_x + msg.x
            self.data[dst:dst + msg.size_x] = patch[src:src + msg.size_x]

    def cost(self, x, y):
        """Cost at a world point, or None where the point falls off the grid."""
        i = int((x - self.origin_x) / self.resolution)
        j = int((y - self.origin_y) / self.resolution)
        if not (0 <= i < self.size_x and 0 <= j < self.size_y):
            return None
        return self.data[j * self.size_x + i]


class CoverageExecutor(Node):

    def __init__(self):
        super().__init__("coverage_executor")

        self.declare_parameter("map_frame", "map")
        self.declare_parameter("robot_frame", "base_footprint")
        self.declare_parameter("costmap_topic", "/global_costmap/costmap_raw")
        self.declare_parameter("planner_id", "GridBased")
        self.declare_parameter("controller_id", "FollowPath")
        self.declare_parameter("goal_checker_id", "goal_checker")
        self.declare_parameter("progress_checker_id", "progress_checker")
        self.declare_parameter("supervision_rate", 2.0)
        self.declare_parameter("check_horizon", 3.0)
        self.declare_parameter("blocked_cost", 150)
        self.declare_parameter("clear_run", 0.5)
        self.declare_parameter("rejoin_advance", 0.5)
        self.declare_parameter("max_replan_attempts", 6)
        self.declare_parameter("replan_cooldown", 2.0)
        self.declare_parameter("progress_window", 4.0)
        self.declare_parameter("rejoin_tolerance", 0.6)
        self.declare_parameter("rejoin_timeout", 90.0)
        self.declare_parameter("stuck_skip", 2.0)
        self.declare_parameter("max_recoveries", 3)
        self.declare_parameter("backup_distance", 0.4)
        self.declare_parameter("backup_speed", 0.15)

        self.map_frame = self.get_parameter("map_frame").value
        self.robot_frame = self.get_parameter("robot_frame").value
        costmap_topic = self.get_parameter("costmap_topic").value
        self.planner_id = self.get_parameter("planner_id").value
        self.controller_id = self.get_parameter("controller_id").value
        self.goal_checker_id = self.get_parameter("goal_checker_id").value
        self.progress_checker_id = self.get_parameter("progress_checker_id").value
        supervision_rate = self.get_parameter("supervision_rate").value
        self.check_horizon = self.get_parameter("check_horizon").value
        self.blocked_cost = self.get_parameter("blocked_cost").value
        self.clear_run = self.get_parameter("clear_run").value
        self.rejoin_advance = self.get_parameter("rejoin_advance").value
        self.max_replan_attempts = self.get_parameter("max_replan_attempts").value
        self.replan_cooldown = self.get_parameter("replan_cooldown").value
        self.progress_window = self.get_parameter("progress_window").value
        self.rejoin_tolerance = self.get_parameter("rejoin_tolerance").value
        self.rejoin_timeout = self.get_parameter("rejoin_timeout").value
        self.stuck_skip = self.get_parameter("stuck_skip").value
        self.max_recoveries = self.get_parameter("max_recoveries").value
        self.backup_distance = self.get_parameter("backup_distance").value
        self.backup_speed = self.get_parameter("backup_speed").value

        self.follow_client = ActionClient(self, FollowPath, "follow_path")
        self.compute_client = ActionClient(self, ComputePathToPose, "compute_path_to_pose")
        self.backup_client = ActionClient(self, BackUp, "backup")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        latched_qos = QoSProfile(depth=1)
        latched_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        self.create_subscription(Path, "/coverage_path", self.on_path, latched_qos)
        self.create_subscription(Costmap, costmap_topic, self.on_costmap, latched_qos)
        self.create_subscription(
            CostmapUpdate, costmap_topic + "_updates", self.on_costmap_update, 10)

        self.active_path_pub = self.create_publisher(Path, "/active_path", latched_qos)
        self.skipped_pub = self.create_publisher(MarkerArray, "/coverage_skipped", latched_qos)

        # the sweep is held as plain coordinates rather than the received
        # PoseStamped list: every replan rebuilds a message from it, and
        # reusing the received poses would mean handing nav2 a path whose
        # headers are shared with the next one built
        self.xy = []
        self.yaw = []
        self.cum = []

        self.costmap = None
        self.state = "waiting"
        self.cursor = 0
        # index the robot is currently detouring towards; while it is set the
        # robot is deliberately off the sweep, so the horizon check is paused
        self.pending_rejoin = None
        self.pending_since = 0.0
        self.plan_target = 0
        self.plan_lead_in = None
        self.plan_lead_from = 0
        self.plan_attempt = 0
        self.min_rejoin = 0
        self.last_replan = 0.0
        self.goal_handle = None
        self.recoveries = 0
        self.skips_without_progress = 0
        self.queued_engage = None
        self.last_failure_at = -math.inf
        self.backed_up = False
        self.detours = 0
        self.recoveries_total = 0
        self.skipped = []

        self.create_timer(1.0 / supervision_rate, self.supervise)

    # ---------------------------------------------------------------- inputs

    def on_path(self, msg: Path):
        if self.xy:
            return

        self.xy = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        if len(self.xy) < 2:
            self.get_logger().error("coverage path has fewer than 2 poses, nothing to drive")
            return

        self.yaw = []
        for i, (x, y) in enumerate(self.xy):
            nxt = self.xy[min(i + 1, len(self.xy) - 1)]
            if nxt == (x, y) and i > 0:
                self.yaw.append(self.yaw[-1])
            else:
                self.yaw.append(math.atan2(nxt[1] - y, nxt[0] - x))

        self.cum = [0.0]
        for (x0, y0), (x1, y1) in zip(self.xy, self.xy[1:]):
            self.cum.append(self.cum[-1] + math.hypot(x1 - x0, y1 - y0))

        self.get_logger().info(
            f"received coverage path: {len(self.xy)} points, {self.cum[-1]:.0f}m of sweep")
        self.engage(0, "driving to the start of the coverage path")

    def on_costmap(self, msg: Costmap):
        self.costmap = CostmapGrid(msg)

    def on_costmap_update(self, msg: CostmapUpdate):
        if self.costmap is None:
            return
        if not self.costmap.fits(msg):
            # the costmap was resized and the full grid that goes with this
            # patch has not arrived yet; drop it rather than blit out of bounds
            return
        self.costmap.apply(msg)

    def robot_xy(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.robot_frame, rclpy.time.Time())
        except TransformException:
            return None
        return (tf.transform.translation.x, tf.transform.translation.y)

    # ------------------------------------------------------- path arithmetic

    def index_after(self, idx, distance):
        """First index at least `distance` metres further along the sweep."""
        target = self.cum[idx] + distance
        j = idx
        while j + 1 < len(self.cum) and self.cum[j] < target:
            j += 1
        return j

    def is_blocked(self, idx):
        if self.costmap is None:
            return False
        cost = self.costmap.cost(*self.xy[idx])
        # off-grid counts as clear: the sweep lives inside the boundary, which
        # the global costmap covers with margin, so this only fires on setups
        # where the costmap is too small to judge - and refusing to drive
        # there would be worse than driving there
        return cost is not None and cost >= self.blocked_cost

    def first_blocked(self, start):
        """First blocked index within `check_horizon` metres ahead of `start`."""
        limit = self.cum[start] + self.check_horizon
        for i in range(start, len(self.xy)):
            if self.cum[i] > limit:
                break
            if self.is_blocked(i):
                return i
        return None

    def first_clear_after(self, blocked_idx):
        """First index past the blockage where the sweep stays clear.

        Clear for `clear_run` metres, not just for one cell: rejoining at the
        first free cell would put the mower back on the sweep inside the
        obstacle's far edge and it would just stop there instead.
        """
        i = max(blocked_idx + 1, self.min_rejoin)
        while i < len(self.xy):
            if self.is_blocked(i):
                i += 1
                continue
            run_end = self.index_after(i, self.clear_run)
            j = i
            while j <= run_end and not self.is_blocked(j):
                j += 1
            if j > run_end:
                return i
            i = j + 1
        return None

    def project(self, xy, start):
        """Closest sweep point within `progress_window` metres after `start`.

        Forwards only, because adjacent rows sit one row spacing apart - ~0.36m
        at the defaults, less than the mower's own tracking error - so a
        nearest-point search over the whole sweep snaps onto the row it has
        already mowed and the run appears to go backwards.

        The window has to stay wide enough to follow the mower through a row-end
        turn, where it cuts the corner and leaves the sweep by more than a row
        spacing: a scan that gives up there falls a corner behind on every row,
        and once it is a whole window behind it never catches up again.
        """
        best, best_d = start, float("inf")
        limit = self.cum[start] + self.progress_window
        for i in range(start, len(self.xy)):
            if self.cum[i] > limit:
                break
            d = math.hypot(self.xy[i][0] - xy[0], self.xy[i][1] - xy[1])
            if d < best_d:
                best_d, best = d, i
        return best, best_d

    def pose_at(self, idx):
        pose = PoseStamped()
        pose.header.frame_id = self.map_frame
        pose.pose.position.x = self.xy[idx][0]
        pose.pose.position.y = self.xy[idx][1]
        pose.pose.orientation.z = math.sin(self.yaw[idx] / 2.0)
        pose.pose.orientation.w = math.cos(self.yaw[idx] / 2.0)
        return pose

    # ------------------------------------------------------------ supervision

    def now_seconds(self):
        return self.get_clock().now().nanoseconds / 1e9

    def supervise(self):
        if self.queued_engage is not None:
            self.start_queued_engage()
            return

        if self.state != "following":
            return

        xy = self.robot_xy()
        if xy is None:
            return

        if self.pending_rejoin is not None:
            self.check_rejoin(xy)
            return

        self.cursor, _ = self.project(xy, self.cursor)

        if self.costmap is None or self.now_seconds() - self.last_replan < self.replan_cooldown:
            return

        blocked = self.first_blocked(self.cursor)
        if blocked is None:
            return

        rejoin = self.first_clear_after(blocked)
        if rejoin is None:
            self.record_skip(blocked, len(self.xy) - 1)
            self.finish("the rest of the coverage path is blocked")
            return

        self.record_skip(blocked, rejoin)
        self.detours += 1
        # the obstacle is seen check_horizon metres out, but the row between
        # here and it is perfectly good grass. Mow up to the last clear point
        # and detour from there, rather than leaving the row the moment the
        # obstacle comes into view and cutting the corner off the mow.
        self.engage(
            rejoin,
            f"obstacle on the sweep {self.cum[blocked] - self.cum[self.cursor]:.1f}m ahead",
            lead_in=max(self.cursor, blocked - 1))

    def check_rejoin(self, xy):
        """Has the robot got back onto the sweep at the point it detoured to?

        Checked against a window rather than the single rejoin pose, because
        the planner's goal tolerance lets the detour end short of it and the
        controller can overshoot past it.
        """
        best, best_d = self.project(xy, self.pending_rejoin)
        if best_d > self.rejoin_tolerance:
            if self.now_seconds() - self.pending_since < self.rejoin_timeout:
                return
            # never came near the rejoin point. Carrying on with the horizon
            # check switched off for the rest of the run is the worse of the
            # two failures, so take the projection and start watching again
            self.get_logger().warn(
                f"detour never rejoined the sweep (closest approach {best_d:.1f}m); "
                f"resuming obstacle checks from point {best}")
        self.cursor = best
        self.pending_rejoin = None
        self.skips_without_progress = 0

    # -------------------------------------------------------------- planning

    def engage(self, idx, reason, lead_in=None):
        """Route to sweep index `idx`, then follow the rest of the sweep.

        With `lead_in`, the mower first drives the sweep as far as that index
        and the detour is planned from there, so the row is mowed right up to
        the obstacle. Without it the detour starts from wherever the mower is
        standing, which is what a recovery wants - it has already stopped.
        """
        if idx >= len(self.xy) - 1:
            self.finish("no coverage path left to drive")
            return

        self.state = "engaging"
        self.queued_engage = (idx, reason, lead_in)
        self.start_queued_engage()

    def start_queued_engage(self):
        """Plan the queued rejoin, once `replan_cooldown` has passed.

        The wait applies to replans asked for by a failed goal too, not just
        to the ones the horizon check asks for. A mower that cannot move from
        where it is fails the next goal the moment it starts, and with no wait
        between the two they chase each other thousands of times a minute -
        one run tore through 112m of sweep in five seconds that way, giving up
        a stretch on every pass.
        """
        if self.queued_engage is None:
            return
        if self.now_seconds() - self.last_replan < self.replan_cooldown:
            return

        idx, reason, lead_in = self.queued_engage
        self.queued_engage = None
        self.plan_target = idx
        self.plan_lead_in = lead_in
        self.plan_lead_from = self.cursor
        self.plan_attempt = 0
        self.min_rejoin = idx + 1
        self.last_replan = self.now_seconds()
        if lead_in is None:
            self.get_logger().info(
                f"{reason}: rejoining the sweep at point {idx} ({self.cum[idx]:.1f}m along)")
        else:
            self.get_logger().info(
                f"{reason}: mowing on to point {lead_in} ({self.cum[lead_in]:.1f}m along), "
                f"then detouring to rejoin at point {idx} ({self.cum[idx]:.1f}m along)")
        self.request_plan()

    def request_plan(self):
        if not self.compute_client.server_is_ready():
            self.finish("planner_server is not available")
            return
        goal = ComputePathToPose.Goal()
        goal.goal = self.pose_at(self.plan_target)
        goal.planner_id = self.planner_id
        if self.plan_lead_in is None:
            goal.use_start = False
        else:
            # planned from the far end of the lead-in rather than from the
            # robot, so the detour picks up where the mowing stops
            goal.use_start = True
            goal.start = self.pose_at(self.plan_lead_in)
        self.compute_client.send_goal_async(goal).add_done_callback(self.on_plan_response)

    def on_plan_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.finish("compute_path_to_pose goal rejected")
            return
        goal_handle.get_result_async().add_done_callback(self.on_plan_result)

    def on_plan_result(self, future):
        wrapper = future.result()
        result = wrapper.result

        if (wrapper.status == GoalStatus.STATUS_SUCCEEDED
                and result.error_code == ComputePathToPose.Result.NONE
                and result.path.poses):
            self.backed_up = False
            self.follow(result.path.poses)
            return

        self.plan_attempt += 1
        self.get_logger().warn(
            f"no route to sweep point {self.plan_target} "
            f"(status = {wrapper.status}, error_code = {result.error_code})")

        if self.plan_attempt >= self.max_replan_attempts:
            self.record_skip(self.plan_target, len(self.xy) - 1)
            self.finish("could not get back onto the coverage path")
            return

        if result.error_code == ComputePathToPose.Result.START_OCCUPIED:
            if self.plan_lead_in is not None:
                # the lead-in runs closer to the obstacle than the planner will
                # start from. Backing up cannot help - the mower is not there
                # yet - so give up the lead-in and detour from where it stands
                self.get_logger().warn(
                    "cannot plan from the end of the lead-in, detouring from here instead")
                self.plan_lead_in = None
                self.request_plan()
                return
            if not self.backed_up:
                # nosed into the obstacle's inflation: no plan starts from in
                # there, so make room before asking again
                self.backed_up = True
                self.back_up(self.request_plan)
                return

        # the rejoin point itself is unreachable - most often it sits in the
        # far edge of the same obstacle - so give up a bit more of the sweep
        nxt = self.index_after(self.plan_target, self.rejoin_advance)
        if nxt <= self.plan_target:
            self.finish("could not get back onto the coverage path")
            return
        self.record_skip(self.plan_target, nxt)
        self.plan_target = nxt
        self.min_rejoin = nxt + 1
        self.request_plan()

    # ------------------------------------------------------------- following

    def follow(self, approach_poses):
        idx = self.plan_target

        # the lead-in stops one short of where the detour was planned from, so
        # that pose comes from the detour instead of being repeated here
        lead_in_poses = []
        if self.plan_lead_in is not None:
            lead_in_poses = [self.pose_at(i)
                             for i in range(self.plan_lead_from, self.plan_lead_in)]

        path = Path()
        path.header.frame_id = self.map_frame
        path.header.stamp = self.get_clock().now().to_msg()
        # the approach already ends at sweep point idx, so the sweep resumes at
        # idx + 1: repeating the pose leaves a zero-length segment at the seam,
        # which the pure-pursuit lookahead search cannot get a heading out of
        path.poses = (lead_in_poses + list(approach_poses)
                      + [self.pose_at(i) for i in range(idx + 1, len(self.xy))])
        for pose in path.poses:
            pose.header.frame_id = path.header.frame_id
            pose.header.stamp = path.header.stamp

        if not self.follow_client.server_is_ready():
            self.finish("controller_server is not available")
            return

        sweep_points = len(path.poses) - len(approach_poses) - len(lead_in_poses)
        lead_in_note = f"{len(lead_in_poses)} of lead-in + " if lead_in_poses else ""
        self.get_logger().info(
            f"following {len(path.poses)} points "
            f"({lead_in_note}{len(approach_poses)} of detour + {sweep_points} of sweep)")

        goal = FollowPath.Goal()
        goal.path = path
        goal.controller_id = self.controller_id
        goal.goal_checker_id = self.goal_checker_id
        goal.progress_checker_id = self.progress_checker_id

        self.state = "following"
        self.cursor = idx
        self.pending_rejoin = idx
        self.pending_since = self.now_seconds()
        self.active_path_pub.publish(path)
        # dropped before the new goal goes out, not after it is accepted: nav2
        # aborts the goal it preempts, and if that abort lands first the result
        # would still match the handle here and read as a real failure
        self.goal_handle = None
        # sent without cancelling the goal it replaces: nav2's action server
        # preempts the running goal itself, and cancelling first would stop the
        # mower dead for the round trip before the detour takes over
        self.follow_client.send_goal_async(goal).add_done_callback(self.on_follow_response)

    def on_follow_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.finish("follow_path goal rejected")
            return
        self.goal_handle = goal_handle
        goal_handle.get_result_async().add_done_callback(
            lambda f, h=goal_handle: self.on_follow_result(h, f))

    def on_follow_result(self, goal_handle, future):
        # a goal that has already been replaced reports back as aborted when
        # nav2 preempts it; that is the replan working, not a failure. A None
        # handle means its replacement is still being accepted, so this result
        # belongs to the goal on the way out either way.
        if self.goal_handle is None or goal_handle is not self.goal_handle:
            return
        if self.state != "following":
            return

        wrapper = future.result()
        if wrapper.status == GoalStatus.STATUS_SUCCEEDED:
            remaining = self.cum[-1] - self.cum[self.cursor]
            xy = self.robot_xy()
            at_end = xy is not None and math.hypot(
                self.xy[-1][0] - xy[0], self.xy[-1][1] - xy[1]) <= COMPLETION_TOLERANCE
            # the cursor is an estimate and the mower's position is not, so
            # standing on the last pose having covered ground counts as done
            # even if the cursor is still catching up
            if remaining <= COMPLETION_TOLERANCE or (
                    at_end and self.cum[self.cursor] > COMPLETION_TOLERANCE):
                self.finish("coverage path complete")
                return
            # nav2's goal checker only looks at the last pose of the path.
            # Parked on top of it - which is where a finished run leaves the
            # mower - it reports the next goal complete before a wheel turns.
            self.get_logger().error(
                f"follow_path reported success with {remaining:.0f}m of sweep still ahead: "
                f"the mower is parked on the end of the coverage path, so nav2's goal "
                f"checker is satisfied before it drives anywhere. Drive it off the end "
                f"pose and start the run again.")
            self.finish("nav2 completed the goal without driving the sweep")
            return

        result = wrapper.result
        self.get_logger().warn(
            f"follow_path stopped at {self.cum[self.cursor]:.1f}m along the sweep "
            f"(status = {wrapper.status}, error_code = {result.error_code}, "
            f"error_msg = '{result.error_msg}')")
        self.handle_follow_failure()

    def handle_follow_failure(self):
        # recoveries are counted per stretch of sweep, not per run: a mower
        # that has covered ground since the last one is not the same mower
        # that is stuck, and counting them together either escalates on a
        # healthy run or never escalates on a stuck one
        self.recoveries_total += 1
        at = self.cum[self.cursor]
        if at - self.last_failure_at > self.stuck_skip:
            self.recoveries = 0
        self.last_failure_at = at
        self.recoveries += 1
        if self.recoveries > self.max_recoveries:
            # repeatedly stuck in the same spot and the costmap does not
            # explain why, so stop arguing with it and give up this stretch
            skip_to = self.index_after(self.cursor, self.stuck_skip)
            self.recoveries = 0
            if skip_to <= self.cursor:
                self.finish("stuck at the end of the coverage path")
                return
            self.skips_without_progress += 1
            if self.skips_without_progress > self.max_recoveries:
                # giving up stretch after stretch and never once rejoining the
                # sweep in between: the mower is not blocked at a place it can
                # route around, it is somewhere nav2 cannot drive it out of
                self.finish(
                    f"gave up {self.skips_without_progress} stretches in a row without "
                    f"getting back onto the sweep - the mower is stuck, not blocked")
                return
            self.record_skip(self.cursor, skip_to)
            self.engage(skip_to, f"stuck: giving up {self.stuck_skip:.1f}m of sweep")
            return

        self.state = "recovering"
        self.back_up(self.replan_after_failure)

    def replan_after_failure(self):
        self.state = "following"
        target = max(self.cursor, self.min_rejoin)
        if self.costmap is not None and self.is_blocked(target):
            clear = self.first_clear_after(target)
            if clear is None:
                self.record_skip(target, len(self.xy) - 1)
                self.finish("the rest of the coverage path is blocked")
                return
            self.record_skip(target, clear)
            target = clear
        self.engage(target, "recovering from a stopped path")

    def back_up(self, then):
        if not self.backup_client.server_is_ready():
            self.get_logger().warn("behavior_server is not available, skipping the back-up")
            then()
            return
        goal = BackUp.Goal()
        goal.target = Point(x=float(self.backup_distance))
        goal.speed = float(self.backup_speed)
        goal.time_allowance = Duration(seconds=15.0).to_msg()
        self.get_logger().info(f"backing up {self.backup_distance:.2f}m to make room")

        def on_result(future):
            wrapper = future.result()
            if wrapper.status != GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().warn(
                    f"back-up did not finish (status = {wrapper.status}, "
                    f"error_code = {wrapper.result.error_code}); replanning from here anyway")
            then()

        def on_response(future):
            handle = future.result()
            if not handle.accepted:
                self.get_logger().warn("behavior_server rejected the back-up goal")
                then()
                return
            handle.get_result_async().add_done_callback(on_result)

        self.backup_client.send_goal_async(goal).add_done_callback(on_response)

    # --------------------------------------------------------------- reporting

    def record_skip(self, start, end):
        if end <= start:
            return
        if self.skipped and start <= self.skipped[-1][1]:
            # runs into the stretch given up just before it: one continuous
            # gap in the mow rather than two, so the metres are not counted
            # twice and RViz does not draw one outline on top of another
            previous_start, previous_end = self.skipped[-1]
            self.skipped[-1] = (previous_start, max(previous_end, end))
        else:
            self.skipped.append((start, end))
        self.publish_skipped()

    def skipped_metres(self):
        return sum(self.cum[end] - self.cum[start] for start, end in self.skipped)

    def publish_skipped(self):
        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        for i, (start, end) in enumerate(self.skipped):
            marker = Marker()
            marker.header.frame_id = self.map_frame
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "coverage_skipped"
            marker.id = i
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.scale.x = 0.08
            marker.color.r = 1.0
            marker.color.a = 1.0
            for j in range(start, end + 1):
                marker.points.append(Point(x=self.xy[j][0], y=self.xy[j][1], z=0.05))
            markers.markers.append(marker)

        self.skipped_pub.publish(markers)

    def finish(self, reason):
        if self.state == "done":
            return
        self.state = "done"
        self.publish_skipped()

        skipped = self.skipped_metres()
        total = self.cum[-1] if self.cum else 0.0
        summary = (f"{self.detours} detour(s) around obstacles the costmap saw coming, "
                   f"{self.recoveries_total} recovery/recoveries after the controller gave up, "
                   f"{skipped:.1f}m of {total:.0f}m skipped in {len(self.skipped)} stretch(es)")
        if reason == "coverage path complete":
            self.get_logger().info(f"coverage path complete: {summary}")
        else:
            self.get_logger().error(f"coverage run ended early - {reason}: {summary}")

    def wait_for_servers(self):
        """Blocking, and so deliberately called before the executor spins:
        waiting on an action server from inside a callback blocks the single
        executor thread that would have to service the discovery it waits on."""
        for name, client in (("planner_server", self.compute_client),
                             ("controller_server", self.follow_client)):
            self.get_logger().info(f"waiting for {name}")
            client.wait_for_server()
        self.backup_client.wait_for_server(timeout_sec=5.0)


def main():
    rclpy.init()
    node = CoverageExecutor()
    try:
        node.wait_for_servers()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
