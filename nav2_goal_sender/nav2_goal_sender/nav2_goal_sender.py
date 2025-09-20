#!/usr/bin/env python3
import math
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav_msgs.srv import GetPlan
from nav2_msgs.action import ComputePathToPose

# ------------------------------
# Per-robot planner client
# ------------------------------
class RobotPlannerClient:
    def __init__(self, node: Node, ns: str, planner_id: str, path_qos: QoSProfile):
        """
        ns: robot namespace, e.g. 'robot1' (no leading slash).
        Will use actions at f'/{ns}/compute_path_to_pose'
        """
        self.node = node
        self.ns = ns.strip('/')

        # Action client for Nav2 planning
        self.plan_ac = ActionClient(
            node,
            ComputePathToPose,
            f'/{self.ns}/compute_path_to_pose'
        )

        # Publishers for visualization
        self.pub_path_current = node.create_publisher(
            Path, f'/{self.ns}/planned_path', path_qos
        )
        self.pub_path_between = node.create_publisher(
            Path, f'/{self.ns}/planned_path_between', path_qos
        )

        # Subscription: “plan from current robot pose to goal”
        self.sub_goal = node.create_subscription(
            PoseStamped,
            f'/{self.ns}/target_pose',
            self._on_goal_from_current,
            10
        )

        # Service: “plan between arbitrary start and goal” (also publishes for RViz)
        self.srv_between = node.create_service(
            GetPlan,
            f'/{self.ns}/plan_between',
            self._on_plan_between
        )

        self.planner_id = planner_id or ""

        node.get_logger().info(
            f'[{self.ns}] planner ready | goal topic: /{self.ns}/target_pose | '
            f'paths: /{self.ns}/planned_path, /{self.ns}/planned_path_between | '
            f'action: /{self.ns}/compute_path_to_pose'
        )

    # ---- public helpers ----
    def plan_from_current(self, goal: PoseStamped, publish_to_between: bool = False):
        """Plan from the robot's current pose to goal (start unset)."""
        plan_goal = ComputePathToPose.Goal()
        plan_goal.goal = goal
        if self.planner_id:
            plan_goal.planner_id = self.planner_id

        # Send async goal
        self.plan_ac.wait_for_server(timeout_sec=1.0)
        fut = self.plan_ac.send_goal_async(plan_goal)
        fut.add_done_callback(lambda ghf: self._handle_goal_handle(ghf, goal, publish_to_between))

    def plan_between(self, start: PoseStamped, goal: PoseStamped):
        """Plan between two arbitrary poses (start provided)."""
        plan_goal = ComputePathToPose.Goal()
        plan_goal.start = start
        plan_goal.goal = goal
        if self.planner_id:
            plan_goal.planner_id = self.planner_id

        self.plan_ac.wait_for_server(timeout_sec=1.0)
        fut = self.plan_ac.send_goal_async(plan_goal)
        # we reuse the same handler; publish to the 'between' topic
        fut.add_done_callback(lambda ghf: self._handle_goal_handle(ghf, goal, publish_to_between=True))

    # ---- callbacks ----
    def _on_goal_from_current(self, goal_msg: PoseStamped):
        # Plan from current robot pose to goal
        self.node.get_logger().info(
            f'[{self.ns}] planning from current → '
            f'({goal_msg.pose.position.x:.2f}, {goal_msg.pose.position.y:.2f}) '
            f'[{goal_msg.header.frame_id or "map"}]'
        )
        self.plan_from_current(goal_msg, publish_to_between=False)

    def _on_plan_between(self, req: GetPlan.Request, resp: GetPlan.Response):
        """Service handler for arbitrary start->goal planning.
        Returns the path (and also publishes to /{ns}/planned_path_between)."""
        self.node.get_logger().info(
            f'[{self.ns}] plan_between: '
            f'start=({req.start.pose.position.x:.2f}, {req.start.pose.position.y:.2f}) '
            f'goal=({req.goal.pose.position.x:.2f}, {req.goal.pose.position.y:.2f})'
        )

        # Kick off the nav2 action and block on a local future to return the response.
        # We'll bridge by storing a local future and waiting on result via rclpy executor spinning.
        done_container = {'path': None}

        def _capture(path: Optional[Path]):
            done_container['path'] = path

        # Fire the action; when result arrives, we stash it
        self.plan_between(req.start, req.goal)

        # We can't directly await here; instead, spin until we publish & capture
        # To avoid long blocks, we spin with a timeout loop.
        # The publish happens in _handle_result; we also stash into done_container via a short-lived subscription to our own publish call.
        # Simpler approach: we replicate logic and set resp.plan when result arrives via a local patch in _handle_result
        # Implement by monkey-patching a one-shot "result sink" on this instance:
        self._oneshot_result_sink = _capture  # type: ignore

        # Spin until _handle_result triggers (or timeout)
        wait_s = 2.0  # short, since this is primarily for visualization; adjust as needed
        end_time = self.node.get_clock().now().nanoseconds + int(wait_s * 1e9)
        while rclpy.ok() and done_container['path'] is None:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if self.node.get_clock().now().nanoseconds > end_time:
                break

        if done_container['path'] is None:
            self.node.get_logger().warn(f'[{self.ns}] plan_between timed out; returning empty plan')
            resp.plan = Path()
        else:
            resp.plan = done_container['path']

        return resp

    # ---- internal helpers ----
    def _handle_goal_handle(self, gh_future, goal_msg: PoseStamped, publish_to_between: bool):
        gh = gh_future.result()
        if not gh or not gh.accepted:
            self.node.get_logger().warn(f'[{self.ns}] plan request rejected')
            return
        gh.get_result_async().add_done_callback(
            lambda resf: self._handle_result(resf, publish_to_between)
        )

    def _handle_result(self, res_future, publish_to_between: bool):
        try:
            result = res_future.result().result  # type: ignore[attr-defined]
        except Exception as e:
            self.node.get_logger().error(f'[{self.ns}] plan result error: {e}')
            return

        path: Path = result.path
        if not path.poses:
            self.node.get_logger().warn(f'[{self.ns}] planner returned empty path')
            # still publish an empty message to update RViz
            (self.pub_path_between if publish_to_between else self.pub_path_current).publish(Path())
            # propagate to oneshot sink if waiting
            if hasattr(self, '_oneshot_result_sink'):
                self._oneshot_result_sink(Path())  # type: ignore
                delattr(self, '_oneshot_result_sink')
            return

        # Compute and log approximate length
        L = 0.0
        for i in range(1, len(path.poses)):
            p0 = path.poses[i-1].pose.position
            p1 = path.poses[i].pose.position
            L += math.hypot(p1.x - p0.x, p1.y - p0.y)

        self.node.get_logger().info(
            f'[{self.ns}] path: {len(path.poses)} poses, ~{L:.2f} m'
        )

        # Publish to the appropriate topic for RViz
        pub = self.pub_path_between if publish_to_between else self.pub_path_current
        pub.publish(path)

        # If a service call is waiting for the plan, feed it
        if hasattr(self, '_oneshot_result_sink'):
            self._oneshot_result_sink(path)  # type: ignore
            delattr(self, '_oneshot_result_sink')


# ------------------------------
# Multi-robot planner manager
# ------------------------------
class MultiNavPlanner(Node):
    def __init__(self):
        super().__init__('nav2_goal_sender')

        # Parameters
        self.declare_parameter('robot_namespaces', ['husky'])
        self.declare_parameter('goal_frame', 'map')
        self.declare_parameter('planner_id', '')

        robot_namespaces: List[str] = list(
            self.get_parameter('robot_namespaces').get_parameter_value().string_array_value
        )
        self.goal_frame: str = self.get_parameter('goal_frame').get_parameter_value().string_value
        self.planner_id: str = self.get_parameter('planner_id').get_parameter_value().string_value

        # Latched-style QoS so the last path stays visible in RViz
        path_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )

        # Build per-robot clients
        self.robot_clients: Dict[str, RobotPlannerClient] = {}
        for ns in robot_namespaces:
            ns_clean = ns.strip('/')
            self.robot_clients[ns_clean] = RobotPlannerClient(self, ns_clean, self.planner_id, path_qos)

        self.get_logger().info(f'MultiNavPlanner up for robots: {", ".join(self.robot_clients.keys())}')

def main():
    rclpy.init()
    node = MultiNavPlanner()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
