#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration as RosDuration

from nav2_msgs.action import ComputePathToPose

class GlobalPathPlanner(Node):

    def __init__(self):
        super().__init__('global_path_planner')

        self.declare_parameter('home_pose_xy', [0.0, 0.0])
        self.declare_parameter('frame_id', 'map')

        self.declare_parameter('planner_action', '/planner_server/compute_path_to_pose')
        self.declare_parameter('goal_topic', '/static_path/goal')
        self.declare_parameter('marker_topic', '/static_paths/markers')
        self.declare_parameter('marker_width', 0.05)
        self.declare_parameter('marker_alpha', 1.0)

        self.frame_id = self.get_parameter('frame_id').value
        home_xy = self.get_parameter('home_pose_xy').value
        self.home_x, self.home_y = float(home_xy[0]), float(home_xy[1])
        self.planner_action = self.get_parameter('planner_action').value
        self.goal_topic = self.get_parameter('goal_topic').value
        self.marker_topic = self.get_parameter('marker_topic').value
        self.marker_width = float(self.get_parameter('marker_width').value)
        self.marker_alpha = float(self.get_parameter('marker_alpha').value)

        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.path_pub = self.create_publisher(Path, '/static_paths/all_paths', latched)
        self.marker_pub = self.create_publisher(MarkerArray, self.marker_topic, latched)

        self.marker_array = MarkerArray()
        self.marker_id = 0

        self.action_client = ActionClient(self, ComputePathToPose, self.planner_action)

        self.create_subscription(PoseStamped, self.goal_topic, self._on_goal, 10)

        self.get_logger().info(
            f"Static planner ready. Home=({self.home_x:.2f}, {self.home_y:.2f}) | "
            f"Listening on {self.goal_topic} | Action: {self.planner_action}"
        )

    def _on_goal(self, goal: PoseStamped) -> None:
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Planner action server no worky.")
            return

        start = PoseStamped()
        start.header.frame_id = self.frame_id
        start.pose.position.x = self.home_x
        start.pose.position.y = self.home_y
        start.pose.orientation.w = 1.0

        if not goal.header.frame_id:
            goal.header.frame_id = self.frame_id

        goal_msg = ComputePathToPose.Goal()
        goal_msg.start = start
        goal_msg.goal = goal
        goal_msg.use_start = True
        goal_msg.planner_id = '' # should just be default planner me thinks

        gx, gy = goal.pose.position.x, goal.pose.position.y
        self.get_logger().info(f"Planning home ({gx:.2f}, {gy:.2f})")

        send_future = self.action_client.send_goal_async(goal_msg)
        send_future.add_done_callback(lambda f: self._on_goal_response(goal, f))


    def _on_goal_response(self, goal: PoseStamped, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as e:
            self.get_logger().error(f"Failed to send ComputePathToPose goal :( {e}")
            return

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("ComputePathToPose goal was rejected")
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda f: self._on_path_result(goal, f))

    def _on_path_result(self, goal: PoseStamped, future) -> None:
        if future.cancelled():
            self.get_logger().error("Planner result future was cancelled.")
            return

        exception = future.exception()
        if exception is not None:
            self.get_logger().error(f"Planner failed: {exception}")
            return

        result = future.result().result
        path: Path = result.path

        now = self.get_clock().now().to_msg()

        if not path.header.frame_id:
            path.header.frame_id = self.frame_id

        path.header.stamp = now
        
        self.path_pub.publish(path)

        self.marker_id += 1
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.ns = 'static_paths'
        marker.id = self.marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = self.marker_width
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = self.marker_alpha
        marker.lifetime = RosDuration(sec=0)  # persistent
        marker.pose.orientation.w = 1.0

        for p in path.poses:
            q = Point()
            q.x = p.pose.position.x
            q.y = p.pose.position.y
            q.z = 0.0
            marker.points.append(q)

        self.marker_array.markers.append(marker)
        self.marker_pub.publish(self.marker_array)

        gx, gy = goal.pose.position.x, goal.pose.position.y
        self.get_logger().info(
            f"Published path {self.marker_id}: home ({gx:.2f}, {gy:.2f}) "
            f"({len(path.poses)} poses)"
        )

def main():
    rclpy.init()
    try:
        rclpy.spin(GlobalPathPlanner())
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
