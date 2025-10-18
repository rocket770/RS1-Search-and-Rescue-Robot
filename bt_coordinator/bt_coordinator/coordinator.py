# bt_coordinator/coordinator.py
import math
import time
from typing import Optional, Tuple, Set

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Quaternion
from sensor_msgs.msg import BatteryState
from std_srvs.srv import Trigger, Empty
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

from vision_msgs.msg import Detection2DArray  # 2D-only

# ---------- small helpers ----------

def yaw_from_quat(q: Quaternion) -> float:
    ysqr = q.y * q.y
    t3 = +2.0 * (q.w * q.z + q.x * q.y)
    t4 = +1.0 - 2.0 * (ysqr + q.z * q.z)
    return math.atan2(t3, t4)

def make_quat_from_yaw(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    q.x = 0.0
    q.y = 0.0
    return q

# ---------- coordinator ----------

class BTCoordinator(Node):
    """
    High-level coordinator for:
      - SLAM pause/resume around detection/goal-driven stops
      - De-dup by class_id (only visit each class once)
      - Battery failsafe to (0,0) + /reset_battery
      - “Resume” gating via /user/resume_explore (Trigger)
      - YOLO: Detection2DArray-only
    """

    ST_EXPLORE = "explore"
    ST_TO_DETECTION = "to_detection"
    ST_TO_MANUAL = "to_manual_goal"
    ST_TO_DOCK = "to_dock"
    ST_WAIT_RESUME = "wait_resume"

    def __init__(self):
        super().__init__("bt_coordinator")

        self.map_frame = self.declare_parameter("map_frame", "map").get_parameter_value().string_value
        self.bt_goal_topic = self.declare_parameter("bt_goal_topic", "user/goal_pose_input").get_parameter_value().string_value
        self.detection_topic = self.declare_parameter("detection_topic", "/yolo_detector/detections").get_parameter_value().string_value
        self.approach_distance = float(self.declare_parameter("approach_distance", 1.0).value)
        self.battery_threshold = float(self.declare_parameter("battery_threshold", 0.30).value)
        self.home_pose_xy = tuple(self.declare_parameter("home_pose_xy", [0.0, 0.0]).value)  # [x, y]
        self.post_manual_resume_suppress_secs = int(self.declare_parameter("post_manual_resume_suppress_secs", 8).value)

        self.state = self.ST_EXPLORE
        self.visited_classes: Set[str] = set()
        self.suppress_detections_until = 0.0
        self.active_goal_id = None

        self.srv_pause = self.create_client(Trigger, "/slam/pause")
        self.srv_resume = self.create_client(Trigger, "/slam/resume")
        self.srv_reset_batt = self.create_client(Empty, "/reset_battery")

        self.srv_user_resume = self.create_service(Trigger, "/user/resume_explore", self._on_user_resume)

        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        self.sub_battery = self.create_subscription(BatteryState, "/battery_state", self._on_battery, 10)
        self.sub_goal = self.create_subscription(PoseStamped, f"/{self.bt_goal_topic}", self._on_user_goal, 10)
        self.sub_detect2d = self.create_subscription(Detection2DArray, self.detection_topic, self._on_det2d, 10)

        self.get_logger().info(
            f"bt_coordinator up. Using Detection2DArray on '{self.detection_topic}'. "
            "States: explore → (to_detection | to_manual | to_dock) → wait_resume."
        )

        self._wait_for_service(self.srv_pause, "/slam/pause")
        self._wait_for_service(self.srv_resume, "/slam/resume")
        self._wait_for_service(self.srv_reset_batt, "/reset_battery")

    def _wait_for_service(self, client, name):
        if not client.service_is_ready():
            self.get_logger().warn(f"Waiting for service {name} ...")
            client.wait_for_service(timeout_sec=5.0)
        if client.service_is_ready():
            self.get_logger().info(f"Connected to {name}")
        else:
            self.get_logger().warn(f"{name} not available yet — continuing (calls will fail until it appears).")

    def call_trigger(self, client) -> bool:
        if not client.service_is_ready():
            self.get_logger().warn("Service not ready")
            return False
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        if future.result() is None:
            self.get_logger().warn("Trigger call timed out")
            return False
        ok = bool(future.result().success)
        if not ok:
            self.get_logger().warn(f"Trigger call failed: {future.result().message}")
        return ok

    def call_empty(self, client) -> bool:
        if not client.service_is_ready():
            self.get_logger().warn("Service not ready")
            return False
        future = client.call_async(Empty.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        return future.result() is not None

    def _on_user_resume(self, req, resp):
        if self.state != self.ST_WAIT_RESUME:
            resp.success = True
            resp.message = "Already exploring or busy; resuming anyway."
            return resp

        resumed = self.call_trigger(self.srv_resume)
        if resumed:
            self.state = self.ST_EXPLORE
            now = time.time()
            if now > self.suppress_detections_until:
                self.suppress_detections_until = now + float(self.post_manual_resume_suppress_secs)
            resp.success = True
            resp.message = "Exploration resumed."
        else:
            resp.success = False
            resp.message = "Failed to resume SLAM."
        return resp

    def _on_battery(self, msg: BatteryState):
        if msg.percentage != msg.percentage:  # NaN guard
            return
        if msg.percentage < self.battery_threshold:
            if self.state != self.ST_TO_DOCK:
                self.get_logger().warn(f"Battery low ({msg.percentage:.2f}); returning home.")
                self._preempt_everything()
                self._pause_slam()
                self._go_to_home()

    def _on_user_goal(self, pose: PoseStamped):
        self.get_logger().info("Received manual goal; pausing SLAM and navigating.")
        self._preempt_everything()
        self._pause_slam()
        self._send_nav_goal(pose, target_state=self.ST_TO_MANUAL)

    def _on_det2d(self, arr: Detection2DArray):
        # ignore if we're not in exploration or if suppressed
        if self._should_ignore_detections():
            return

        frame = getattr(arr.header, "frame_id", "") or self.map_frame

        # Strategy: pick the highest-score (class, pose) pair from the first detection that has a pose. 
        for det in arr.detections:
            if not det.results:
                continue

            # sort results by score desc
            best = max(det.results, key=lambda r: getattr(getattr(r, "hypothesis", None), "score", 0.0))
            hyp = best.hypothesis
            cls = getattr(hyp, "class_id", None)
            if not cls:
                continue
            if cls in self.visited_classes:
                continue

            try:
                p = best.pose.pose.position
                o = best.pose.pose.orientation
            except Exception:
                continue

            x, y = float(p.x), float(p.y)
            yaw_det = yaw_from_quat(o)

            ax = x - self.approach_distance * math.cos(yaw_det)
            ay = y - self.approach_distance * math.sin(yaw_det)
            ayaw = math.atan2(y - ay, x - ax)

            self.get_logger().info(
                f"Detection '{cls}' at ({x:.2f},{y:.2f}); approaching to ({ax:.2f},{ay:.2f})."
            )
            self._preempt_everything()
            self._pause_slam()

            goal = PoseStamped()
            goal.header.frame_id = frame
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.pose.position.x = ax
            goal.pose.position.y = ay
            goal.pose.orientation = make_quat_from_yaw(ayaw)

            self._send_nav_goal(goal, target_state=self.ST_TO_DETECTION, detection_class=cls)
            break  # handle one at a time

    def _should_ignore_detections(self) -> bool:
        if self.state in (self.ST_TO_MANUAL, self.ST_TO_DOCK, self.ST_WAIT_RESUME):
            return True
        if time.time() < self.suppress_detections_until:
            return True
        return False


    def _preempt_everything(self):
        if self.nav_client.server_is_ready():
            try:
                self.nav_client.cancel_all_goals()
            except Exception:
                pass

    def _pause_slam(self):
        ok = self.call_trigger(self.srv_pause)
        if not ok:
            self.get_logger().warn("Failed to pause SLAM (continuing).")

    def _send_nav_goal(self, pose: PoseStamped, target_state: str, detection_class: Optional[str] = None):
        if not self.nav_client.server_is_ready():
            self.get_logger().warn("navigate_to_pose server not ready.")
            return
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        self.state = target_state
        self.active_goal_id = self.nav_client.send_goal_async(goal_msg, feedback_callback=self._on_nav_feedback)
        self.active_goal_id.add_done_callback(lambda fut: self._on_nav_accepted(fut, detection_class))

    def _on_nav_feedback(self, feedback):
        # prob debug here idk
        pass

    def _on_nav_accepted(self, fut, detection_class: Optional[str]):
        goal_handle = fut.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Nav goal rejected.")
            self.state = self.ST_WAIT_RESUME
            return
        self.get_logger().info("Nav goal accepted.")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda r_fut: self._on_nav_result(r_fut, detection_class))

    def _on_nav_result(self, r_fut, detection_class: Optional[str]):
        status = r_fut.result().status
        self.get_logger().info(f"Nav result status: {status}")
        if self.state == self.ST_TO_DETECTION and detection_class:
            self.visited_classes.add(detection_class)
        if self.state == self.ST_TO_DOCK:
            self.get_logger().info("At home; calling /reset_battery ...")
            self.call_empty(self.srv_reset_batt)
        self.state = self.ST_WAIT_RESUME
        self.get_logger().info("Waiting for /user/resume_explore")

    def _go_to_home(self):
        home = PoseStamped()
        home.header.frame_id = self.map_frame
        home.header.stamp = self.get_clock().now().to_msg()
        home.pose.position.x = float(self.home_pose_xy[0])
        home.pose.position.y = float(self.home_pose_xy[1])
        home.pose.orientation = make_quat_from_yaw(0.0)
        self._send_nav_goal(home, target_state=self.ST_TO_DOCK)

def main():
    rclpy.init()
    node = BTCoordinator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
