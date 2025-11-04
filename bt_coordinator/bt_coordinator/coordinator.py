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

from nav2_msgs.msg import SpeedLimit

from geometry_msgs.msg import Twist

from std_msgs.msg import Bool 

from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from collections import deque

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

class BTCoordinator(Node):

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
        self.approach_distance = float(self.declare_parameter("approach_distance", 2.0).value)
        self.battery_threshold = float(self.declare_parameter("battery_threshold", 0.30).value)
        self.home_pose_xy = tuple(self.declare_parameter("home_pose_xy", [0.0, 0.0]).value)  # [x, y]
        self.post_manual_resume_suppress_secs = int(self.declare_parameter("post_manual_resume_suppress_secs", 1).value)
        self.cmd_vel_topic = self.declare_parameter("cmd_vel_topic", "/cmd_vel").get_parameter_value().string_value
        self.ui_move_topic = self.declare_parameter("ui_move_topic", "/ui/move").get_parameter_value().string_value

        self.detection_memory = int(self.declare_parameter("detection_memory", 4).value)

        self.state = self.ST_EXPLORE
        self.visited_classes = deque(maxlen=self.detection_memory)   
        self.suppress_detections_until = 0.0
        self.active_goal_id = None

        self.srv_reset_batt = self.create_client(Empty, "/reset_battery")

        self.srv_user_resume = self.create_service(Trigger, "/user/resume_explore", self._on_user_resume)

        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        self.sub_goal = self.create_subscription(PoseStamped, f"/{self.bt_goal_topic}", self._on_user_goal, 10)
        self.sub_detect2d = self.create_subscription(Detection2DArray, self.detection_topic, self._on_det, 10)

        self._wait_for_service(self.srv_reset_batt, "/reset_battery")

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.pub_explore_resume = self.create_publisher(Bool, "/explore/resume", qos)

        battery_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.sub_ui_move = self.create_subscription(Twist, self.ui_move_topic, self._on_ui_move, battery_qos)


        self.sub_battery = self.create_subscription(
            BatteryState,
            "/battery_state",
            self._on_battery,
            battery_qos,
        )

        # remmebr to move up top later for consistnacy lol
        self.global_goal_topic = self.declare_parameter(
            "global_goal_topic", "/static_path/goal"
        ).get_parameter_value().string_value
        self.pub_global_goal = self.create_publisher(PoseStamped, self.global_goal_topic, 10)

        self.pub_cmd_vel = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        self.global_paths = set()

        self.slowdown_factor = float(self.declare_parameter("slowdown_factor", 0.5).value) 

        self.speed_limit_topic = self.declare_parameter(
            "speed_limit_topic", "/speed_limit"
        ).get_parameter_value().string_value
        self.pub_speed_limit = self.create_publisher(SpeedLimit, self.speed_limit_topic, 10)

        self._speed_reduced = False

        self.get_logger().info(
            f"bt_coordinator up. Using '{self.detection_topic}'. "
        )

        self._manual_paused_once = False
        self._explore_was_paused = False

    def _wait_for_service(self, client, name):
        if not client.service_is_ready():
            self.get_logger().warn(f"Waiting for service {name} ...")
            client.wait_for_service(timeout_sec=5.0)
        if client.service_is_ready():
            self.get_logger().info(f"Connected to {name}")
        else:
            self.get_logger().warn(f"{name} not available yet — continuing (calls will fail until it appears).")
    
    # lil helper that wraps calls, returns true if call completed and server responsed with success
    def call_trigger(self, client) -> bool:
        if not client.service_is_ready():
            self.get_logger().warn("Service not ready")
            return False
        future = client.call_async(Trigger.Request())

        # block until done or 3s have passed
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        if future.result() is None:
            self.get_logger().warn("Trigger call timed out")
            return False
        ok = bool(future.result().success)
        if not ok:
            self.get_logger().warn(f"Trigger call failed: {future.result().message}")
        return ok

    # helper so we can check if response is empty or not (call reutrns empty yes/no)
    def call_empty(self, client) -> bool:
        # no point checking if we can't find the service
        if not client.service_is_ready():
            self.get_logger().warn("Service not ready")
            return False
        
        future = client.call_async(Empty.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        return future.result() is not None
    
    def _on_ui_move(self, msg: Twist):
        # First manual twist we cancel Nav2, pause SLAM once, move to WAIT_RESUME
        if not self._manual_paused_once:
            # If speed was reduced for an approach, restore full speed 
            if self._speed_reduced:
                self._set_speed_percentage(1.0)
                self._speed_reduced = False

            self.get_logger().info("UI move pausing...")
            self._preempt_everything()
            self._pause_explore()
            self.state = self.ST_WAIT_RESUME
            self._manual_paused_once = True

        self.pub_cmd_vel.publish(msg)

    def _is_valid_vel(self, t: Twist) -> bool:        
        return (abs(t.linear.x) > 1e-3 or abs(t.linear.y) > 1e-3 or abs(t.linear.z) > 1e-3 or abs(t.angular.x) > 1e-3 or abs(t.angular.y) > 1e-3 or abs(t.angular.z) > 1e-3)

    def _set_speed_percentage(self, pct: float):
        self.get_logger().info(f"Setting speed: '{pct}'. ")
        msg = SpeedLimit()
        msg.percentage = True
        msg.speed_limit = float(max(0.0, min(1.0, pct)) * 100.0)
        self.pub_speed_limit.publish(msg)
        
    def _on_user_resume(self, req, resp):
        self._resume_explore()
        self.state = self.ST_EXPLORE

        if self._manual_paused_once:
            now = time.time()
            self.suppress_detections_until = now + float(self.post_manual_resume_suppress_secs)
        else:
            # resume after an auto-detection: allow new detections immediately
            self.suppress_detections_until = 0.0

        self._manual_paused_once = False

        resp.success = True
        resp.message = "Exploration resumed."
        if self._speed_reduced:
            self._set_speed_percentage(1.0)
            self._speed_reduced = False
            
        return resp

    def _on_battery(self, msg: BatteryState):
        if msg.percentage != msg.percentage:  #nan thigny
            return
        if msg.percentage < self.battery_threshold:
            if self.state != self.ST_TO_DOCK:
                self.get_logger().warn(f"Battery low ({msg.percentage:.2f}); returning home.")
                if self._speed_reduced == True:
                    self._set_speed_percentage(1.0)
                    self._speed_reduced = False
                self._preempt_everything()
                self._pause_explore()
                self._go_to_home()

    def _on_user_goal(self, pose: PoseStamped):
        if self._speed_reduced:
            self._set_speed_percentage(1.0)
            self._speed_reduced = False
        self.get_logger().info("Received manual goal; pausing SLAM and navigating.")
        self._preempt_everything()
        self._pause_explore()
        self._send_nav_goal(pose, target_state=self.ST_TO_MANUAL)

    def _publish_static_goal(self, x: float, y: float, frame: str):
        msg = PoseStamped()
        msg.header.frame_id = frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = 0.0

        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = 0.0
        q.w = 1.0
        msg.pose.orientation = q

        self.pub_global_goal.publish(msg)
        self.get_logger().info(
            f"Published static path goal at ({x:.2f}, {y:.2f}) to {self.global_goal_topic}"
        )

    def _on_det(self, arr: Detection2DArray):
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

            pt = (round(x),round(y))
            if pt not in self.global_paths:
                self._publish_static_goal(x, y, frame)
                self.global_paths.add(pt)

            ax = x - self.approach_distance * math.cos(yaw_det)
            ay = y - self.approach_distance * math.sin(yaw_det)
            ayaw = math.atan2(y - ay, x - ax)

            self.get_logger().info(
                f"Detection '{cls}' at ({x:.2f},{y:.2f}); approaching to ({ax:.2f},{ay:.2f})."
            )

            self._preempt_everything()
            self._pause_explore()

            if self._speed_reduced == False:
                self._set_speed_percentage(self.slowdown_factor)
                self._speed_reduced = True
                
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
            self.get_logger().info(
                f" Ignored Detection due to state!!!!!!!!!!!!! State: {self.state}"
            )
            return True
        t = time.time()
        if t < self.suppress_detections_until:
            self.get_logger().info(
                f" Ignored Detection due to time!!!!!!!!!!!!! Time: {self.suppress_detections_until-t}"
            )
            return True
        return False


    def _preempt_everything(self):
        if self.nav_client.server_is_ready():
            try:
                self.nav_client.cancel_all_goals()
            except Exception:
                pass

    def _pause_explore(self):
        msg = Bool()
        msg.data = False
        self.pub_explore_resume.publish(msg)
        self._explore_was_paused = True        
        self.get_logger().info("Sent False to /explore/resume")
        time.sleep(1)


    def _resume_explore(self) -> bool:
        msg = Bool()
        msg.data = True
        self.pub_explore_resume.publish(msg)
        self.get_logger().info("Sent True to /explore/resume")
        self._explore_was_paused = False        

        return True

    def _send_nav_goal(self, pose: PoseStamped, target_state: str, detection_class: Optional[str] = None):
        if not self.nav_client.server_is_ready():
            self.get_logger().warn("navigate_to_pose server not ready.")
            return
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        self.state = target_state
        self.active_goal_id = self.nav_client.send_goal_async(goal_msg)
        self.active_goal_id.add_done_callback(lambda fut: self._on_nav_accepted(fut, detection_class))

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

        prev_state = self.state  # remember what this goal was for

        if prev_state == self.ST_TO_DETECTION and detection_class:
            self.visited_classes.append(detection_class)

        if prev_state == self.ST_TO_DOCK:
            self.get_logger().info("At home; calling /reset_battery ...")
            self.call_empty(self.srv_reset_batt)

        # only go to WAIT_RESUME if we had actually paused explore
        if self._explore_was_paused:
            self.state = self.ST_WAIT_RESUME
            self.get_logger().info("Waiting for /user/resume_explore")
        else:
            self.state = self.ST_EXPLORE
            self.get_logger().info("Nav done; staying in explore.")


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
