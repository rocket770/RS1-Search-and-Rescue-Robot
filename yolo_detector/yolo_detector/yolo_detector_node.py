#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.task import Future
from rclpy.duration import Duration
from rclpy.time import Time

import numpy as np
import cv2
from cv_bridge import CvBridge

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point, PoseStamped
from std_msgs.msg import Header

import tf2_geometry_msgs

from ultralytics import YOLO
import torch

import message_filters
import tf2_ros

from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose, BoundingBox2D

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from builtin_interfaces.msg import Time as BuiltinTime
from math import sqrt

class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_detector')

        # Parameters
        self.declare_parameter('model_path', 'runs_poc/03_stageB_full/weights/best_stable_best_result.pt')
        self.declare_parameter('conf_thres', 0.25)
        self.declare_parameter('iou_thres', 0.5)
        self.declare_parameter('camera_frame', 'camera_optical_frame')
        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('depth_scale', 1.0)   # if depth is in m
        self.declare_parameter('merge_distance', 1.5) # meters for dedup in global list

        self.model_path   = self.get_parameter('model_path').value
        self.conf_thres   = self.get_parameter('conf_thres').value
        self.iou_thres    = self.get_parameter('iou_thres').value
        self.cam_frame    = self.get_parameter('camera_frame').value
        self.target_frame = self.get_parameter('target_frame').value
        self.depth_scale  = self.get_parameter('depth_scale').value
        self.merge_dist   = float(self.get_parameter('merge_distance').value)

        # Auto GPU then CPU fallback
        self.device = 0 if torch.cuda.is_available() else "cpu"
        self.get_logger().info(f"Using device: {'CUDA:0' if self.device == 0 else 'CPU'}")

        # Load YOLO model
        self.model = YOLO(self.model_path)
        self.model.to(self.device)

        self.bridge = CvBridge()
        self.K = None  # cam intrinsics (fx, fy, cx, cy)

        # TF buffer/listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        reliable_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST
        )
        startup_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST
        )

        # Wait for initial CameraInfo
        self._startup_caminfo = None
        ready_future = Future()

        def _caminfo_once(msg: CameraInfo):
            self._startup_caminfo = msg
            if not ready_future.done():
                ready_future.set_result(True)

        temp_sub = self.create_subscription(CameraInfo, '/camera/camera_info', _caminfo_once, startup_qos)
        rclpy.spin_until_future_complete(self, ready_future, timeout_sec=None)
        self.destroy_subscription(temp_sub)

        if not ready_future.done():
            self.get_logger().error("Timed out waiting for initial CameraInfo; shutting down.")
            raise RuntimeError("No CameraInfo received at startup")
        else:
            msg = self._startup_caminfo
            self.K = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])  # fx, fy, cx, cy
            self.cam_frame = msg.header.frame_id
            self.get_logger().info(
                f"Initial CameraInfo: fx={self.K[0]:.1f} fy={self.K[1]:.1f} "
                f"cx={self.K[2]:.1f} cy={self.K[3]:.1f} frame={self.cam_frame}"
            )

        self._latest_caminfo = None
        self._last_caminfo_warn = None

        caminfo_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST
        )
        self.create_subscription(CameraInfo, '/camera/camera_info', self._caminfo_cb, caminfo_qos)

        self.create_timer(1.0, self._process_caminfo_1hz)

        sub_rgb   = message_filters.Subscriber(self, Image, '/camera/image',       qos_profile=reliable_qos)
        sub_depth = message_filters.Subscriber(self, Image, '/camera/depth/image', qos_profile=reliable_qos)

        ats = message_filters.ApproximateTimeSynchronizer([sub_rgb, sub_depth], queue_size=10, slop=0.1)
        ats.registerCallback(self.cb)

        # Publishers
        self.pub_det   = self.create_publisher(Detection2DArray, '/yolo_detector/detections', 10)
        self.pub_image = self.create_publisher(Image, '/yolo_detector/detections/image', reliable_qos)
        self.pub_global= self.create_publisher(Detection2DArray, '/yolo_detector/global_detections', 10)

        self.get_logger().info(f"YOLO detector started! With model {self.model_path}")


        # 'class': str, 'point': Point, 'score': float, 'count': int
        self.global_dets = []

    # CameraInfo sampling 1 Hz
    def _caminfo_cb(self, msg: CameraInfo):
        self._latest_caminfo = msg

    def _process_caminfo_1hz(self):
        if self._latest_caminfo is None:
            return
        msg = self._latest_caminfo
        self.K = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])
        self.cam_frame = msg.header.frame_id

    def cb(self, rgb_msg, depth_msg):
        # only run if camera info is there
        if self.K is None:
            now = self.get_clock().now()
            if (self._last_caminfo_warn is None) or ((now - self._last_caminfo_warn) > Duration(seconds=5.0)):
                self.get_logger().warning("Waiting for CameraInfo (intrinsics not ready)")
                self._last_caminfo_warn = now
            return

        fx, fy, cx, cy = self.K # camera intrinsics or whatever its called lol

        # Convert ROS to OpenCV
        color = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
        depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        if depth.dtype != np.float32:
            depth = depth.astype(np.float32) * self.depth_scale

        results = self.model.predict(
            source=color, conf=self.conf_thres, iou=self.iou_thres,
            verbose=False, device=self.device
        )

        det_array = Detection2DArray()
        det_array.header = rgb_msg.header  
        annotated = color.copy()

        any_global_added = False  # track if we added to the persistent array this frame

        for r in results:
            if not getattr(r, "boxes", None) or len(r.boxes) == 0:
                continue
            names = getattr(r, "names", None) or getattr(self.model, "names", {}) or {}
            for box in r.boxes:
                cls_id = int(box.cls[0])
                label_name = names.get(cls_id, str(cls_id))
                conf = float(box.conf[0])
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().tolist()
                u, v = (x1 + x2) / 2.0, (y1 + y2) / 2.0

                # get depth or skip
                d = self.get_depth_median(depth, int(u), int(v), k=5)
                if d is None or not np.isfinite(d) or d <= 0.0:
                    continue

                #pose in camera frame
                Xc, Yc, Zc = self.pixel_to_3d(u, v, d, self.K)
                ps = PoseStamped()
                ps.header = rgb_msg.header
                ps.header.frame_id = self.cam_frame
                ps.pose.position.x = float(Xc)
                ps.pose.position.y = float(Yc)
                ps.pose.position.z = float(Zc)
                ps.pose.orientation.w = 1.0

                # ransform to target frame; if it fails, skip detection entirely
                out = self._safe_transform_pose(ps)
                if out is None:
                    continue

                # Dedup into the persistent global list (by class and dist)
                added = self._add_to_global_if_new(label_name, out.pose.position, conf)
                any_global_added = any_global_added or added

                #only now make per-frame detection (for the image overlay topic)
                det = Detection2D()
                det.header = rgb_msg.header

                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = label_name
                hyp.hypothesis.score = conf
                hyp.pose.pose.position = out.pose.position
                det.results.append(hyp)

                bb = BoundingBox2D()
                bb.center.position.x = float(u); bb.center.position.y = float(v)
                bb.size_x = float(x2 - x1); bb.size_y = float(y2 - y1)
                det.bbox = bb

                det_array.detections.append(det)

                # draw only for published per-frame detections
                x1_i, y1_i, x2_i, y2_i = map(int, (x1, y1, x2, y2))
                cv2.rectangle(annotated, (x1_i, y1_i), (x2_i, y2_i), (0, 255, 0), 2)
                label = f"{label_name} {conf:.2f}"
                cv2.putText(annotated, label, (x1_i, max(0, y1_i - 5)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)

        # Publish per-frame detections
        if len(det_array.detections) > 0:
            self.pub_det.publish(det_array)

        # publish the global (persistent) array ONLY if something new was added 
        if any_global_added:
            global_msg = self._build_global_detection_array(det_array.header.stamp)
            self.pub_global.publish(global_msg)

        # Annotated image goes out regardless; comment these two lines if you want to only publish when detections occur
        img_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
        img_msg.header = rgb_msg.header
        self.pub_image.publish(img_msg)


    # honestyly not certain with how this works, but its trys to match the time up of the messages so it can transform more accruately
    def _safe_transform_pose(self, ps: PoseStamped) -> PoseStamped | None:
        try:
            return self.tf_buffer.transform(ps, self.target_frame, timeout=Duration(seconds=0.2))
        except tf2_ros.ExtrapolationException as e:
            self.get_logger().debug(f"Future extrapolation; retrying with latest TF: {e}")
            try:
                ps_latest = PoseStamped()
                ps_latest.header = ps.header
                ps_latest.header.stamp = BuiltinTime(sec=0, nanosec=0)  # special value meaning "latest"
                ps_latest.pose = ps.pose
                return self.tf_buffer.transform(ps_latest, self.target_frame, timeout=Duration(seconds=0.2))
            except Exception:
                try:
                    original = Time.from_msg(ps.header.stamp)
                    backdated = (original - Duration(seconds=0.1)).to_msg()
                    ps_back = PoseStamped()
                    ps_back.header = ps.header
                    ps_back.header.stamp = backdated
                    ps_back.pose = ps.pose
                    return self.tf_buffer.transform(ps_back, self.target_frame, timeout=Duration(seconds=0.2))
                except Exception as e3:
                    self.get_logger().warning(f"TF transform still failing after fallbacks: {e3}")
                    return None
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.TimeoutException) as e:
            self.get_logger().warning(f"TF transform failed: {e}")
            return None

    def _add_to_global_if_new(self, class_id: str, p: Point, score: float) -> bool:
        for g in self.global_dets:
            if g['class'] != class_id:
                continue
            if self._dist_xy(g['point'], p) <= self.merge_dist:
                # already have a nearby one; update stats (optional)
                g['score'] = max(g['score'], score)
                g['count'] += 1
                return False
        # new unique detection
        self.global_dets.append({'class': class_id, 'point': Point(x=p.x, y=p.y, z=p.z), 'score': score, 'count': 1})
        return True

    @staticmethod
    def _dist_xy(a: Point, b: Point) -> float:
        return ((a.x - b.x)**2 + (a.y - b.y)**2) ** 0.5

    def _build_global_detection_array(self, stamp: BuiltinTime) -> Detection2DArray:
        arr = Detection2DArray()
        arr.header.stamp = stamp
        arr.header.frame_id = self.target_frame
        for g in self.global_dets:
            det = Detection2D()
            det.header.stamp = stamp
            det.header.frame_id = self.target_frame
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = g['class']
            hyp.hypothesis.score = g['score']
            hyp.pose.pose.position = g['point']
            det.results.append(hyp)
            arr.detections.append(det)
        return arr

    @staticmethod
    def get_depth_median(depth_img, u, v, k=5):
        h, w = depth_img.shape[:2]
        half = k // 2
        x1 = max(0, u-half); x2 = min(w, u+half+1)
        y1 = max(0, v-half); y2 = min(h, v+half+1)
        patch = depth_img[int(y1):int(y2), int(x1):int(x2)]
        vals = patch[np.isfinite(patch) & (patch > 0)]
        if vals.size == 0:
            return None
        return float(np.median(vals))

    @staticmethod
    def pixel_to_3d(u, v, depth, K):
        fx, fy, cx, cy = K
        X = (u - cx) * depth / fx
        Y = (v - cy) * depth / fy
        Z = depth
        return X, Y, Z


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
