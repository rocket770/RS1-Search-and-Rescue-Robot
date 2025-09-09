#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.task import Future
from rclpy.duration import Duration

import numpy as np
import cv2
from cv_bridge import CvBridge

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point, PoseStamped
from std_msgs.msg import Header

from ultralytics import YOLO
import torch

import message_filters
import tf2_ros
import tf2_geometry_msgs

from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose, BoundingBox2D

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import time


class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_detector')

        # Parameters
        self.declare_parameter('model_path', 'runs_poc/03_stageB_full/weights/best.pt')
        self.declare_parameter('conf_thres', 0.25)
        self.declare_parameter('iou_thres', 0.5)
        self.declare_parameter('camera_frame', 'camera_optical_frame')
        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('depth_scale', 1.0)  # 0.001 if depth is mm

        self.model_path  = self.get_parameter('model_path').value
        self.conf_thres  = self.get_parameter('conf_thres').value
        self.iou_thres   = self.get_parameter('iou_thres').value
        self.cam_frame   = self.get_parameter('camera_frame').value
        self.target_frame= self.get_parameter('target_frame').value
        self.depth_scale = self.get_parameter('depth_scale').value

        # Auto GPU then CPU fallback
        self.device = 0 if torch.cuda.is_available() else "cpu"
        self.get_logger().info(f"Using device: {'CUDA:0' if self.device == 0 else 'CPU'}")

        # Load YOLO model
        self.model = YOLO(self.model_path)
        self.model.to(self.device)

        self.bridge = CvBridge()
        self.K = None  # intrinsics (fx, fy, cx, cy)

        # TF buffer/listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        reliable_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST
        )

        # Block startup until first CameraInfo
        startup_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST
        )

        self._startup_caminfo = None
        ready_future = Future()

        def _caminfo_once(msg: CameraInfo):
            # store and fulfill future
            self._startup_caminfo = msg
            if not ready_future.done():
                ready_future.set_result(True)

        temp_sub = self.create_subscription(CameraInfo, '/camera/camera_info', _caminfo_once, startup_qos)

        timeout_sec = 10.0  # set to None to wait indefinitely
        rclpy.spin_until_future_complete(self, ready_future, timeout_sec=timeout_sec)

        self.destroy_subscription(temp_sub)

        if not ready_future.done():
            self.get_logger().error("Timed out waiting for initial CameraInfo, shutting down.")
            raise RuntimeError("No CameraInfo received at startup")
        else:
            # Seed intrinsics from the first message
            msg = self._startup_caminfo
            self.K = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])  # fx, fy, cx, cy
            self.cam_frame = msg.header.frame_id
            self.get_logger().info(
                f"Initial CameraInfo: fx={self.K[0]:.1f} fy={self.K[1]:.1f} "
                f"cx={self.K[2]:.1f} cy={self.K[3]:.1f} frame={self.cam_frame}"
            )

        self._latest_caminfo = None
        self._logged_k = True  # already logged above
        self._last_caminfo_warn = None  # for throttled warning in cb()

        # Keep only latest to avoid backlog
        caminfo_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST
        )
        self.create_subscription(CameraInfo, '/camera/camera_info', self._caminfo_cb, caminfo_qos)

        # Timer to consume CameraInfo once per second and update self.K/self.cam_frame
        self.create_timer(1.0, self._process_caminfo_1hz)

        sub_rgb   = message_filters.Subscriber(self, Image, '/camera/image',       qos_profile=reliable_qos)
        sub_depth = message_filters.Subscriber(self, Image, '/camera/depth/image', qos_profile=reliable_qos)

        ats = message_filters.ApproximateTimeSynchronizer([sub_rgb, sub_depth], queue_size=10, slop=0.1)
        ats.registerCallback(self.cb)

        self.pub_det   = self.create_publisher(Detection2DArray, '/yolo_detector/detections', 10)
        self.pub_image = self.create_publisher(Image, '/yolo_detector/detections/image', reliable_qos)

        self.get_logger().info(f"YOLO detector started! With model {self.model_path}")

    # CameraInfo sampling 1 Hz
    def _caminfo_cb(self, msg: CameraInfo):
        self._latest_caminfo = msg  # stash only

    def _process_caminfo_1hz(self):
        if self._latest_caminfo is None:
            return
        msg = self._latest_caminfo
        self.K = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])  # fx, fy, cx, cy
        self.cam_frame = msg.header.frame_id

    def cb(self, rgb_msg, depth_msg):
        # Ensure intrinsics are available
        if self.K is None:
            now = self.get_clock().now()
            if (self._last_caminfo_warn is None) or ((now - self._last_caminfo_warn) > Duration(seconds=5.0)):
                self.get_logger().warning("Waiting for CameraInfo (intrinsics not ready)")
                self._last_caminfo_warn = now
            return

        fx, fy, cx, cy = self.K

        # Convert ROS to OpenCV
        color = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
        depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        if depth.dtype != np.float32:
            depth = depth.astype(np.float32) * self.depth_scale

        # YOLO inference
        results = self.model.predict(
            source=color, conf=self.conf_thres, iou=self.iou_thres,
            verbose=False, device=self.device
        )

        det_array = Detection2DArray()
        det_array.header = rgb_msg.header  # keep original stamp/frame
        annotated = color.copy()

        for r in results:
            if not getattr(r, "boxes", None) or len(r.boxes) == 0:
                continue
            names = getattr(r, "names", None) or getattr(self.model, "names", {}) or {}
            for box in r.boxes:
                cls_id = int(box.cls[0])
                conf = float(box.conf[0])
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().tolist()
                u, v = (x1 + x2) / 2.0, (y1 + y2) / 2.0

                # Build a Detection2D
                det = Detection2D()
                det.header = rgb_msg.header

                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = names.get(cls_id, str(cls_id))
                hyp.hypothesis.score = conf
                det.results.append(hyp)

                bb = BoundingBox2D()
                bb.center.position.x = float(u); bb.center.position.y = float(v)
                bb.size_x = float(x2 - x1); bb.size_y = float(y2 - y1)
                det.bbox = bb

                det_array.detections.append(det)

                # -draw box/label on the annotated image 
                x1_i, y1_i, x2_i, y2_i = map(int, (x1, y1, x2, y2))
                cv2.rectangle(annotated, (x1_i, y1_i), (x2_i, y2_i), (0, 255, 0), 2)
                label = f"{hyp.hypothesis.class_id} {conf:.2f}"
                cv2.putText(annotated, label, (x1_i, max(0, y1_i - 5)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)

        # Publish det results
        self.pub_det.publish(det_array)

        # Then the image with the det info drawn on it
        img_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
        img_msg.header = rgb_msg.header
        self.pub_image.publish(img_msg)

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

    def transform_point(self, point_in_cam, stamp):
        try:
            ps = PoseStamped()
            ps.header.stamp = stamp
            ps.header.frame_id = self.cam_frame
            ps.pose.position = point_in_cam
            ps.pose.orientation.w = 1.0

            trans = self.tf_buffer.lookup_transform(
                self.target_frame, self.cam_frame, rclpy.time.Time()
            )
            out = tf2_geometry_msgs.do_transform_pose(ps, trans)
            return out.position
        except Exception as e:
            self.get_logger().warning(f"TF failed: {e}")
            return Point()


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
