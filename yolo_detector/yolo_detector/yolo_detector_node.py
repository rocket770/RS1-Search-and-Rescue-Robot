#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

import numpy as np
import cv2
from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point, PoseStamped

from ultralytics import YOLO
import torch

import message_filters

from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose, BoundingBox2D

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from math import sqrt

import threading, subprocess, re, sys, time


class IgnitionPoseWatcher:

    def __init__(self, topic: str, cli: str = "ign"):
        self.topic = topic
        self.cli = cli  
        self._poses = {}            
        self._lock = threading.Lock()
        self._stop = False
        self._t = threading.Thread(target=self._run, daemon=True)
        self._t.start()

    def stop(self):
        self._stop = True

    def get_xyz(self, name: str):
        with self._lock:
            return self._poses.get(name)

    def _run(self):
        cmd = [self.cli, "topic", "-t", self.topic, "-e"]
        try:
            p = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
                text=True,
                bufsize=1,
                universal_newlines=True,
            )
        except FileNotFoundError:
            print(f"[IgnitionPoseWatcher] '{self.cli}' not found in PATH", file=sys.stderr)
            return

        name = None; x = y = z = None
        in_pose = False; in_pos = False
        rx_name = re.compile(r'^\s*name:\s*"([^"]+)"')
        rx_xyz  = re.compile(r'^\s*([xyz]):\s*([-+0-9.eE]+)')

        while not self._stop:
            line = p.stdout.readline()
            if not line:
                time.sleep(0.02)
                continue
            s = line.strip()

            if s.startswith("pose {"):
                in_pose = True; in_pos = False
                name = None; x = y = z = None
                continue

            if in_pose and s == "}":
                # end of a single pose block
                if name is not None and x is not None and y is not None:
                    with self._lock:
                        self._poses[name] = (x, y, z if z is not None else 0.0)
                in_pose = False; in_pos = False
                continue

            if not in_pose:
                continue

            m = rx_name.match(s)
            if m:
                name = m.group(1)
                continue

            if s.startswith("position"):
                in_pos = True
                continue
            if in_pos and s == "}":
                in_pos = False
                continue

            if in_pos:
                m2 = rx_xyz.match(s)
                if m2:
                    axis, val = m2.group(1), float(m2.group(2))
                    if axis == "x": x = val
                    elif axis == "y": y = val
                    elif axis == "z": z = val


class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_detector')

        self.declare_parameter('model_path', 'runs_poc/03_stageB_full/weights/best_stable_best_result.pt')
        self.declare_parameter('conf_thres', 0.25)
        self.declare_parameter('iou_thres', 0.5)
        self.declare_parameter('target_frame', 'map')

        self.declare_parameter('ign_topic', '/world/large/pose/info')
        self.declare_parameter('ign_cli', 'ign')  

        self.declare_parameter('name_aliases', '')

        self.model_path   = self.get_parameter('model_path').value
        self.conf_thres   = float(self.get_parameter('conf_thres').value)
        self.iou_thres    = float(self.get_parameter('iou_thres').value)
        self.target_frame = self.get_parameter('target_frame').value

        self.ign_topic = self.get_parameter('ign_topic').value
        self.ign_cli   = self.get_parameter('ign_cli').value

        aliases_str = self.get_parameter('name_aliases').value.strip()
        self.name_alias = self._parse_aliases(aliases_str)

        self.device = 0 if torch.cuda.is_available() else "cpu"
        self.get_logger().info(f"Using device: {'CUDA:0' if self.device == 0 else 'CPU'}")

        try:
            self.model = YOLO(self.model_path)
            self.model.to(self.device)
        except Exception as e:
            self.get_logger().error(f"Failed to load YOLO model at {self.model_path}: {e}")
            raise

        self.bridge = CvBridge()

        self.get_logger().info(f"Starting Pose_V watcher on {self.ign_cli} topic: {self.ign_topic}")
        self.pose_watcher = IgnitionPoseWatcher(self.ign_topic, cli=self.ign_cli)

        reliable_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                                  history=HistoryPolicy.KEEP_LAST)

        sub_rgb   = message_filters.Subscriber(self, Image, '/camera/image', qos_profile=reliable_qos)

        ats = message_filters.ApproximateTimeSynchronizer([sub_rgb], queue_size=10, slop=0.1)
        ats.registerCallback(self.cb)

        self.pub_det    = self.create_publisher(Detection2DArray, '/yolo_detector/detections', 10)
        self.pub_image  = self.create_publisher(Image, '/yolo_detector/detections/image', reliable_qos)
        self.pub_global = self.create_publisher(Detection2DArray, '/yolo_detector/global_detections', 10)

        # persistent global list: {'class': str, 'point': Point, 'score': float, 'count': int}
        self.global_dets = []

        self.get_logger().info(f"YOLO detector started with model {self.model_path} (no TF/depth math)")

    @staticmethod
    def _parse_aliases(s: str):
        if not s:
            return {}
        out = {}
        for pair in s.split(','):
            if ':' in pair:
                k, v = pair.split(':', 1)
                k = k.strip(); v = v.strip()
                if k:
                    out[k] = v
        return out

    @staticmethod
    def _dist_xy(a: Point, b: Point) -> float:
        return ((a.x - b.x)**2 + (a.y - b.y)**2) ** 0.5

    def _add_to_global_if_new(self, class_id: str, p: Point, score: float) -> bool:
        for g in self.global_dets:
            if g['class'] != class_id:
                continue
            if self._dist_xy(g['point'], p) <= 0.5: 
                g['score'] = max(g['score'], score)
                g['count'] += 1
                return False
        self.global_dets.append({'class': class_id, 'point': Point(x=p.x, y=p.y, z=p.z), 'score': score, 'count': 1})
        return True

    def _build_global_detection_array(self, stamp) -> Detection2DArray:
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

    def cb(self, rgb_msg: Image):
     
        # Convert ROS to OpenCV
        try:
            color = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
        except Exception as e:
            self.get_logger().warning(f"cv_bridge conversion failed: {e}")
            return

        # Run YOLO
        try:
            results = self.model.predict(
                source=color,
                conf=self.conf_thres,
                iou=self.iou_thres,
                verbose=False,
                device=self.device
            )
        except Exception as e:
            self.get_logger().warning(f"YOLO inference failed: {e}")
            return

        det_array = Detection2DArray()
        det_array.header = rgb_msg.header
        annotated = color.copy()

        any_global_added = False

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

                # --- lk up world pose by entity name from Ignition/Gazebo ---
                ign_name = self.name_alias.get(label_name, label_name)
                xyz = self.pose_watcher.get_xyz(ign_name)
                
                if xyz is None:
                    # no pose available with that name in the current Pose_V stream
                    self.get_logger().info(
                        f"found box but could not find pose {label_name} with ign_name {ign_name}"
                    )
                    continue

                world_x, world_y, world_z = xyz

                out = PoseStamped()
                out.header = rgb_msg.header
                out.header.frame_id = self.target_frame
                out.pose.position.x = float(world_x)
                out.pose.position.y = float(world_y)
                out.pose.position.z = float(world_z)
                out.pose.orientation.w = 1.0

                # Dedup into global list
                added = self._add_to_global_if_new(label_name, out.pose.position, conf)
                any_global_added = any_global_added or added

                # Per-frame detection message
                det = Detection2D()
                det.header = rgb_msg.header

                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = label_name
                hyp.hypothesis.score = conf
                hyp.pose.pose.position = out.pose.position
                det.results.append(hyp)

                bb = BoundingBox2D()
                bb.center.position.x = float(u)
                bb.center.position.y = float(v)
                bb.size_x = float(x2 - x1)
                bb.size_y = float(y2 - y1)
                det.bbox = bb

                det_array.detections.append(det)

                # Draw on image for rvis debug or in future we can use for our ui
                x1_i, y1_i, x2_i, y2_i = map(int, (x1, y1, x2, y2))
                cv2.rectangle(annotated, (x1_i, y1_i), (x2_i, y2_i), (0, 255, 0), 2)
                cv2.putText(annotated, f"{label_name} {conf:.2f}",
                            (x1_i, max(0, y1_i - 5)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)

        # Publish per-frame detections
        if len(det_array.detections) > 0:
            self.pub_det.publish(det_array)

        # Publish global detections but we only do it wheen new change
        if any_global_added:
            global_msg = self._build_global_detection_array(det_array.header.stamp)
            self.pub_global.publish(global_msg)

        # Publish annotated image (always)
        try:
            img_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
            img_msg.header = rgb_msg.header
            self.pub_image.publish(img_msg)
        except Exception as e:
            self.get_logger().warning(f"Failed to publish annotated image: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.pose_watcher.stop()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
