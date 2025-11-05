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


from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from tf2_ros import Buffer, TransformListener
from rclpy.qos import qos_profile_sensor_data

import math

class TwoDTo3DPose:

    def __init__(self,
                 depth_topic: str = '/camera/depth/image',
                 camera_info_topic: str = '/camera/camera_info',
                 camera_frame: str = 'camera_link',
                 target_frame: str = 'map',
                 depth_scale: float = 1.0,
                 subscribe_to_depth: bool = True):
     
        self.depth_topic = depth_topic
        self.camera_info_topic = camera_info_topic
        self.camera_frame = camera_frame
        self.target_frame = target_frame
        self.depth_scale = float(depth_scale)

   
        self.bridge = CvBridge()   # Ros image messages to OpenCV images
        self._depth = None         # Stores data from depth topic
        self._depth_stamp = None   # Timestamp of depth message captured
        self._K = None             # stores data from camera info topic intrinsics
        self._Kinv = None          # inverse of K calculated from k

        #
        self._node = rclpy.create_node('two_d_to_three_d_pose_internal')     

     
        self._tf_buffer = Buffer()                                            # object to track transforms from relative poses and data from tf and tf_static
        self._tf_listener = TransformListener(self._tf_buffer, self._node)    # listens to tf and tf_static topics and puts data into the buffer

    
        self._sub_caminfo = self._node.create_subscription(                  # listens to camera info topic
            CameraInfo,                                                      # inorder to build the camera intrinics
            self.camera_info_topic, 
            self._on_caminfo,                                                # callback function when message is received
            qos_profile_sensor_data                                          
        )

        # Only subscribe to depth if requested
        if subscribe_to_depth:
            self._sub_depth = self._node.create_subscription(                # listens to depth topic 
                Image,                                                       # inorder to get depth data and timestamp
                self.depth_topic, 
                self._on_depth, 
                qos_profile_sensor_data
            )
            self._node.get_logger().info(f"Subscribed to depth: {self.depth_topic}")
        else:
            self._node.get_logger().info("Depth will be fed manually (no auto-subscription)")

        # background spin for subs/TF
        self._stop = False                             
        self._spin_thread = threading.Thread(target=self._spin, daemon=True)  
        self._spin_thread.start()  

    

    def stop(self):
        self._stop = True
        try:
            self._node.destroy_node()
        except Exception:
            pass

    def _spin(self):
        while rclpy.ok() and not self._stop:
            rclpy.spin_once(self._node, timeout_sec=0.1)



    def _on_caminfo(self, msg: CameraInfo):                       # callback function when camera info message is received
        fx, fy, cx, cy = msg.k[0], msg.k[4], msg.k[2], msg.k[5]   # extract intrinsics from camera info sequence
        self._K = np.array([[fx, 0.0, cx],
                            [0.0, fy, cy],
                            [0.0, 0.0, 1.0]], dtype=np.float64)   # 64-bit float numpy array for camera intrinsics
        try:
            self._Kinv = np.linalg.inv(self._K)                   # calculate inverse of K
            self._node.get_logger().info(f"Camera intrinsics received: fx={fx:.1f}, fy={fy:.1f}, cx={cx:.1f}, cy={cy:.1f}")
        except np.linalg.LinAlgError:     
            self._K = None
            self._Kinv = None
            self._node.get_logger().warn("Camera matrix not invertible")

    def _on_depth(self, msg: Image):                                               # callback function when depth message is received to store depth and timstamp
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")   # convert ros depth message to opencv image numpy array
            if img.dtype != np.float32 and img.dtype != np.float64:                # check if depth image is not in float 32 or 64 format
                img = img.astype(np.float32) * self.depth_scale                    # convert to float32 and scale  to depth scale
            self._depth = img                                                      # store depth image
            self._depth_stamp = msg.header.stamp                                   # store timestamp of depth message
            self._node.get_logger().debug(f"Depth received: shape={img.shape}, dtype={img.dtype}, range=[{img.min():.2f}, {img.max():.2f}]")
        except Exception as e:
            self._node.get_logger().warn(f"Depth conversion failed: {e}")
            self._depth = None
            self._depth_stamp = None

   

    def get_xyz(self, bbox, stamp=None):      
       
        if not isinstance(bbox, (tuple, list)) or len(bbox) != 4:     # check if bbox is a tuple or list is not length 4
            return None
        if self._Kinv is None or self._depth is None:
            if self._Kinv is None:
                self._node.get_logger().debug("get_xyz failed: K matrix not available")
            if self._depth is None:
                self._node.get_logger().debug("get_xyz failed: depth not available")
            return None

        x1, y1, x2, y2 = bbox                     # unpack bbox coordinates

        u = int(round((x1 + x2) * 0.5))
        v = int(round(y2))

        H, W = self._depth.shape[:2]              # get height and width of depth image (first 2 entries)
        if v < 0 or v >= H or u < 0 or u >= W:
            self._node.get_logger().debug(f"get_xyz failed: pixel ({u},{v}) out of bounds ({W}x{H})")
            return None

    
        z = float(self._depth[v, u])                    # get depth value at pixel (u,v)
        if not np.isfinite(z) or z <= 0.0:
            self._node.get_logger().debug(f"get_xyz failed: invalid depth {z} at pixel ({u},{v})")
            return None

    
        pix = np.array([u, v, 1.0], dtype=np.float64)   # pixel coordinates in homogeneous form
        Xc = self._Kinv @ pix                           # camera coordinates Xc = K^-1 * [u,v,1]^T
        Xc *= z                                         # scale by depth to get 3D camera coordinates

        pt = PointStamped()                             # create PointStamped message for 3d point and timestamp
        pt.header.frame_id = self.camera_frame          # set frame_id to camera frame
        pt.header.stamp = stamp if stamp is not None else (self._depth_stamp or self._node.get_clock().now().to_msg())   # set timestamp
        pt.point.x, pt.point.y, pt.point.z = float(Xc[0]), float(Xc[1]), float(Xc[2])                                    # set point coordinates

        try:
            out = self._tf_buffer.transform(pt, self.target_frame, timeout=Duration(seconds=0.1))    # transform point from camera frame to world frame using TF
            return (out.point.x, out.point.y, out.point.z)                                           
        except Exception as e:                                                                       
            self._node.get_logger().debug(f"TF {self.camera_frame}->{self.target_frame} failed: {e}")  
            return None



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

        self.declare_parameter('rgb_topic', '/camera/image')
        self.declare_parameter('depth_topic', '/camera/depth/image')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('camera_frame', 'camera_link')
        self.declare_parameter('depth_scale', 1.0)

        self.model_path   = self.get_parameter('model_path').value
        self.conf_thres   = float(self.get_parameter('conf_thres').value)
        self.iou_thres    = float(self.get_parameter('iou_thres').value)
        self.target_frame = self.get_parameter('target_frame').value

        self.ign_topic = self.get_parameter('ign_topic').value
        self.ign_cli   = self.get_parameter('ign_cli').value

        aliases_str = self.get_parameter('name_aliases').value.strip()
        self.name_alias = self._parse_aliases(aliases_str)

        rgb_topic = self.get_parameter('rgb_topic').value
        depth_topic = self.get_parameter('depth_topic').value
        camera_info_topic = self.get_parameter('camera_info_topic').value
        camera_frame = self.get_parameter('camera_frame').value
        depth_scale = float(self.get_parameter('depth_scale').value)

        self.device = 0 if torch.cuda.is_available() else "cpu"
        self.get_logger().info(f"Using device: {'CUDA:0' if self.device == 0 else 'CPU'}")

        try:
            self.model = YOLO(self.model_path)
            self.model.to(self.device)
            self.get_logger().info(f"YOLO model loaded successfully from {self.model_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to load YOLO model at {self.model_path}: {e}")
            raise

        self.bridge = CvBridge()

        self.get_logger().info("Starting 2D->3D projector (depth + camera_info + TF)")
        self.pose_watcher = TwoDTo3DPose(
            depth_topic=depth_topic,
            camera_info_topic=camera_info_topic,
            camera_frame=camera_frame,
            target_frame=self.target_frame,
            depth_scale=depth_scale,
            subscribe_to_depth=False  # We'll sync manually
        )

        
        reliable_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                                  history=HistoryPolicy.KEEP_LAST)

        # Subscribe to BOTH RGB and Depth
        self.get_logger().info(f"Subscribing to RGB: {rgb_topic}")
        self.get_logger().info(f"Subscribing to Depth: {depth_topic}")
        
        self.sub_rgb = message_filters.Subscriber(self, Image, rgb_topic, qos_profile=reliable_qos)
        self.sub_depth = message_filters.Subscriber(self, Image, depth_topic, qos_profile=reliable_qos)
        
        # Synchronize them
        self.ats = message_filters.ApproximateTimeSynchronizer(
            [self.sub_rgb, self.sub_depth],
            queue_size=10, 
            slop=0.5
        )
        self.ats.registerCallback(self.cb)
        self.get_logger().info("Message synchronizer configured (slop=0.05s)")

        self.pub_det    = self.create_publisher(Detection2DArray, '/yolo_detector/detections', 10)
        self.pub_image = self.create_publisher(Image, '/yolo_detector/detections/image', qos_profile_sensor_data)
        self.pub_global = self.create_publisher(Detection2DArray, '/yolo_detector/global_detections', 10)

        # persistent global list: {'class': str, 'point': Point, 'score': float, 'count': int}
        self.global_dets = []

        self.callback_count = 0

        self.get_logger().info(f"YOLO detector started with synchronized RGB+Depth from {rgb_topic} and {depth_topic}")
        self.get_logger().info("Waiting for synchronized messages...")

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
        
        if math.isnan(p.x) or math.isnan(p.y) or math.isnan(p.z):
            return False

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

    def cb(self, rgb_msg: Image, depth_msg: Image):
        self.callback_count += 1
        
        # LOG 1: Callback triggered
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"CALLBACK #{self.callback_count} TRIGGERED")
        self.get_logger().info(f"RGB timestamp: {rgb_msg.header.stamp.sec}.{rgb_msg.header.stamp.nanosec:09d}")
        self.get_logger().info(f"Depth timestamp: {depth_msg.header.stamp.sec}.{depth_msg.header.stamp.nanosec:09d}")
        self.get_logger().info(f"RGB encoding: {rgb_msg.encoding}, size: {rgb_msg.width}x{rgb_msg.height}")
        self.get_logger().info(f"Depth encoding: {depth_msg.encoding}, size: {depth_msg.width}x{depth_msg.height}")
        
        # Manually update depth in pose_watcher for synchronization
        self.pose_watcher._on_depth(depth_msg)
        
        # LOG 2: Depth status
        if self.pose_watcher._depth is not None:
            self.get_logger().info(f"Depth loaded: shape={self.pose_watcher._depth.shape}, "
                                 f"range=[{self.pose_watcher._depth.min():.2f}, {self.pose_watcher._depth.max():.2f}]")
        else:
            self.get_logger().error("Depth is NONE after feeding!")
        
        self.get_logger().info(f"K matrix available: {self.pose_watcher._K is not None}")
        if self.pose_watcher._K is not None:
            self.get_logger().info(f"K matrix:\n{self.pose_watcher._K}")

        # Convert ROS to OpenCV
        try:
            color = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
            self.get_logger().info(f"RGB converted: shape={color.shape}, dtype={color.dtype}")
        except Exception as e:
            self.get_logger().error(f"cv_bridge conversion failed: {e}")
            return

        # Run YOLO
        self.get_logger().info(f"Running YOLO with conf={self.conf_thres}, iou={self.iou_thres}...")
        try:
            results = self.model.predict(
                source=color,
                conf=self.conf_thres,
                iou=self.iou_thres,
                verbose=False,
                device=self.device
            )
            self.get_logger().info(f"YOLO complete. Results count: {len(results)}")
        except Exception as e:
            self.get_logger().error(f"YOLO inference failed: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())
            return

        det_array = Detection2DArray()
        det_array.header = rgb_msg.header
        annotated = color.copy()

        any_global_added = False

        for idx, r in enumerate(results):
            self.get_logger().info(f"Processing result {idx+1}/{len(results)}")
            self.get_logger().info(f"  Has 'boxes' attribute: {hasattr(r, 'boxes')}")
            
            if not getattr(r, "boxes", None):
                self.get_logger().info("  No 'boxes' attribute or it's None")
                continue
            
            num_boxes = len(r.boxes)
            self.get_logger().info(f"  Number of boxes: {num_boxes}")
            
            if num_boxes == 0:
                self.get_logger().info("  No boxes detected (empty)")
                continue

            names = getattr(r, "names", None) or getattr(self.model, "names", {}) or {}
            self.get_logger().info(f"  Class names available: {list(names.values())}")
            
            for box_idx, box in enumerate(r.boxes):
                cls_id = int(box.cls[0])
                label_name = names.get(cls_id, str(cls_id))
                conf = float(box.conf[0])
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().tolist()
                u, v = (x1 + x2) / 2.0, (y1 + y2) / 2.0

                self.get_logger().info(f"  Box {box_idx+1}: class='{label_name}' conf={conf:.3f} "
                                     f"bbox=[{x1:.0f},{y1:.0f},{x2:.0f},{y2:.0f}]")

                # --- 2D->3D: compute world point from bbox bottom-centre via depth + TF ---
                xyz = self.pose_watcher.get_xyz((x1, y1, x2, y2), stamp=rgb_msg.header.stamp)


                if xyz is None:
                    self.get_logger().warn(f"Could not compute 3D point for '{label_name}', using NaN")
                    world_x, world_y, world_z = float('nan'), float('nan'), float('nan')  # ← Invalid but passes through
                else:
                    self.get_logger().info(f"3D position: ({xyz[0]:.3f}, {xyz[1]:.3f}, {xyz[2]:.3f})")
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
        self.get_logger().info(f"Total detections to publish: {len(det_array.detections)}")
        
        if len(det_array.detections) > 0:
            self.pub_det.publish(det_array)
            self.get_logger().info("✓ Published per-frame detections")
        else:
            self.get_logger().info("✗ No detections to publish")

        # Publish global detections but we only do it when new change
        if any_global_added:
            global_msg = self._build_global_detection_array(det_array.header.stamp)
            self.pub_global.publish(global_msg)
            self.get_logger().info("✓ Published global detections (new objects added)")

        # Publish annotated image (always)
        try:
            img_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
            img_msg.header = rgb_msg.header
            self.pub_image.publish(img_msg)
            self.get_logger().info("✓ Published annotated image")
        except Exception as e:
            self.get_logger().error(f"Failed to publish annotated image: {e}")
        
        self.get_logger().info("=" * 60)


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