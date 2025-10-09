#!/usr/bin/env python3
import os
import math
import numpy as np
import cv2
from typing import List, Tuple

# ---- NumPy 1.24+ compat (must be top, before tf_transformations/transforms3d) ----
if not hasattr(np, "float"):
    np.float   = float
    np.int     = int
    np.bool    = bool
    np.object  = object
    np.complex = complex
    np.long    = int
# ----------------------------------------------------------------------------------

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, QoSHistoryPolicy

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped
from cv_bridge import CvBridge
import tf2_ros
from tf_transformations import quaternion_matrix

# Services
from ros_gz_interfaces.srv import SetEntityPose as GZSetEntityPose
from robot_localization.srv import SetPose as RLSetPose


# =======================
# Your static world boxes
# =======================
CLASS_NAMES = ["duck", "bear", "frog", "man", "monkey"]
CLASS_ID = {n:i for i,n in enumerate(CLASS_NAMES)}

# half-size (x,y,z) per class (meters)
BBOX_HALFSIZE = {
    "duck":   (0.15, 0.10, 0.12),
    "bear":   (0.50, 0.35, 0.60),
    "frog":   (0.10, 0.08, 0.05),
    "man":    (0.35, 0.30, 1.00),
    "monkey": (0.20, 0.15, 0.30),
}

# (name, class, (x,y,z), (roll,pitch,yaw))  -- in WORLD frame (your map)
STATIC_OBJECTS = [
    ("duck",   "duck",   (-10.0,  9.0, 0.0),    (0.0,    0.0,     0.0)),
    ("frog",   "frog",   ( 10.0,  5.0, 0.0),    (0.0,    0.0,     0.0)),
    ("bear",   "bear",   ( -6.0, 11.0, 1.25),   (1.5708, 0.0,    -1.0)),
    ("man",    "man",    ( 11.0,  1.0, 0.0),    (1.5708, 0.0, -1.5708)),
    ("monkey", "monkey", (-10.0,  8.0, 0.0),    (0.0,    0.0,     0.0)),
]

# =========
# IO paths
# =========
DATA_ROOT = "dataset"
IMG_DIR   = os.path.join(DATA_ROOT, "images")
LBL_DIR   = os.path.join(DATA_ROOT, "labels")
PREV_DIR  = os.path.join(DATA_ROOT, "previews")

def ensure_dirs():
    os.makedirs(IMG_DIR, exist_ok=True)
    os.makedirs(LBL_DIR, exist_ok=True)
    os.makedirs(PREV_DIR, exist_ok=True)

# =========
# Math bits
# =========
def tf_to_matrix(msg: TransformStamped):
    t = msg.transform.translation
    q = msg.transform.rotation
    M = quaternion_matrix([q.x, q.y, q.z, q.w])
    M[0, 3] = t.x
    M[1, 3] = t.y
    M[2, 3] = t.z
    return M

def rpy_to_matrix(x, y, z, roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    Rz = np.array([[cy, -sy, 0.0],
                   [sy,  cy, 0.0],
                   [0.0, 0.0, 1.0]])
    Ry = np.array([[ cp, 0.0,  sp],
                   [0.0, 1.0, 0.0],
                   [-sp, 0.0,  cp]])
    Rx = np.array([[1.0, 0.0, 0.0],
                   [0.0,  cr,-sr],
                   [0.0,  sr, cr]])
    R = Rz @ Ry @ Rx
    T = np.eye(4, dtype=float)
    T[:3, :3] = R
    T[0, 3], T[1, 3], T[2, 3] = x, y, z
    return T

def corners_from_halfsizes(hsx, hsy, hsz):
    xs = [-hsx, hsx]; ys = [-hsy, hsy]; zs = [-hsz, hsz]
    return np.array([[X, Y, Z] for X in xs for Y in ys for Z in zs], dtype=float)

# =============================
# Single-shot capturer Node
# =============================
class OneShotCapturer(Node):
    """
    Minimal utility:
      - waits once for CameraInfo, grabs K and cam_frame
      - looks up static T(cam <- base_link)
      - for each waypoint: teleport Gazebo + set TF pose, sleep, capture ONE image, project, save
    """
    def __init__(self,
                 img_topic="/camera/image",
                 info_topic="/camera/camera_info",
                 world_frame="map",
                 base_link="base_link",
                 cam_frame="camera_rgb_optical_frame",
                 gz_set_pose_srv="/world/large_demo/set_pose",
                 rl_set_pose_srv="/set_pose",
                 model_name="husky",
                 entity_type=1,
                 settle_sec=1.5):
        super().__init__("one_shot_capturer")
        ensure_dirs()

        self.img_topic   = img_topic
        self.info_topic  = info_topic
        self.world_frame = world_frame
        self.base_link   = base_link
        self.cam_frame   = cam_frame
        self.gz_srv_name = gz_set_pose_srv
        self.rl_srv_name = rl_set_pose_srv
        self.model_name  = model_name
        self.entity_type = int(entity_type)
        self.settle_sec  = float(settle_sec)

        self.bridge = CvBridge()
        self.K = None
        self.cam_frame_auto = None

        # TF
        self.tf_buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf, self)
        self.T_cam_base = None

        # services
        self.gz_cli = self.create_client(GZSetEntityPose, self.gz_srv_name)
        self.rl_cli = self.create_client(RLSetPose, self.rl_srv_name)
        self.get_logger().info("Waiting for services…")
        self.gz_cli.wait_for_service()
        self.rl_cli.wait_for_service()
        self.get_logger().info("Services ready.")

        # subscribers for one-shot waits
        self.info_qos = 10
        qos_img = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             history=QoSHistoryPolicy.KEEP_LAST, depth=10)
        self.img_qos = qos_img

    # --- helpers to wait for one message ---
    def wait_for_camera_info(self, timeout_sec=5.0):
        msg_holder = {"msg": None}
        sub = self.create_subscription(CameraInfo, self.info_topic,
                                       lambda m: msg_holder.__setitem__("msg", m), self.info_qos)
        self.get_logger().info(f"Waiting for CameraInfo on {self.info_topic}…")
        end = self.get_clock().now() + Duration(seconds=timeout_sec)
        while rclpy.ok() and msg_holder["msg"] is None and self.get_clock().now() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.destroy_subscription(sub)

        if msg_holder["msg"] is None:
            raise RuntimeError("Timeout waiting for CameraInfo")
        msg = msg_holder["msg"]
        self.K = np.array(msg.k, dtype=float).reshape(3,3)
        if not self.cam_frame:
            self.cam_frame_auto = (msg.header.frame_id or "").strip()
            self.cam_frame = self.cam_frame_auto or self.cam_frame
        self.get_logger().info(f"Got K and cam_frame='{self.cam_frame}'")
        return True

    def lookup_T_cam_base(self, timeout_sec=2.0):
        self.get_logger().info(f"Looking up T({self.cam_frame} <- {self.base_link})…")
        end = self.get_clock().now() + Duration(seconds=timeout_sec)
        while rclpy.ok() and self.get_clock().now() < end:
            try:
                t = self.tf_buf.lookup_transform(self.cam_frame, self.base_link, rclpy.time.Time())
                self.T_cam_base = tf_to_matrix(t)
                self.get_logger().info("Got static camera extrinsic (cam<-base_link).")
                return True
            except Exception:
                rclpy.spin_once(self, timeout_sec=0.1)
        raise RuntimeError("Failed to get T(cam<-base_link)")

    # --- services ---
    def teleport_gazebo(self, x, y, z, yaw):
        req = GZSetEntityPose.Request()
        req.entity.name = str(self.model_name)
        req.entity.type = self.entity_type
        req.pose.position.x = float(x)
        req.pose.position.y = float(y)
        req.pose.position.z = float(z)
        req.pose.orientation.x = 0.0
        req.pose.orientation.y = 0.0
        req.pose.orientation.z = math.sin(yaw/2.0)
        req.pose.orientation.w = math.cos(yaw/2.0)
        fut = self.gz_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=2.0)
        return fut.result() is not None

    def set_tf_pose(self, x, y, z, yaw):
        req = RLSetPose.Request()
        req.pose.header.frame_id = self.world_frame
        req.pose.pose.pose.position.x = float(x)
        req.pose.pose.pose.position.y = float(y)
        req.pose.pose.pose.position.z = float(z)
        req.pose.pose.pose.orientation.x = 0.0
        req.pose.pose.pose.orientation.y = 0.0
        req.pose.pose.pose.orientation.z = math.sin(yaw/2.0)
        req.pose.pose.pose.orientation.w = math.cos(yaw/2.0)
        req.pose.pose.covariance = [0.0]*36
        fut = self.rl_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=2.0)
        return fut.result() is not None

    # --- one frame grab ---
    def grab_one_image(self, timeout_sec=3.0):
        holder = {"img": None}
        sub = self.create_subscription(Image, self.img_topic,
                                       lambda m: holder.__setitem__("img", m), self.img_qos)
        end = self.get_clock().now() + Duration(seconds=timeout_sec)
        while rclpy.ok() and holder["img"] is None and self.get_clock().now() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        self.destroy_subscription(sub)
        return holder["img"]

    # --- projection of STATIC_OBJECTS with commanded pose ---
    def project_and_save(self, img_msg, idx: int, base_xyz_yaw: Tuple[float,float,float,float]):
        cv_img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
        H, W = cv_img.shape[:2]
        fx, fy, cx, cy = self.K[0,0], self.K[1,1], self.K[0,2], self.K[1,2]

        # camera pose in world: T(cam<-world) = T(cam<-base) * T(base<-world)
        bx, by, bz, byaw = base_xyz_yaw
        T_base_world = rpy_to_matrix(bx, by, bz, 0.0, 0.0, byaw)  # world->base
        T_cam_world  = self.T_cam_base @ T_base_world            # world->cam

        labels = []
        preview_boxes = []
        for name, cls_name, (ox, oy, oz), (r, p, yy) in STATIC_OBJECTS:
            M_world_obj = rpy_to_matrix(ox, oy, oz, r, p, yy)  # world->obj
            corners_obj = corners_from_halfsizes(*BBOX_HALFSIZE[cls_name])
            corners_obj_h   = np.concatenate([corners_obj, np.ones((8,1))], axis=1).T  # (4,8)
            corners_world_h = M_world_obj @ corners_obj_h
            # Obj->Cam: (cam<-world)^-1 * (world->obj)
            corners_cam_h   = np.linalg.inv(T_cam_world) @ corners_world_h
            corners_cam     = (corners_cam_h[:3, :] / corners_cam_h[3, :]).T

            pix = []
            for xw, yw, zw in corners_cam:
                if zw <= 0.0:
                    pix.append(None)
                    continue
                u = fx*(xw/zw) + cx
                v = fy*(yw/zw) + cy
                pix.append((u, v))
            val = [(u,v) for uv in pix if uv is not None for (u,v) in [uv]]
            if not val:
                continue
            us = [u for (u,_) in val]; vs = [v for (_,v) in val]
            umin, umax = max(0, min(W-1, min(us))), max(0, min(W-1, max(us)))
            vmin, vmax = max(0, min(H-1, min(vs))), max(0, min(H-1, max(vs)))
            bw, bh = max(0.0, umax-umin), max(0.0, vmax-vmin)
            if bw < 2 or bh < 2:
                continue
            cxn = (umin+umax)*0.5 / W
            cyn = (vmin+vmax)*0.5 / H
            nwn = bw / W
            nhn = bh / H
            labels.append((CLASS_ID[cls_name], cxn, cyn, nwn, nhn))
            preview_boxes.append((int(umin), int(vmin), int(umax), int(vmax), cls_name))

        # save
        fname = f"{idx:06d}.jpg"
        cv2.imwrite(os.path.join(IMG_DIR, fname), cv_img)
        with open(os.path.join(LBL_DIR, fname.replace(".jpg", ".txt")), "w") as f:
            for (cid, cxn, cyn, nwn, nhn) in labels:
                f.write(f"{cid} {cxn:.6f} {cyn:.6f} {nwn:.6f} {nhn:.6f}\n")

        # preview
        overlay = cv_img.copy()
        if labels:
            for (x1,y1,x2,y2,nm) in preview_boxes:
                cv2.rectangle(overlay, (x1,y1), (x2,y2), (0,255,0), 2)
                cv2.putText(overlay, nm, (x1, max(0,y1-6)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)
        else:
            cv2.putText(overlay, "NO BOXES", (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255,255,255), 2)
        cv2.imwrite(os.path.join(PREV_DIR, fname), overlay)

        return len(labels)

    # ---- main routine: single pass over waypoints ----
    def run_once(self, waypoints: List[Tuple[float,float,float,float]]):
        # 1) intrinsics + cam_frame
        self.wait_for_camera_info(timeout_sec=10.0)
        # 2) T(cam<-base_link)
        self.lookup_T_cam_base(timeout_sec=5.0)

        idx = 0
        for (x, y, z, yaw) in waypoints:
            self.get_logger().info(f"[{idx}] teleporting to x={x:.2f} y={y:.2f} z={z:.2f} yaw(deg)={math.degrees(yaw):.1f}")
            ok1 = self.teleport_gazebo(x, y, z, yaw)
            ok2 = self.set_tf_pose(x, y, z, yaw)
            if not (ok1 and ok2):
                self.get_logger().warn("Teleport or SetPose failed; continuing anyway.")

            # wait for settle
            t_end = self.get_clock().now() + Duration(seconds=self.settle_sec)
            while rclpy.ok() and self.get_clock().now() < t_end:
                rclpy.spin_once(self, timeout_sec=0.05)

            # grab exactly one image
            img_msg = self.grab_one_image(timeout_sec=3.0)
            if img_msg is None:
                self.get_logger().warn("No image received; skipping waypoint.")
                continue

            # project and save
            n = self.project_and_save(img_msg, idx, (x,y,z,yaw))
            self.get_logger().info(f"[{idx}] boxes={n}")
            idx += 1


# ====================
# run it as a script
# ====================
def main():
    rclpy.init()

    # Waypoints: (x, y, z, yaw)
    # Build a small tour around "man" at (11,1). Add more as needed.
    waypoints = []
    targets = [
        ("man",  11.0,  1.0, 0.0),
        ("duck", -10.0, 9.0, 0.0),
        ("frog", 10.0,  5.0, 0.0),
        ("bear", -6.0, 11.0, 1.25),
        ("monkey",-10.0,8.0, 0.0),
    ]
    radii = [4.0, 6.0, 8.0]
    angles_deg = [0, 45, 90, 135, 180, 225, 270, 315]

    for (_nm, ox, oy, oz) in targets:
        for R in radii:
            for ang in angles_deg:
                a = math.radians(ang)
                rx = ox + R*math.cos(a)
                ry = oy + R*math.sin(a)
                rz = 0.0  # camera height is in TF (base->cam), we keep base at ground
                yaw = math.atan2(oy - ry, ox - rx)  # face target center
                waypoints.append((rx, ry, rz, yaw))

    node = OneShotCapturer(
        img_topic="/camera/image",
        info_topic="/camera/camera_info",
        world_frame="map",
        base_link="base_link",
        cam_frame="camera_rgb_optical_frame",
        gz_set_pose_srv="/world/large_demo/set_pose",
        rl_set_pose_srv="/set_pose",
        model_name="husky",
        entity_type=1,              # 1 = model
        settle_sec=1.8,             # bump if needed
    )

    try:
        node.run_once(waypoints)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
