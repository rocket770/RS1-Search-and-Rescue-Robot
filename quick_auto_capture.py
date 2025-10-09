#!/usr/bin/env python3
import os
import math
import numpy as np
import cv2
from typing import List, Tuple

# ---- NumPy 1.24+ compat (must be top) ----
if not hasattr(np, "float"):
    np.float   = float
    np.int     = int
    np.bool    = bool
    np.object  = object
    np.complex = complex
    np.long    = int
# ------------------------------------------

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, QoSHistoryPolicy

from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import tf2_ros

# Services
from ros_gz_interfaces.srv import SetEntityPose as GZSetEntityPose
from robot_localization.srv import SetPose as RLSetPose


# =======================
# Config / Classes
# =======================
CLASS_NAMES = ["duck", "bear", "frog", "man", "monkey"]
CLASS_ID = {n: i for i, n in enumerate(CLASS_NAMES)}

# Half-sizes (meters) in a simple canonical sense:
# hx = half-width (left/right), hy = half-depth (front/back), hz = half-height (up/down)
BBOX_HALFSIZE = {
    "duck":   (0.15, 0.10, 0.12),
    "bear":   (0.50, 0.35, 0.60),
    "frog":   (0.10, 0.08, 0.05),
    "man":    (0.35, 0.30, 1.00),
    "monkey": (0.20, 0.15, 0.30),
}

# Global depth-to-size settings
SCALE_K   = 1.00   # global multiplier
SIZE_EXP  = 1.00   # exponent on distance (1.0 = true pinhole; >1 shrinks faster with distance)

# Per-class scale multipliers (post pinhole)
CLASS_SCALE = {
    "duck":   1.15,  # slightly bigger
    "bear":   1.20,  # make bigger overall
    "frog":   1.00,
    "man":    1.00,
    "monkey": 1.10,  # slightly bigger
}

# Per-class vertical center offsets (fraction of image height).
# Negative = move UP; Positive = move DOWN.
CLASS_CENTER_Y_OFFSET_FRAC = {
    "frog":  +0.06,   # move frog down a bit
    "man":   -0.16,   # move humans further up
    # others default 0
}

# Per-class horizontal center offsets (fraction of image width).
# Negative = left; Positive = right.
CLASS_CENTER_X_OFFSET_FRAC = {
    "bear":  +0.05,   # nudge bear to the right
    # others default 0
}

# Clamp limits for the box size as a fraction of the image
MIN_FRAC_W = 0.04
MIN_FRAC_H = 0.06
MAX_FRAC_W = 0.95
MAX_FRAC_H = 0.95

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


# =============================
# Single-shot capturer Node
# =============================
class OneShotCapturer(Node):
    """
    Teleports the robot, captures one image, and writes ONE centered YOLO box.
    The box size is computed from class physical size and distance to the known target.
    Center is image center with per-class X/Y offsets; no 3-D box projection.
    """
    def __init__(self,
                 img_topic="/camera/image",
                 info_topic="/camera/camera_info",
                 world_frame="map",
                 gz_set_pose_srv="/world/large_demo/set_pose",
                 rl_set_pose_srv="/set_pose",
                 model_name="husky",
                 entity_type=1,
                 settle_sec=1.5):
        super().__init__("one_shot_capturer_center_sized_offsets")
        ensure_dirs()

        self.img_topic   = img_topic
        self.info_topic  = info_topic
        self.world_frame = world_frame
        self.gz_srv_name = gz_set_pose_srv
        self.rl_srv_name = rl_set_pose_srv
        self.model_name  = model_name
        self.entity_type = int(entity_type)
        self.settle_sec  = float(settle_sec)

        self.bridge = CvBridge()
        self.fx = None
        self.fy = None

        # TF buffer not strictly needed here but harmless to keep
        self.tf_buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf, self)

        # services
        self.gz_cli = self.create_client(GZSetEntityPose, self.gz_srv_name)
        self.rl_cli = self.create_client(RLSetPose, self.rl_srv_name)
        self.get_logger().info("Waiting for services…")
        self.gz_cli.wait_for_service()
        self.rl_cli.wait_for_service()
        self.get_logger().info("Services ready.")

        # QoS
        self.info_qos = 10
        qos_img = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             history=QoSHistoryPolicy.KEEP_LAST, depth=10)
        self.img_qos = qos_img

    # --- camera info for intrinsics ---
    def wait_for_camera_info(self, timeout_sec=5.0):
        holder = {"msg": None}
        sub = self.create_subscription(CameraInfo, self.info_topic,
                                       lambda m: holder.__setitem__("msg", m), self.info_qos)
        self.get_logger().info(f"Waiting for CameraInfo on {self.info_topic}…")
        end = self.get_clock().now() + Duration(seconds=timeout_sec)
        while rclpy.ok() and holder["msg"] is None and self.get_clock().now() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.destroy_subscription(sub)

        if holder["msg"] is None:
            raise RuntimeError("Timeout waiting for CameraInfo")

        K = np.array(holder["msg"].k, dtype=float).reshape(3,3)
        self.fx, self.fy = float(K[0,0]), float(K[1,1])
        self.get_logger().info(f"Intrinsics ok: fx={self.fx:.1f} fy={self.fy:.1f}")

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

    # --- center box sized by class + distance + per-class offsets ---
    def center_box_pixels(self, W: int, H: int, cls_name: str, dist_m: float) -> Tuple[int,int,int,int]:
        """
        Returns integer (x1,y1,x2,y2) for a box centered with per-class X/Y offsets.
        Size ≈ (physical_size / dist^SIZE_EXP) * focal_length * CLASS_SCALE.
        width_phys = 2*max(hx,hy), height_phys = 2*hz
        """
        hx, hy, hz = BBOX_HALFSIZE[cls_name]
        width_phys  = 2.0 * max(hx, hy) * SCALE_K
        height_phys = 2.0 * hz * SCALE_K

        # distance term, protected
        Z = max(dist_m, 1e-3)
        Z_term = Z ** SIZE_EXP

        # base pinhole scaling
        w_pix = int(round(self.fx * (width_phys  / Z_term)))
        h_pix = int(round(self.fy * (height_phys / Z_term)))

        # per-class scale multipliers
        cscale = CLASS_SCALE.get(cls_name, 1.0)
        w_pix = int(round(w_pix * cscale))
        h_pix = int(round(h_pix * cscale))

        # Clamp to sane fractions of the image
        w_min = int(round(MIN_FRAC_W * W))
        h_min = int(round(MIN_FRAC_H * H))
        w_max = int(round(MAX_FRAC_W * W))
        h_max = int(round(MAX_FRAC_H * H))
        w_pix = max(w_min, min(w_pix, w_max))
        h_pix = max(h_min, min(h_pix, h_max))

        # Center + per-class offsets
        off_y = CLASS_CENTER_Y_OFFSET_FRAC.get(cls_name, 0.0)
        off_x = CLASS_CENTER_X_OFFSET_FRAC.get(cls_name, 0.0)
        cx = int(round(W / 2.0 + off_x * W))
        cy = int(round(H / 2.0 + off_y * H))

        x1 = max(0, cx - w_pix // 2)
        y1 = max(0, cy - h_pix // 2)
        x2 = min(W - 1, cx + w_pix // 2)
        y2 = min(H - 1, cy + h_pix // 2)
        return x1, y1, x2, y2

    def project_and_save_center(self,
                                img_msg,
                                idx: int,
                                base_pose: Tuple[float,float,float,float],
                                target: Tuple[str,float,float,float,float]):
        """
        base_pose: (bx,by,bz,byaw)  [byaw only used for teleport]
        target: (cls_name, ox, oy, oz, Rref)
        """
        cv_img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
        H, W = cv_img.shape[:2]

        cls_name, ox, oy, oz, Rref = target
        bx, by, bz, byaw = base_pose

        # Distance robot<->object center in XY (meters)
        dist_xy = math.hypot(ox - bx, oy - by)

        # Centered (with offsets), distance-scaled box
        x1, y1, x2, y2 = self.center_box_pixels(W, H, cls_name, dist_xy)

        # YOLO-normalized (cx, cy, w, h)
        cxn = ((x1 + x2) * 0.5) / W
        cyn = ((y1 + y2) * 0.5) / H
        nwn = (x2 - x1) / W
        nhn = (y2 - y1) / H

        labels = [(CLASS_ID[cls_name], cxn, cyn, nwn, nhn)]

        # save image + label
        fname = f"{idx:06d}.jpg"
        cv2.imwrite(os.path.join(IMG_DIR, fname), cv_img)
        with open(os.path.join(LBL_DIR, fname.replace(".jpg", ".txt")), "w") as f:
            for (cid, cxn, cyn, nwn, nhn) in labels:
                f.write(f"{cid} {cxn:.6f} {cyn:.6f} {nwn:.6f} {nhn:.6f}\n")

        # preview overlay
        overlay = cv_img.copy()
        cv2.rectangle(overlay, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(overlay, cls_name, (x1, max(0, y1 - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)
        cv2.imwrite(os.path.join(PREV_DIR, fname), overlay)

        return 1

    # ---- main routine: single pass over waypoints ----
    def run_once(self, waypoints: List[Tuple[float,float,float,float,float,str,float,float,float]]):
        """
        waypoints entries contain:
        (x, y, z, yaw, Rref, cls_name, ox, oy, oz)
        """
        self.wait_for_camera_info(timeout_sec=10.0)

        idx = 0
        for (x, y, z, yaw, Rref, cls_name, ox, oy, oz) in waypoints:
            self.get_logger().info(
                f"[{idx}] teleport x={x:.2f} y={y:.2f} z={z:.2f} yaw(deg)={math.degrees(yaw):.1f} "
                f"R={Rref:.2f} class={cls_name}"
            )
            ok1 = self.teleport_gazebo(x, y, z, yaw)
            ok2 = self.set_tf_pose(x, y, z, yaw)
            if not (ok1 and ok2):
                self.get_logger().warn("Teleport or SetPose failed; continuing anyway.")

            # settle
            t_end = self.get_clock().now() + Duration(seconds=self.settle_sec)
            while rclpy.ok() and self.get_clock().now() < t_end:
                rclpy.spin_once(self, timeout_sec=0.05)

            # image
            img_msg = self.grab_one_image(timeout_sec=3.0)
            if img_msg is None:
                self.get_logger().warn("No image received; skipping waypoint.")
                continue

            # centered, distance-scaled (with per-class offsets) box
            n = self.project_and_save_center(
                img_msg, idx,
                (x, y, z, yaw),
                (cls_name, ox, oy, oz, Rref)
            )
            self.get_logger().info(f"[{idx}] boxes={n}")
            idx += 1


# ====================
# run it as a script
# ====================
def main():
    rclpy.init()

    # Waypoints: (x, y, z, yaw, R, cls_name, ox, oy, oz)
    waypoints: List[Tuple[float,float,float,float,float,str,float,float,float]] = []

    targets = [
        ("man",    11.0,  1.0, 0.0),
        ("duck",  -10.0,  9.0, 0.0),
        ("frog",   10.0,  5.0, 0.0),
        ("bear",   -6.0, 11.0, 1.25),
        ("monkey",-10.0,  8.0, 0.0),
    ]
    radii = [2.0, 4.0, 6.0, 8.0, 9.0]
    angles_deg = [0, 60]
    #angles_deg = [0, 30, 60, 90, 120, 150, 180, 210, 240, 270, 300, 330]

    for (cls_name, ox, oy, oz) in targets:
        for R in radii:
            for ang in angles_deg:
                a = math.radians(ang)
                rx = ox + R * math.cos(a)
                ry = oy + R * math.sin(a)
                rz = 0.0
                yaw = math.atan2(oy - ry, ox - rx)  # face target center
                waypoints.append((rx, ry, rz, yaw, R, cls_name, ox, oy, oz))

    node = OneShotCapturer(
        img_topic="/camera/image",
        info_topic="/camera/camera_info",
        world_frame="map",
        gz_set_pose_srv="/world/large_demo/set_pose",
        rl_set_pose_srv="/set_pose",
        model_name="husky",
        entity_type=1,
        settle_sec=1.8,
    )

    try:
        node.run_once(waypoints)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
