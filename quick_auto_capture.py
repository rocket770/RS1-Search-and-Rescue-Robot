#!/usr/bin/env python3
# requires the ign bridge to be running

import os
import re
import math
import numpy as np
import cv2
from typing import List, Tuple, Optional
from datetime import datetime

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
import argparse
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, QoSHistoryPolicy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# Services (only for teleporting the robot & setting its pose)
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
    "duck":   (0.15, 0.1, 0.17),
    "bear":   (0.70, 0.55, 0.75),
    "frog":   (0.10, 0.08, 0.05),
    "man":    (0.35, 0.30, 0.65),
    "monkey": (0.20, 0.15, 0.25),
}

# Global depth-to-size settings
SCALE_K   = 0.80
SIZE_EXP  = 1.20

# Per-class scale multipliers (post pinhole)
CLASS_SCALE = {
    "duck":   1.15,
    "bear":   1.20,
    "frog":   1.00,
    "man":    1.00,
    "monkey": 0.85,
}

# Per-class vertical center offsets (fraction of **box** height)
CLASS_CENTER_Y_OFFSET_FRAC = {
    "frog":  +1.3,
    "man":   -0.30,
    "bear":  -0.20,
    "monkey": -0.10,
    "duck": -0.05
}

# Per-class horizontal center offsets (fraction of **box** width)
CLASS_CENTER_X_OFFSET_FRAC = {
    "bear":  +0.12,
    "duck":  +0.11,
}

# Clamp limits for the box size as a fraction of the image
MIN_FRAC_W = 0.01
MIN_FRAC_H = 0.015
MAX_FRAC_W = 0.90
MAX_FRAC_H = 0.90

DATA_ROOT = "dataset"
IMG_DIR   = os.path.join(DATA_ROOT, "images")
LBL_DIR   = os.path.join(DATA_ROOT, "labels")
PREV_DIR  = os.path.join(DATA_ROOT, "previews")

def ensure_dirs():
    os.makedirs(IMG_DIR, exist_ok=True)
    os.makedirs(LBL_DIR, exist_ok=True)
    os.makedirs(PREV_DIR, exist_ok=True)

HFOV_DEG = 60.0
VFOV_DEG = None
CAM_CENTER_X_FRAC = 0.50
CAM_CENTER_Y_FRAC = 0.50

def synthetic_intrinsics(W: int, H: int):
    hfov = math.radians(HFOV_DEG)
    if VFOV_DEG is None:
        vfov = 2.0 * math.atan((H / W) * math.tan(hfov / 2.0))
    else:
        vfov = math.radians(VFOV_DEG)
    fx = (W / 2.0) / math.tan(hfov / 2.0)
    fy = (H / 2.0) / math.tan(vfov / 2.0)
    cx = CAM_CENTER_X_FRAC * W
    cy = CAM_CENTER_Y_FRAC * H
    return fx, fy, cx, cy

def next_index_for_prefix(prefix: str, directory: str, ext: str = ".jpg") -> int:
    """Look for files like f\"{prefix}<six digits>{ext}\" and return next index."""
    ensure_dirs()
    pat = re.compile(rf"^{re.escape(prefix)}(\d{{6}}){re.escape(ext)}$")
    max_idx = -1
    for fname in os.listdir(directory):
        m = pat.match(fname)
        if m:
            idx = int(m.group(1))
            max_idx = max(max_idx, idx)
    return max_idx + 1

class OneShotCapturer(Node):
    def __init__(self,
                 img_topic="/camera/image",
                 world_frame="map",
                 gz_set_pose_srv="/world/large_demo/set_pose",
                 rl_set_pose_srv="/set_pose",
                 model_name="husky",
                 entity_type=1,
                 settle_sec=1.5,
                 only_class: Optional[str] = None):
        """
        only_class:
          - if set to one of CLASS_NAMES, the run will only generate data for that class
            AND the filename prefix will be f\"{only_class}_\"
          - if None, it will run for all classes with a timestamp prefix to avoid collisions
        """
        super().__init__("one_shot_capturer_synth_camera")
        ensure_dirs()

        self.img_topic   = img_topic
        self.world_frame = world_frame
        self.gz_srv_name = gz_set_pose_srv
        self.rl_srv_name = rl_set_pose_srv
        self.model_name  = model_name
        self.entity_type = int(entity_type)
        self.settle_sec  = float(settle_sec)
        self.map_bounds = (-12.75, 12.75, -12.75, 12.75)

        # NEW: class selection + prefix
        if only_class is not None and only_class not in CLASS_NAMES:
            raise ValueError(f"only_class must be one of {CLASS_NAMES}, got {only_class}")
        self.only_class = only_class
        self.prefix = f"{only_class}_" if only_class else datetime.now().strftime("%Y%m%d_%H%M%S_")
        self.base_idx = next_index_for_prefix(self.prefix, IMG_DIR, ".jpg")

        self.bridge = CvBridge()

        # services
        self.gz_cli = self.create_client(GZSetEntityPose, self.gz_srv_name)
        self.rl_cli = self.create_client(RLSetPose, self.rl_srv_name)
        self.get_logger().info("Waiting for services…")
        self.gz_cli.wait_for_service()
        self.rl_cli.wait_for_service()
        self.get_logger().info("Services ready.")

        qos_img = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             history=QoSHistoryPolicy.KEEP_LAST, depth=10)
        self.img_qos = qos_img

    def inside_bounds(self, x: float, y: float) -> bool:
        xmin, xmax, ymin, ymax = self.map_bounds
        return (xmin <= x <= xmax) and (ymin <= y <= ymax)

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

    # --- math-only center/size ---
    def center_box_pixels(self, W: int, H: int, cls_name: str,
                          base_xyz: Tuple[float,float,float],
                          obj_xyz: Tuple[float,float,float]) -> Tuple[int,int,int,int]:
        fx, fy, cx, cy = synthetic_intrinsics(W, H)
        hx, hy, hz = BBOX_HALFSIZE[cls_name]
        width_phys  = 2.0 * max(hx, hy) * SCALE_K
        height_phys = 2.0 * hz * SCALE_K
        bx, by, bz = base_xyz
        ox, oy, oz = obj_xyz
        Z = max(((ox - bx)**2 + (oy - by)**2 + (oz - bz)**2) ** 0.5, 1e-3)
        cscale = CLASS_SCALE.get(cls_name, 1.0)
        w_pix = fx * (width_phys  / (Z ** SIZE_EXP)) * cscale
        h_pix = fy * (height_phys / (Z ** SIZE_EXP)) * cscale
        w_min = MIN_FRAC_W * W
        h_min = MIN_FRAC_H * H
        w_max = MAX_FRAC_W * W
        h_max = MAX_FRAC_H * H
        w_pix = max(w_min, min(w_pix, w_max))
        h_pix = max(h_min, min(h_pix, h_max))
        off_x = CLASS_CENTER_X_OFFSET_FRAC.get(cls_name, 0.0)
        off_y = CLASS_CENTER_Y_OFFSET_FRAC.get(cls_name, 0.0)
        u = cx + off_x * w_pix
        v = cy + off_y * h_pix
        x1 = int(round(max(0, u - 0.5 * w_pix)))
        y1 = int(round(max(0, v - 0.5 * h_pix)))
        x2 = int(round(min(W - 1, u + 0.5 * w_pix)))
        y2 = int(round(min(H - 1, v + 0.5 * h_pix)))
        return x1, y1, x2, y2

    def project_and_save_center(self,
                                img_msg,
                                idx: int,
                                base_pose: Tuple[float,float,float,float],
                                target: Tuple[str,float,float,float,float]):
        cv_img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
        H, W = cv_img.shape[:2]

        cls_name, ox, oy, oz, Rref = target
        bx, by, bz, byaw = base_pose

        x1, y1, x2, y2 = self.center_box_pixels(
            W, H, cls_name,
            base_xyz=(bx, by, bz),
            obj_xyz=(ox, oy, oz)
        )

        cxn = ((x1 + x2) * 0.5) / W
        cyn = ((y1 + y2) * 0.5) / H
        nwn = max(1e-6, (x2 - x1) / W)
        nhn = max(1e-6, (y2 - y1) / H)
        cxn = min(max(cxn, 0.0), 1.0)
        cyn = min(max(cyn, 0.0), 1.0)
        nwn = min(max(nwn, 0.0), 1.0)
        nhn = min(max(nhn, 0.0), 1.0)

        labels = [(CLASS_ID[cls_name], cxn, cyn, nwn, nhn)]

        # save image + label + preview with prefix based on class
        fname = f"{self.prefix}{idx:06d}.jpg"
        img_path = os.path.join(IMG_DIR, fname)
        lbl_path = os.path.join(LBL_DIR, fname.replace(".jpg", ".txt"))
        prv_path = os.path.join(PREV_DIR, fname)

        cv2.imwrite(img_path, cv_img)
        with open(lbl_path, "w") as f:
            for (cid, cxn, cyn, nwn, nhn) in labels:
                f.write(f"{cid} {cxn:.6f} {cyn:.6f} {nwn:.6f} {nhn:.6f}\n")

        overlay = cv_img.copy()
        cv2.rectangle(overlay, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(overlay, cls_name, (x1, max(0, y1 - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)
        cv2.imwrite(prv_path, overlay)

        return 1

    def run_once(self, waypoints: List[Tuple[float,float,float,float,float,str,float,float,float]]):
        idx = self.base_idx
        for (x, y, z, yaw, Rref, cls_name, ox, oy, oz) in waypoints:
            self.get_logger().info(
                f"[{idx}] teleport x={x:.2f} y={y:.2f} z={z:.2f} yaw(deg)={math.degrees(yaw):.1f} "
                f"R={Rref:.2f} class={cls_name}"
            )
            if not self.inside_bounds(x, y):
                self.get_logger().warn(
                    f"[{idx}] Skipping: waypoint ({x:.2f},{y:.2f}) is outside map bounds"
                )
                idx += 1
                continue
            ok1 = self.teleport_gazebo(x, y, z, yaw)
            ok2 = self.set_tf_pose(x, y, z, yaw)
            if not (ok1 and ok2):
                self.get_logger().warn("Teleport or SetPose failed; continuing anyway.")

            t_end = self.get_clock().now() + Duration(seconds=self.settle_sec)
            while rclpy.ok() and self.get_clock().now() < t_end:
                rclpy.spin_once(self, timeout_sec=0.05)

            img_msg = self.grab_one_image(timeout_sec=3.0)
            if img_msg is None:
                self.get_logger().warn("No image received; skipping waypoint.")
                idx += 1
                continue

            n = self.project_and_save_center(
                img_msg, idx,
                (x, y, z, yaw),
                (cls_name, ox, oy, oz, Rref)
            )
            self.get_logger().info(f"[{idx}] boxes={n}")
            idx += 1


def build_waypoints(only_class: Optional[str] = None):
    # Waypoints: (x, y, z, yaw, R, cls_name, ox, oy, oz)
    waypoints: List[Tuple[float,float,float,float,float,str,float,float,float]] = []

    targets = [
        ("man",    7.0,  4.0, 0.0),
        ("duck",  -10.0,  9.0, 0.0),
        ("frog",   6.0,  -4.0, 0.0),
        ("bear",   -7.0, -6.0, 1.25),
        ("monkey", 2.0,  -7.0, 0.0),
    ]
    if only_class:
        targets = [t for t in targets if t[0] == only_class]

    radii = [2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0]
    angles_deg = [0, 15, 30, 45, 60, 75, 90, 105, 120, 135, 150, 165, 180, 195, 210, 225, 240, 255, 270, 285, 300, 315, 330]

    for (cls_name, ox, oy, oz) in targets:
        radii_for_target = [1.5, 2.0, 2.5] if cls_name == "frog" else [3.0, 4.0, 5.0, 6.0, 7.0, 8.0] if cls_name == "bear" else radii
        for R in radii_for_target:
            for ang in angles_deg:
                a = math.radians(ang)
                rx = ox + R * math.cos(a)
                ry = oy + R * math.sin(a)
                rz = 0.0
                yaw = math.atan2(oy - ry, ox - rx)  # face target center
                waypoints.append((rx, ry, rz, yaw, R, cls_name, ox, oy, oz))
    return waypoints


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--img_topic", default="/camera/image")
    parser.add_argument("--world_frame", default="map")
    parser.add_argument("--gz_set_pose_srv", default="/world/large_demo/set_pose")
    parser.add_argument("--rl_set_pose_srv", default="/set_pose")
    parser.add_argument("--model_name", default="husky")
    parser.add_argument("--entity_type", type=int, default=1)
    parser.add_argument("--settle_sec", type=float, default=1.8)
    # optional CLI passthrough for convenience (still a class param)
    parser.add_argument("--class_name", choices=CLASS_NAMES, default=None,
                        help="If set, run only this class and use its name as the filename prefix.")
    args = parser.parse_args()

    rclpy.init()

    node = OneShotCapturer(
        img_topic=args.img_topic,
        world_frame=args.world_frame,
        gz_set_pose_srv=args.gz_set_pose_srv,
        rl_set_pose_srv=args.rl_set_pose_srv,
        model_name=args.model_name,
        entity_type=args.entity_type,
        settle_sec=args.settle_sec,
        only_class=args.class_name,   # <-- class param
    )

    waypoints = build_waypoints(only_class=args.class_name)

    try:
        node.run_once(waypoints)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
