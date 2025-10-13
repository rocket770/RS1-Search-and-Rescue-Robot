#!/usr/bin/env python3

#requires the ign bridge to be running

import os
import math
import numpy as np
import cv2
from typing import List, Tuple

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
    "duck":   (0.11, 0.13, 0.16),
    "bear":   (0.70, 0.55, 0.75),
    "frog":   (0.10, 0.08, 0.05),
    "man":    (0.35, 0.30, 0.65),
    "monkey": (0.20, 0.15, 0.25),
}

# Global depth-to-size settings
SCALE_K   = 0.80   # global multiplier on physical sizes
SIZE_EXP  = 1.20   # exponent on distance (1.0 = true pinhole; >1 shrinks faster)

# Per-class scale multipliers (post pinhole)
CLASS_SCALE = {
    "duck":   1.15,  # slightly bigger
    "bear":   1.20,  # make bigger overall
    "frog":   1.00,
    "man":    1.00,
    "monkey": 0.85,  # slightly bigger
}

# Per-class vertical center offsets (fraction of **box** height).
# Negative = move UP; Positive = move DOWN.
CLASS_CENTER_Y_OFFSET_FRAC = {
    "frog":  +1.3,   # move frog down a bit
    "man":   -0.30,   # move humans further up
    #"duck":   -0.15,   # move duck down
    "bear":   -0.20,
    "monkey": -0.10
    # others default 0
}

# Per-class horizontal center offsets (fraction of **box** width).
# Negative = left; Positive = right.
CLASS_CENTER_X_OFFSET_FRAC = {
    "bear":  +0.12,   # nudge bear to the right
    "duck":  +0.1,   # nudge duck to the left

    # others default 0
}

# Clamp limits for the box size as a fraction of the image
MIN_FRAC_W = 0.01   # gentler mins than before (preserve scaling curve)
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


HFOV_DEG = 60.0          # horizontal FOV of the virtual camera (tweak as needed)
VFOV_DEG = None          # None => derive from aspect ratio; or set e.g. 45.0
CAM_CENTER_X_FRAC = 0.50 # principal point as fraction of image width
CAM_CENTER_Y_FRAC = 0.50 # principal point as fraction of image height

def synthetic_intrinsics(W: int, H: int):
    """Build virtual intrinsics (fx, fy, cx, cy) from FOV + image size."""
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


class OneShotCapturer(Node):

    def __init__(self,
                 img_topic="/camera/image",
                 world_frame="map",
                 gz_set_pose_srv="/world/large_demo/set_pose",
                 rl_set_pose_srv="/set_pose",
                 model_name="husky",
                 entity_type=1,
                 settle_sec=1.5):
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

        self.bridge = CvBridge()

        # services
        self.gz_cli = self.create_client(GZSetEntityPose, self.gz_srv_name)
        self.rl_cli = self.create_client(RLSetPose, self.rl_srv_name)
        self.get_logger().info("Waiting for services…")
        self.gz_cli.wait_for_service()
        self.rl_cli.wait_for_service()
        self.get_logger().info("Services ready.")

        # QoS
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

        # physical extents
        hx, hy, hz = BBOX_HALFSIZE[cls_name]
        width_phys  = 2.0 * max(hx, hy) * SCALE_K
        height_phys = 2.0 * hz * SCALE_K

        # distance robot<->object center in 3D
        bx, by, bz = base_xyz
        ox, oy, oz = obj_xyz
        Z = max(((ox - bx)**2 + (oy - by)**2 + (oz - bz)**2) ** 0.5, 1e-3)

        # pinhole scaling
        cscale = CLASS_SCALE.get(cls_name, 1.0)
        w_pix = fx * (width_phys  / (Z ** SIZE_EXP)) * cscale
        h_pix = fy * (height_phys / (Z ** SIZE_EXP)) * cscale

        # gentle clamps so 1/Z remains visible
        w_min = MIN_FRAC_W * W
        h_min = MIN_FRAC_H * H
        w_max = MAX_FRAC_W * W
        h_max = MAX_FRAC_H * H
        w_pix = max(w_min, min(w_pix, w_max))
        h_pix = max(h_min, min(h_pix, h_max))

        # center at synthetic principal point, then apply offsets RELATIVE TO BOX
        off_x = CLASS_CENTER_X_OFFSET_FRAC.get(cls_name, 0.0)
        off_y = CLASS_CENTER_Y_OFFSET_FRAC.get(cls_name, 0.0)
        u = cx + off_x * w_pix
        v = cy + off_y * h_pix

        # corners (clip to image)
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

        # compute math-only, distance-scaled box
        x1, y1, x2, y2 = self.center_box_pixels(
            W, H, cls_name,
            base_xyz=(bx, by, bz),
            obj_xyz=(ox, oy, oz)
        )

        # YOLO-normalized (cx, cy, w, h) clamped to [0,1]
        cxn = ((x1 + x2) * 0.5) / W
        cyn = ((y1 + y2) * 0.5) / H
        nwn = max(1e-6, (x2 - x1) / W)
        nhn = max(1e-6, (y2 - y1) / H)
        cxn = min(max(cxn, 0.0), 1.0)
        cyn = min(max(cyn, 0.0), 1.0)
        nwn = min(max(nwn, 0.0), 1.0)
        nhn = min(max(nhn, 0.0), 1.0)

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

    def run_once(self, waypoints: List[Tuple[float,float,float,float,float,str,float,float,float]]):
        idx = 0
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

            # settle
            t_end = self.get_clock().now() + Duration(seconds=self.settle_sec)
            while rclpy.ok() and self.get_clock().now() < t_end:
                rclpy.spin_once(self, timeout_sec=0.05)

            # image
            img_msg = self.grab_one_image(timeout_sec=3.0)
            if img_msg is None:
                self.get_logger().warn("No image received; skipping waypoint.")
                idx += 1
                continue

            # centered, distance-scaled (with per-class offsets) box
            n = self.project_and_save_center(
                img_msg, idx,
                (x, y, z, yaw),
                (cls_name, ox, oy, oz, Rref)
            )
            self.get_logger().info(f"[{idx}] boxes={n}")
            idx += 1



def main():
    rclpy.init()

    # Waypoints: (x, y, z, yaw, R, cls_name, ox, oy, oz)
    waypoints: List[Tuple[float,float,float,float,float,str,float,float,float]] = []

    targets = [
        ("man",    9.0,  4.0, 0.0),
        ("duck",  -10.0,  9.0, 0.0),
        ("frog",   6.0,  -4.0, 0.0),
        ("bear",   -7.0, -6.0, 1.25),
        ("monkey", 2.0,  -7.0, 0.0),
    ]
    radii = [2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0]
    angles_deg = [0, 15, 30, 45, 60, 75, 90, 105, 120, 135, 150, 165, 180, 195, 210, 225, 240, 255, 270, 285, 300, 315, 330]

    #radii = [2.0, 5.0, 8.0, 10.0]
    #angles_deg = [0, 120,240, 330]

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

    node = OneShotCapturer(
        img_topic="/camera/image",
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
