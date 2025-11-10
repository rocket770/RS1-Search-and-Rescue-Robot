#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from typing import Optional

class NightVisionNode(Node):
    # _process() and _vignette_mask() are from chatgpt!!!
    def __init__(self):
        super().__init__('night_vision_node')
        self.declare_parameter('in_image', '/model/husky/camera/image')
        self.declare_parameter('out_image', '/night_vision/image')

        # Hyper paratemers for vignette_mask(), used to adjust how strong the fitler is.
        self.declare_parameter('noise_stddev', 8.0)
        self.declare_parameter('gain', 2.0)
        self.declare_parameter('clahe_clip', 3.0)
        self.declare_parameter('vignette_strength', 0.2)
        self.declare_parameter('publish_hz', 1.0)  

        self.in_topic  = self.get_parameter('in_image').value
        self.out_topic = self.get_parameter('out_image').value
        self.noise_std = float(self.get_parameter('noise_stddev').value)
        self.gain      = float(self.get_parameter('gain').value)
        self.clip      = float(self.get_parameter('clahe_clip').value)
        self.vign      = float(self.get_parameter('vignette_strength').value)
        self.pub_hz    = float(self.get_parameter('publish_hz').value)

        self.bridge = CvBridge()
        self.pub = self.create_publisher(Image, self.out_topic, 10)
        self.sub = self.create_subscription(Image, self.in_topic, self._cb_store_latest, 10)
        self.get_logger().info(f"Night vision on {self.in_topic} -> {self.out_topic} (publish_hz={self.pub_hz})")

        self.clahe = cv2.createCLAHE(clipLimit=self.clip, tileGridSize=(8,8))
        self.vignette_cache = {}

        # storage for latest incoming frame
        self._latest_msg: Optional[Image] = None

        period = 1.0 / self.pub_hz
        self.timer = self.create_timer(period, self._timer_publish)

    # builds and cache a radial vignette mask by darkneuing pixels away from the center limited to a specific brigthness
    def _vignette_mask(self, h, w):
        key = (h, w)
        if key in self.vignette_cache:
            return self.vignette_cache[key]
        y, x = np.indices((h, w))
        cx, cy = w/2.0, h/2.0
        r = np.sqrt((x - cx)**2 + (y - cy)**2)
        r /= r.max() + 1e-6
        mask = 1.0 - self.vign * (r**2)
        mask = np.clip(mask, 0.5, 1.0).astype(np.float32)
        self.vignette_cache[key] = mask
        return mask

    # grab the ros image and use open cv to enhance contrast and add some noiss
    # also reduce blue and red chanels to make the image green
    # than add some blur to make it look cool and convert to ros image and publish
    def _process(self, msg: Image) -> Image:
        cv = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        gray = cv2.cvtColor(cv, cv2.COLOR_BGR2GRAY)
        gray = cv2.convertScaleAbs(gray, alpha=self.gain, beta=0)
        gray = self.clahe.apply(gray)

        noise = np.random.normal(0, self.noise_std, gray.shape).astype(np.float32)
        noisy = np.clip(gray.astype(np.float32) + noise, 0, 255).astype(np.uint8)

        g = noisy
        b = (noisy * 0.15).astype(np.uint8)
        r = (noisy * 0.05).astype(np.uint8)
        nvg = cv2.merge([b, g, r])

        blur = cv2.GaussianBlur(nvg, (0,0), 1.5)
        nvg = cv2.addWeighted(nvg, 1.0, blur, 0.35, 0)

        h, w = nvg.shape[:2]
        mask = self._vignette_mask(h, w)
        nvg = (nvg.astype(np.float32) * mask[...,None]).clip(0,255).astype(np.uint8)

        out = self.bridge.cv2_to_imgmsg(nvg, encoding='bgr8')
        out.header = msg.header
        return out

    def _cb_store_latest(self, msg: Image):
        self._latest_msg = msg

    def _cb_process_and_publish(self, msg: Image):
        try:
            out = self._process(msg)
            self.pub.publish(out)
        except Exception as e:
            self.get_logger().warn(f"processing error: {e}")

    def _timer_publish(self):
        if self._latest_msg is None:
            return
        try:
            out = self._process(self._latest_msg)
            self.pub.publish(out)
        except Exception as e:
            self.get_logger().warn(f"processing error: {e}")

def main():
    rclpy.init()
    rclpy.spin(NightVisionNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
