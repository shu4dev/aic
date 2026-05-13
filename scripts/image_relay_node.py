#!/usr/bin/env python3
"""
Image relay node: subscribes to raw full-res camera topics, resizes to training
resolution, JPEG-encodes, and republishes as sensor_msgs/CompressedImage.

This reduces MCAP bag size by ~160x (19GB → ~120MB per trial), making
data collection and conversion dramatically faster.

Run inside aic_eval container before starting aic_engine:
    python3 /tmp/image_relay_node.py

Published topics (sensor_msgs/CompressedImage, JPEG):
    /left_camera/image_small
    /center_camera/image_small
    /right_camera/image_small
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image, CompressedImage

# Must match mcap_to_lerobot.py training resolution (CAMERA_WIDTH/HEIGHT * image_scale)
TARGET_W = 288
TARGET_H = 256
JPEG_QUALITY = 85

CAMERAS = ["left_camera", "center_camera", "right_camera"]

# Use BEST_EFFORT to match typical camera publisher QoS
SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)


class ImageRelayNode(Node):
    def __init__(self):
        super().__init__("image_relay")
        self._pubs = {}
        self._subs = []
        for cam in CAMERAS:
            pub = self.create_publisher(
                CompressedImage, f"/{cam}/image_small", SENSOR_QOS
            )
            self._pubs[cam] = pub
            sub = self.create_subscription(
                Image,
                f"/{cam}/image",
                lambda msg, c=cam: self._callback(msg, c),
                SENSOR_QOS,
            )
            self._subs.append(sub)
        self.get_logger().info(
            f"Image relay ready: raw → image_small ({TARGET_W}×{TARGET_H} JPEG q{JPEG_QUALITY})"
        )

    def _callback(self, msg: Image, cam_name: str):
        enc = msg.encoding.lower()
        try:
            raw = np.frombuffer(bytes(msg.data), np.uint8).reshape(
                msg.height, msg.width, -1
            )
        except ValueError:
            return

        # Normalize to BGR
        if enc in ("rgb8",):
            bgr = cv2.cvtColor(raw, cv2.COLOR_RGB2BGR)
        elif enc in ("rgba8",):
            bgr = cv2.cvtColor(raw, cv2.COLOR_RGBA2BGR)
        elif enc in ("bgra8",):
            bgr = cv2.cvtColor(raw[:, :, :3], cv2.COLOR_BGRA2BGR)
        else:
            bgr = raw  # assume bgr8

        # Resize to training resolution
        if bgr.shape[1] != TARGET_W or bgr.shape[0] != TARGET_H:
            bgr = cv2.resize(bgr, (TARGET_W, TARGET_H), interpolation=cv2.INTER_AREA)

        # JPEG encode
        ok, buf = cv2.imencode(
            ".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY]
        )
        if not ok:
            return

        out = CompressedImage()
        out.header = msg.header
        out.format = "jpeg"
        out.data = buf.tobytes()
        self._pubs[cam_name].publish(out)


def main():
    rclpy.init()
    node = ImageRelayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
