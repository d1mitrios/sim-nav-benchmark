#!/usr/bin/env python3
# Decompresses /camera_jpeg (CompressedImage) -> raw /camera (Image rgb8 640x480)
# locally inside WSL, so yolo_navigator.py stays 100% unchanged.
import rclpy, cv2
import numpy as np
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CompressedImage

class Relay(Node):
    def __init__(self):
        super().__init__('camera_relay')
        self.pub = self.create_publisher(Image, '/camera', qos_profile_sensor_data)
        self.sub = self.create_subscription(CompressedImage, '/camera_jpeg', self.cb, qos_profile_sensor_data)
        self.n = 0

    def cb(self, m):
        bgr = cv2.imdecode(np.frombuffer(m.data, np.uint8), cv2.IMREAD_COLOR)
        if bgr is None:
            return
        rgb = bgr[:, :, ::-1]
        out = Image()
        out.header = m.header
        out.height, out.width = rgb.shape[0], rgb.shape[1]
        out.encoding = 'rgb8'
        out.is_bigendian = 0
        out.step = 3 * rgb.shape[1]
        out.data = np.ascontiguousarray(rgb).tobytes()
        self.pub.publish(out)
        self.n += 1
        if self.n % 25 == 0:
            print(f'relayed {self.n} frames')

rclpy.init()
rclpy.spin(Relay())
