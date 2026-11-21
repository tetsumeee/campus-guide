#!/usr/bin/env python3
# Quick AprilTag ID check (no calibration needed) - prints which tag IDs the camera sees.
import time, cv2, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

DICT_ID = cv2.aruco.DICT_APRILTAG_36h11
try:   # OpenCV >= 4.7
    _det = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(DICT_ID),
                                   cv2.aruco.DetectorParameters())
    detect = lambda g: _det.detectMarkers(g)
except AttributeError:   # older OpenCV (Ubuntu 22.04)
    _dict = cv2.aruco.Dictionary_get(DICT_ID)
    _par = cv2.aruco.DetectorParameters_create()
    detect = lambda g: cv2.aruco.detectMarkers(g, _dict, parameters=_par)

class TagTest(Node):
    def __init__(self):
        super().__init__('tag_quicktest')
        self.br, self.last = CvBridge(), 0.0
        self.create_subscription(Image, '/image_raw', self.cb, qos_profile_sensor_data)
        self.get_logger().info('Hold a tag in front of the camera...')

    def cb(self, msg):
        if time.time() - self.last < 0.5:
            return
        self.last = time.time()
        gray = self.br.imgmsg_to_cv2(msg, 'mono8')
        _, ids, _ = detect(gray)
        if ids is None:
            self.get_logger().info('No tag')
        else:
            self.get_logger().info(f'Seen tag IDs: {sorted(ids.flatten().tolist())}')

rclpy.init(); rclpy.spin(TagTest())
