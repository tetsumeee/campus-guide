#!/usr/bin/env python3
# Headless camera calibration v2: captures only when the board is still,
# then removes the worst views and recalibrates.
import os, time
import numpy as np, cv2, yaml, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

COLS, ROWS = 8, 6      # inner corners
SQUARE = 0.025         # square size in meters
TARGET = 30            # views to collect
INTERVAL = 1.5         # min seconds between captures
STILL_PX = 1.5         # board must move less than this (pixels) between frames
DROP_FRAC = 0.2        # drop the worst 20% of views before final calibration
CAM_NAME = 'mmal_service_16.1'
OUT = os.path.expanduser(f'~/.ros/camera_info/{CAM_NAME}.yaml')


class Calibrator(Node):
    def __init__(self):
        super().__init__('headless_calibrator')
        self.bridge = CvBridge()
        self.objp = np.zeros((COLS * ROWS, 3), np.float32)
        self.objp[:, :2] = np.mgrid[0:COLS, 0:ROWS].T.reshape(-1, 2) * SQUARE
        self.obj_pts, self.img_pts = [], []
        self.prev = None
        self.last, self.last_msg, self.size, self.done = 0.0, 0.0, None, False
        self.create_subscription(Image, '/image_raw', self.cb, qos_profile_sensor_data)
        self.get_logger().info(f'Hold the board STILL at each pose. Collecting {TARGET} views...')

    def cb(self, msg):
        if self.done:
            return
        now = time.time()
        gray = self.bridge.imgmsg_to_cv2(msg, 'mono8')
        self.size = (gray.shape[1], gray.shape[0])
        flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE + cv2.CALIB_CB_FAST_CHECK
        ok, corners = cv2.findChessboardCorners(gray, (COLS, ROWS), flags)
        if not ok:
            self.prev = None
            if now - self.last_msg > 3:
                self.get_logger().info('Board not found - move it into view')
                self.last_msg = now
            return
        # Stillness check: compare with previous frame
        still = self.prev is not None and np.mean(np.linalg.norm(corners - self.prev, axis=2)) < STILL_PX
        self.prev = corners
        if not still or now - self.last < INTERVAL:
            return
        corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1),
                                   (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
        self.obj_pts.append(self.objp)
        self.img_pts.append(corners)
        self.last = now
        self.get_logger().info(f'Captured {len(self.img_pts)}/{TARGET} - move to a NEW pose, then hold still')
        if len(self.img_pts) >= TARGET:
            self.calibrate()

    def per_view_errors(self, K, D, rvecs, tvecs, obj, img):
        errs = []
        for o, i, r, t in zip(obj, img, rvecs, tvecs):
            proj, _ = cv2.projectPoints(o, r, t, K, D)
            errs.append(np.sqrt(np.mean(np.sum((proj - i) ** 2, axis=2))))
        return np.array(errs)

    def calibrate(self):
        self.done = True
        self.get_logger().info('Calibrating (pass 1)...')
        rms, K, D, rv, tv = cv2.calibrateCamera(self.obj_pts, self.img_pts, self.size, None, None)
        self.get_logger().info(f'Pass 1 error = {rms:.3f} px')
        errs = self.per_view_errors(K, D, rv, tv, self.obj_pts, self.img_pts)
        keep = np.argsort(errs)[:max(10, int(len(errs) * (1 - DROP_FRAC)))]
        obj = [self.obj_pts[i] for i in keep]
        img = [self.img_pts[i] for i in keep]
        self.get_logger().info(f'Dropped {len(errs) - len(keep)} worst views, calibrating (pass 2)...')
        rms, K, D, _, _ = cv2.calibrateCamera(obj, img, self.size, None, None)
        w, h = self.size
        P = np.zeros((3, 4))
        P[:3, :3] = K
        data = {
            'image_width': w,
            'image_height': h,
            'camera_name': CAM_NAME,
            'camera_matrix': {'rows': 3, 'cols': 3, 'data': K.flatten().tolist()},
            'distortion_model': 'plumb_bob',
            'distortion_coefficients': {'rows': 1, 'cols': 5, 'data': D.flatten()[:5].tolist()},
            'rectification_matrix': {'rows': 3, 'cols': 3, 'data': np.eye(3).flatten().tolist()},
            'projection_matrix': {'rows': 3, 'cols': 4, 'data': P.flatten().tolist()},
        }
        os.makedirs(os.path.dirname(OUT), exist_ok=True)
        with open(OUT, 'w') as f:
            yaml.safe_dump(data, f, default_flow_style=None, sort_keys=False)
        quality = 'GOOD' if rms < 0.5 else ('OK' if rms < 1.0 else 'POOR - redo it')
        self.get_logger().info(f'Done! Reprojection error = {rms:.3f} px ({quality})')
        self.get_logger().info(f'Saved to {OUT}')


def main():
    rclpy.init()
    node = Calibrator()
    while rclpy.ok() and not node.done:
        rclpy.spin_once(node, timeout_sec=0.5)
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
