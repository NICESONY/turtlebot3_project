#!/usr/bin/env python3
import os
import math
import time
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist

class ArucoNavRobot(Node):
    def __init__(self):
        super().__init__('aruco_nav_robot')
        self.bridge = CvBridge()
        # 1) 카메라 압축 이미지 구독
        self.create_subscription(
            CompressedImage,
            '/camera/image_raw/compressed',
            self.cb_image,
            10)
        # 2) 주석 이미지 퍼블리시 (optional)
        self.pub_img = self.create_publisher(
            CompressedImage,
            '/camera/aruco_annotated/compressed',
            10)
        # 3) cmd_vel 퍼블리시
        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10)

        # 카메라 캘리브레이션 로드
        pkg_dir = os.path.dirname(__file__)
        calib_file = os.path.join(pkg_dir, 'camera_calibration.npz')
        if not os.path.exists(calib_file):
            self.get_logger().error(f"Calibration file not found: {calib_file}")
            raise FileNotFoundError(calib_file)
        data = np.load(calib_file)
        self.camera_matrix = data['camera_matrix']
        self.dist_coeffs = data['distortion_coefficients']

        # ArUco 세팅
        self.aruco_dict   = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters_create()
        self.marker_length = 0.02   # m
        # 조정 파라미터
        self.center_thresh = 0.1    # 화면 폭의 10%
        self.angle_thresh  = 5.0    # degrees
        self.lin_speed     = 0.08   # m/s
        self.ang_speed     = 0.4    # rad/s

    def cb_image(self, msg: CompressedImage):
        # decode
        arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if frame is None:
            return

        out_frame = self.annotate_and_control(frame)

        # publish annotated image
        ret, buf = cv2.imencode('.jpg', out_frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        if ret:
            out = CompressedImage()
            out.header = msg.header
            out.format = 'jpeg'
            out.data = buf.tobytes()
            self.pub_img.publish(out)

    def annotate_and_control(self, frame: np.ndarray) -> np.ndarray:
        h, w = frame.shape[:2]
        img_cx = w / 2

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(
            gray, self.aruco_dict, parameters=self.aruco_params)

        twist = Twist()  # 기본 정지

        if ids is not None and 1 in ids.flatten():
            # ID 1 인덱스
            idx = int(np.where(ids.flatten() == 1)[0][0])
            pts = corners[idx].reshape(4,2).astype(np.float32)
            # 1) marker center
            cx = float(np.mean(pts[:,0]))
            # 2) orientation angle (top-left→top-right)
            dx = pts[1,0] - pts[0,0]
            dy = pts[1,1] - pts[0,1]
            angle_deg = math.degrees(math.atan2(dy, dx))

            # draw for debug
            cv2.aruco.drawDetectedMarkers(frame, [pts], [1])
            cv2.putText(frame,
                        f"Cx:{cx:.0f}  Ang:{angle_deg:.1f}",
                        (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)

            # 제어 로직
            # 1) orientation correction
            if abs(angle_deg) > self.angle_thresh:
                twist.linear.x  = self.lin_speed
                # angle_deg >0 → marker top-edge slopes down→rotate left(+z)
                twist.angular.z =  self.ang_speed if angle_deg>0 else -self.ang_speed

            # 2) centering correction
            elif abs(cx - img_cx) > self.center_thresh * w:
                twist.linear.x  = self.lin_speed
                # cx < center → marker left → rotate left
                twist.angular.z =  self.ang_speed if cx < img_cx else -self.ang_speed

            # 3) aligned → 정면 전진
            else:
                twist.linear.x  = 0.12
                twist.angular.z = 0.0

        else:
            # ID 1 미검출 → 제자리 회전하며 탐색
            twist.linear.x  = 0.0
            twist.angular.z = 0.5

        # 퍼블리시
        self.cmd_pub.publish(twist)
        return frame

def main(args=None):
    rclpy.init(args=args)
    node = ArucoNavRobot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__=='__main__':
    main()
