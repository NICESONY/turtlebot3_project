#!/usr/bin/env python3
import os
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage

class ArucoDetectCompressed(Node):
    def __init__(self):
        super().__init__('aruco_detect_compressed')
        # CompressedImage 구독
        self.sub = self.create_subscription(
            CompressedImage,
            '/camera/image_raw/compressed',
            self.cb_image,
            10)
        # CompressedImage 퍼블리시
        self.pub = self.create_publisher(
            CompressedImage,
            '/camera/aruco_annotated/compressed',
            10)
        self.bridge = CvBridge()

        # 캘리브레이션 로드
        pkg_dir = os.path.dirname(__file__)
        calib_file = os.path.join(pkg_dir, 'camera_calibration.npz')
        if not os.path.exists(calib_file):
            self.get_logger().error(f"Calibration file not found: {calib_file}")
            raise FileNotFoundError(calib_file)
        data = np.load(calib_file)
        self.camera_matrix = data['camera_matrix']
        self.dist_coeffs = data['distortion_coefficients']

        # ArUco 사전 및 파라미터
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters_create()
        self.marker_length = 0.02  # 단위: m

    def cb_image(self, msg: CompressedImage):
        # 압축 해제
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is None:
            return

        annotated = self.annotate(frame)

        # 다시 압축하여 퍼블리시
        ret, buf = cv2.imencode('.jpg', annotated, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        if not ret:
            return
        out_msg = CompressedImage()
        out_msg.header = msg.header
        out_msg.format = 'jpeg'
        out_msg.data = np.array(buf).tobytes()
        self.pub.publish(out_msg)

    def annotate(self, frame: np.ndarray) -> np.ndarray:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # detectMarkers 사용 (구 버전 호환)
        corners, ids, _ = cv2.aruco.detectMarkers(
            gray,
            self.aruco_dict,
            parameters=self.aruco_params)
        if ids is not None and len(ids) > 0:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            # 자세 추정
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners,
                self.marker_length,
                self.camera_matrix,
                self.dist_coeffs)
            for rvec, tvec, marker_id, corner in zip(rvecs, tvecs, ids.flatten(), corners):
                # 좌표축 그리기
                cv2.aruco.drawAxis(
                    frame,
                    self.camera_matrix,
                    self.dist_coeffs,
                    rvec,
                    tvec,
                    self.marker_length)
                # 거리 정보
                x, y, z = tvec.flatten() * 100  # cm
                text = f"ID:{int(marker_id)} X:{x:.1f}cm Y:{y:.1f}cm Z:{z:.1f}cm"
                # 텍스트 위치: 마커 첫 코너 위
                pt = tuple(corner.reshape(-1, 2)[0].astype(int) - np.array([0,10]))
                cv2.putText(
                    frame,
                    text,
                    pt,
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0,255,0),
                    2)
        return frame


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectCompressed()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
