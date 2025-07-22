#!/usr/bin/env python3
import os
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class ArucoDetectTurtle(Node):
    def __init__(self):
        super().__init__('aruco_detect_turtle')
        # CvBridge: ROS Image ↔ OpenCV 변환
        self.bridge = CvBridge()
        # RAW 이미지 토픽 구독
        self.sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.cb_image,
            10
        )

        # 카메라 캘리브레이션 데이터 로드
        pkg_dir = os.path.dirname(__file__)
        calib_path = os.path.join(pkg_dir, 'camera_calibration.npz')
        if not os.path.exists(calib_path):
            self.get_logger().error(f"Calibration file not found: {calib_path}")
            raise FileNotFoundError(calib_path)
        data = np.load(calib_path)
        self.camera_matrix = data['camera_matrix']
        self.dist_coeffs   = data['distortion_coefficients']

        # 사용할 ArUco 사전 및 마커 크기 (m)
        self.aruco_dict_type = cv2.aruco.DICT_4X4_50
        self.marker_length = 0.02

    def cb_image(self, msg: Image):
        # ROS Image → OpenCV BGR
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        # ArUco 검출 + 주석
        annotated = self.pose_estimation(frame)
        # 결과 화면 표시
        cv2.imshow('Aruco Detection', annotated)
        cv2.waitKey(1)

    def pose_estimation(self, frame: np.ndarray) -> np.ndarray:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # 사전 & 파라미터 생성 (구 API)
        aruco_dict = cv2.aruco.getPredefinedDictionary(self.aruco_dict_type)
        params     = cv2.aruco.DetectorParameters_create()
        # 마커 검출
        corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=params)

        if ids is not None:
            # 마커 외곽선 그리기
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            for corner, marker_id in zip(corners, ids.flatten()):
                # 2D 이미지 포인트: 4개 코너
                img_pts = corner.reshape(-1, 2).astype(np.float32)
                # 3D 객체 포인트: 마커 정사각형 4개 꼭짓점
                half = self.marker_length / 2
                obj_pts = np.array([
                    [-half,  half, 0],
                    [ half,  half, 0],
                    [ half, -half, 0],
                    [-half, -half, 0],
                ], dtype=np.float32)

                # solvePnP 로 포즈 계산
                success, rvec, tvec = cv2.solvePnP(
                    obj_pts,
                    img_pts,
                    self.camera_matrix,
                    self.dist_coeffs,
                )
                if not success:
                    continue

                # 좌표축 그리기
                cv2.aruco.drawAxis(
                    frame,
                    self.camera_matrix,
                    self.dist_coeffs,
                    rvec,
                    tvec,
                    self.marker_length
                )

                # 거리(cm) 텍스트
                x, y, z = (tvec.flatten() * 100)
                text = f"ID:{int(marker_id)} X:{x:.1f}cm Y:{y:.1f}cm Z:{z:.1f}cm"
                # 텍스트 위치: 첫 번째 코너 바로 위
                pt = tuple(img_pts[0].astype(int))
                cv2.putText(
                    frame,
                    text,
                    (pt[0], pt[1] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    2
                )

        return frame

def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectTurtle()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    # 종료 시 정리
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
