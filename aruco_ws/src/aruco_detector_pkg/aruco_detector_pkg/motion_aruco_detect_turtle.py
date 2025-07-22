#!/usr/bin/env python3
import os
import time
import math
import threading
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist

class ArucoDetectRobot(Node):
    def __init__(self):
        super().__init__('fast_aruco_detect_turtle')  # 노드 이름 설정

        # CvBridge: ROS Image <-> OpenCV 이미지 변환 도구
        self.bridge = CvBridge()

        # 1) 카메라 압축 이미지 구독 설정
        self.create_subscription(
            CompressedImage,
            '/camera/image_raw/compressed',
            self.cb_image,
            10)

        # 2) 주석(아루코 마커 표시)된 압축 이미지 퍼블리시 설정
        self.pub_img = self.create_publisher(
            CompressedImage,
            '/camera/aruco_annotated/compressed',
            10)

        # 3) 로봇 이동 명령 퍼블리시 설정 (/cmd_vel 토픽)
        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10)

        # 카메라 캘리브레이션 데이터를 로드
        pkg_dir = os.path.dirname(__file__)
        calib_file = os.path.join(pkg_dir, 'camera_calibration.npz')
        if not os.path.exists(calib_file):
            self.get_logger().error(f"Calibration file not found: {calib_file}")
            raise FileNotFoundError(calib_file)
        data = np.load(calib_file)
        self.camera_matrix = data['camera_matrix']      # 내부 파라미터 행렬
        self.dist_coeffs = data['distortion_coefficients']  # 왜곡 계수

        # ArUco 마커 검출 사전 및 파라미터 초기화
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters_create()
        self.marker_length = 0.02  # 마커 실제 길이(m)

        # 이미 행동을 수행한 마커 ID 집합 (중복 실행 방지)
        self.processed_ids = set()

    def cb_image(self, msg: CompressedImage):
        # 압축된 ROS 메시지를 OpenCV 이미지로 변환
        arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if frame is None:
            return  # 디코딩 실패 시 리턴

        # 이미지에 아루코 마커 표시 및 행동 명령 호출
        annotated = self.annotate_and_control(frame)

        # 주석된 이미지를 다시 JPEG로 압축하여 퍼블리시
        ret, buf = cv2.imencode('.jpg', annotated, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        if not ret:
            return
        out_msg = CompressedImage()
        out_msg.header = msg.header
        out_msg.format = 'jpeg'
        out_msg.data = buf.tobytes()
        self.pub_img.publish(out_msg)

    def annotate_and_control(self, frame: np.ndarray) -> np.ndarray:
        # 그레이스케일로 변환하여 검출 성능 향상
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # 아루코 마커 검출
        corners, ids, _ = cv2.aruco.detectMarkers(
            gray,
            self.aruco_dict,
            parameters=self.aruco_params)
        if ids is not None and len(ids) > 0:
            # 마커 외곽선 그리기
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            # 각 마커에 대해 포즈(자세) 추정
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners,
                self.marker_length,
                self.camera_matrix,
                self.dist_coeffs)

            for rvec, tvec, marker_id, corner in zip(rvecs, tvecs, ids.flatten(), corners):
                # 좌표축 그리기: x:red, y:green, z:blue
                cv2.aruco.drawAxis(
                    frame,
                    self.camera_matrix,
                    self.dist_coeffs,
                    rvec,
                    tvec,
                    self.marker_length)

                # 마커 ID별 행동 수행 (한 번만 실행)
                if marker_id not in self.processed_ids:
                    self.processed_ids.add(marker_id)
                    threading.Thread(
                        target=self.process_marker,
                        args=(int(marker_id),),
                        daemon=True).start()

                # 마커까지의 거리(cm) 표시
                x, y, z = tvec.flatten() * 100
                text = f"ID:{marker_id} X:{x:.1f} Y:{y:.1f} Z:{z:.1f}"
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

    def process_marker(self, marker_id: int):
        """
        검출된 마커 ID에 따라 로봇 행동을 수행:
          1 → 왼쪽으로 한 바퀴 회전
          2 → 오른쪽으로 한 바퀴 회전
          3 → 후진
        """
        if marker_id == 1:
            self.get_logger().info("Marker 1: rotate left one revolution")
            self.rotate(speed=1.0, angle=2*math.pi)
        elif marker_id == 2:
            self.get_logger().info("Marker 2: rotate right one revolution")
            self.rotate(speed=-1.0, angle=2*math.pi)
        elif marker_id == 3:
            self.get_logger().info("Marker 3: move backward briefly")
            self.move(linear=-0.1, duration=1.0)
        else:
            self.get_logger().info(f"Marker {marker_id}: no action defined")

    def rotate(self, speed: float, angle: float):
        """
        로봇을 angular.z 축으로 회전:
          speed: 회전 속도 (rad/s 양수: 왼쪽, 음수: 오른쪽)
          angle: 회전 각도 (rad)
        """
        twist = Twist()
        twist.angular.z = speed
        duration = abs(angle / speed)
        rate = 10  # 퍼블리시 주기(Hz)
        interval = 1.0 / rate
        start = time.time()
        while time.time() - start < duration and rclpy.ok():
            self.cmd_pub.publish(twist)
            time.sleep(interval)
        # 이동 후 정지 명령
        self.cmd_pub.publish(Twist())

    def move(self, linear: float, duration: float):
        """
        로봇을 직진/후진:
          linear: 선속도 (m/s 양수: 전진, 음수: 후진)
          duration: 동작 시간 (초)
        """
        twist = Twist()
        twist.linear.x = linear
        rate = 10
        interval = 1.0 / rate
        start = time.time()
        while time.time() - start < duration and rclpy.ok():
            self.cmd_pub.publish(twist)
            time.sleep(interval)
        self.cmd_pub.publish(Twist())  # 정지


def main(args=None):
    # ROS 2 초기화 및 노드 스핀
    rclpy.init(args=args)
    node = ArucoDetectRobot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
