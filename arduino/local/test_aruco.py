#!/usr/bin/env python3
import os
import time
import threading
import math
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger

class CropHarvestNode(Node):
    def __init__(self):
        super().__init__('crop_harvest_node')
        self.bridge = CvBridge()

        # ROS 설정
        self.create_subscription(
            CompressedImage,
            '/camera/image_raw/compressed',
            self.cb_image,
            10)
        self.pub_img = self.create_publisher(
            CompressedImage,
            '/camera/aruco_annotated/compressed',
            10)
        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10)
        self.servo_client = self.create_client(Trigger, 'servo_control')

        # 카메라 캘리브레이션
        pkg_dir = os.path.dirname(__file__)
        data = np.load(os.path.join(pkg_dir, 'camera_calibration.npz'))
        self.camera_matrix = data['camera_matrix']
        self.dist_coeffs = data['distortion_coefficients']

        # ArUco 설정
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters_create()
        self.marker_length = 0.02

        # 상태 변수
        self.stage = 0  # 0: 수확 → 1: 배출 → 2: 완료
        self.processing = False
        self.harvest_to_drop = {1: 10, 2: 9, 3: 8}
        self.current_harvest_id = 1

        self.threshold_cm = 20.0
        self.approach_speed = 0.05
        self.max_angular = math.pi * 0.2

    def cb_image(self, msg: CompressedImage):
        arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if frame is None:
            return
        annotated = self.annotate_and_control(frame)
        ret, buf = cv2.imencode('.jpg', annotated, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        if not ret:
            return
        out = CompressedImage()
        out.header = msg.header
        out.format = 'jpeg'
        out.data = buf.tobytes()
        self.pub_img.publish(out)

    def annotate_and_control(self, frame):
        h, w = frame.shape[:2]
        center_x = w / 2
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(
            gray, self.aruco_dict, parameters=self.aruco_params)
        if ids is None:
            return frame  # 마커 미탐지 시 정지 호출 제거

        cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners, self.marker_length, self.camera_matrix, self.dist_coeffs)

        for marker_id, tvec, corner, rvec in zip(ids.flatten(), tvecs, corners, rvecs):
            z_cm = float(tvec.flatten()[2] * 100)
            cx = corner.reshape(-1, 2)[:, 0].mean()
            error = (cx - center_x) / center_x
            ang = -error * self.max_angular

            # 수확 마커 인식
            if self.stage == 0 and marker_id in self.harvest_to_drop:
                self.current_harvest_id = marker_id
                if z_cm > self.threshold_cm:
                    twist = Twist()
                    twist.linear.x = self.approach_speed
                    twist.angular.z = ang
                    self.cmd_pub.publish(twist)
                    self.get_logger().info(f"[수확 접근] 마커 {marker_id}, 거리: {z_cm:.1f} cm")
                    self.processing = False
                elif not self.processing:
                    self.processing = True
                    self.stop()
                    threading.Thread(target=self.rotate_and_wait, daemon=True).start()
                break

            # 배출 마커 인식
            dropoff_id = self.harvest_to_drop.get(self.current_harvest_id, 10)
            if self.stage == 1 and marker_id == dropoff_id:
                if z_cm > self.threshold_cm and not self.processing:
                    twist = Twist()
                    twist.linear.x = self.approach_speed
                    twist.angular.z = ang
                    self.cmd_pub.publish(twist)
                    self.get_logger().info(f"[배출 접근] 마커 {marker_id}, 거리: {z_cm:.1f} cm")
                elif not self.processing:
                    self.processing = True
                    self.stop()
                    self.get_logger().info("배출 지점 도달, 360도 회전 후 배출")
                    threading.Thread(target=self.rotate_and_wait_then_dropoff, daemon=True).start()
                break

            cv2.aruco.drawAxis(
                frame, self.camera_matrix, self.dist_coeffs, rvec, tvec, self.marker_length)
        return frame

    def rotate_360(self):
        twist = Twist()
        twist.angular.z = self.max_angular
        target_rotation = math.pi
        rotated = 0.0
        last_time = time.time()
        while rotated < target_rotation:
            current_time = time.time()
            dt = current_time - last_time
            rotated += abs(twist.angular.z) * dt
            last_time = current_time
            self.cmd_pub.publish(twist)
            time.sleep(0.01)
        self.stop()

    def rotate_90_cw(self):
        twist = Twist()
        twist.angular.z = -self.max_angular
        target_rotation = math.pi / 2
        rotated = 0.0
        last_time = time.time()
        while rotated < target_rotation:
            current_time = time.time()
            dt = current_time - last_time
            rotated += abs(twist.angular.z) * dt
            last_time = current_time
            self.cmd_pub.publish(twist)
            time.sleep(0.01)
        self.stop()

    def rotate_and_wait(self):
        self.get_logger().info("수확 마커 도달 → 360도 회전")
        self.rotate_360()
        self.get_logger().info("수확 회전 완료. 'done' 입력 대기")
        self.wait_confirmation()

    def rotate_and_wait_then_dropoff(self):
        self.rotate_360()
        self.perform_dropoff()

    def wait_confirmation(self):
        ans = input("수확 완료 시 'done' 입력: ").strip().lower()
        if ans == 'done':
            self.get_logger().info("수확 완료됨. 90도 회전 후 배출 지점 탐색 시작")
            # 1) 시계 방향 90도 회전
            self.rotate_90_cw()
            # 2) 드롭오프 단계로 전환
            self.stage = 1
            # 3) 배출 마커 인식할 때까지 직진
            twist = Twist()
            twist.linear.x = self.approach_speed
            self.cmd_pub.publish(twist)
            self.get_logger().info("배출 마커 탐색: 직진 중...")
        self.processing = False

    def perform_dropoff(self):
        self.get_logger().info("서보모터 작동 중...")
        if self.servo_client.wait_for_service(timeout_sec=1.0):
            req = Trigger.Request()
            fut = self.servo_client.call_async(req)
            rclpy.spin_until_future_complete(self, fut)
            res = fut.result()
            if res and res.success:
                self.get_logger().info("서보모터 동작 성공")
            else:
                self.get_logger().error("서보모터 동작 실패")
        else:
            self.get_logger().error("서보모터 서비스 연결 실패")
        self.stage = 2
        self.processing = False

    def stop(self):
        self.cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = CropHarvestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
