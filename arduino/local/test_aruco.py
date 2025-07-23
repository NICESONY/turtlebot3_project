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
        # ROS 토픽 설정
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
        # 서보모터 동작 서비스 클라이언트
        self.servo_client = self.create_client(Trigger, 'servo_control')
        # 캘리브레이션
        pkg_dir = os.path.dirname(__file__)
        data = np.load(os.path.join(pkg_dir, 'camera_calibration.npz'))
        self.camera_matrix = data['camera_matrix']
        self.dist_coeffs = data['distortion_coefficients']
        # ArUco 파라미터
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters_create()
        self.marker_length = 0.02
        # 상태 및 파라미터
        self.stage = 0  # 0: 수확 접근 → 1: 배출 접근 → 2: 완료
        self.processing = False
        self.harvest_id = 1
        self.dropoff_id = 10
        self.threshold_cm = 20.0
        self.approach_speed = 0.05  # m/s
        self.max_angular = 0.3      # rad/s

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
            if self.stage != 1:
                self.stop()
            return frame

        cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners, self.marker_length, self.camera_matrix, self.dist_coeffs)

        for marker_id, tvec, corner, rvec in zip(ids.flatten(), tvecs, corners, rvecs):
            z_cm = float(tvec.flatten()[2] * 100)

            # 수확 지점(1번) 접근 및 360° 회전
            if self.stage == 0 and marker_id == self.harvest_id:
                cx = corner.reshape(-1,2)[:,0].mean()
                error = (cx - center_x) / center_x
                ang = -error * self.max_angular

                if z_cm > self.threshold_cm:
                    twist = Twist()
                    twist.linear.x = self.approach_speed
                    twist.angular.z = ang
                    self.cmd_pub.publish(twist)
                    # self.get_logger().info(f"현재 거리: {z_cm:.2f} cm (임계값: {self.threshold_cm:.2f} cm)")
                elif not self.processing:
                    self.processing = True
                    self.stop()
                    threading.Thread(target=self.rotate_and_wait, daemon=True).start()
                break

            # 배출 지점(10번) 접근 및 서보모터 동작
            elif self.stage == 1 and marker_id == self.dropoff_id:
                cx = corner.reshape(-1,2)[:,0].mean()
                error = (cx - center_x) / center_x
                ang = -error * self.max_angular

                if z_cm > self.threshold_cm and not self.processing:
                    twist = Twist()
                    twist.linear.x = self.approach_speed
                    twist.angular.z = ang
                    self.cmd_pub.publish(twist)
                    # self.get_logger().info(f"현재 거리: {z_cm:.2f} cm (임계값: {self.threshold_cm:.2f} cm)")
                elif not self.processing:
                    self.processing = True
                    self.stop()
                    self.get_logger().info("Arrived at drop-off point (7cm)")
                    threading.Thread(target=self.perform_dropoff, daemon=True).start()
                break

            # 축 그리기
            cv2.aruco.drawAxis(
                frame, self.camera_matrix, self.dist_coeffs, rvec, tvec, self.marker_length)
        return frame

    def rotate_and_wait(self):
        self.get_logger().info("Reached harvest threshold, rotating 360° before dump.")
        twist = Twist()
        twist.angular.z = self.max_angular
        duration = 2 * math.pi / abs(self.max_angular)
        start = time.time()
        while time.time() - start < duration:
            self.cmd_pub.publish(twist)
            time.sleep(0.01)
        self.stop()
        self.get_logger().info("Rotation complete, waiting for 'done' to confirm harvest.")
        self.wait_confirmation()

    def wait_confirmation(self):
        ans = input("수확 완료 시 'done' 입력: ").strip().lower()
        self.processing = False
        if ans == 'done':
            self.get_logger().info("Harvest confirmed, scanning for drop-off marker")
            self.stage = 1

    def perform_dropoff(self):
        self.get_logger().info("Stage 1: Drop-off action")
        # 서보모터 서비스 호출
        if self.servo_client.wait_for_service(timeout_sec=1.0):
            req = Trigger.Request()
            fut = self.servo_client.call_async(req)
            rclpy.spin_until_future_complete(self, fut)
            res = fut.result()
            if res and res.success:
                self.get_logger().info("Servo actuated successfully")
            else:
                self.get_logger().error("Servo actuation failed")
        else:
            self.get_logger().error("Servo service unavailable")
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
