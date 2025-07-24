# file: follow_yolo_node.py
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import math

TARGET_CLASS = 'person'   # 따라갈 객체 (원하면 'box', 'dog' 등)
CONF_THRES   = 0.5

class YoloFollower(Node):
    def __init__(self):
        super().__init__('yolo_follower')
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_cb, 10)
        self.cmd_pub   = self.create_subscription(Twist, '/cmd_vel', self.dummy_cb, 10) # placeholder
        self.cmd_pub   = self.create_publisher(Twist, '/cmd_vel', 10)
        self.follow_sub = self.create_subscription(Bool, '/follow_mod', self.follow_cm, 10)

        # YOLO 모델 로드(사전학습)
        self.model = YOLO('yolov8n.pt')  # 속도 우선. 정확도 원하면 s/m/l로 교체

        # PID gains (회전/전진)
        self.k_yaw  = 0.005   # 화면 중앙 대비 오차 → 회전 제어
        self.k_dist = 0.002   # bbox 면적(or 높이) 기반 전진 제어
        self.target_area = 0.15  # 적당한 bbox 넓이 비율(카메라 프레임 대비)
        self.follow_mod = False

        self.last_cmd_time = self.get_clock().now()

    def dummy_cb(self, msg): # nothing
        pass

    def image_cb(self, msg):
        if follow_mod:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            h, w, _ = cv_img.shape

            # YOLO 추론
            results = self.model(cv_img, verbose=False, conf=CONF_THRES)
            boxes = results[0].boxes

            target_box = None
            max_area = 0

            for box, cls_id in zip(boxes.xyxy, boxes.cls):
                cls_name = self.model.names[int(cls_id)]
                if cls_name != TARGET_CLASS:
                    continue
                x1, y1, x2, y2 = box.cpu().numpy().astype(int)
                area = (x2 - x1) * (y2 - y1)
                if area > max_area:
                    max_area = area
                    target_box = (x1, y1, x2, y2)

            twist = Twist()

            if target_box is not None:
                x1, y1, x2, y2 = target_box
                cx = (x1 + x2) / 2
                cy = (y1 + y2) / 2
                box_area_ratio = max_area / (w * h)

                # 좌우 오차 → z 회전
                error_x = (cx - w/2)
                yaw_cmd = - self.k_yaw * error_x   # 카메라 기준 좌우 반대면 부호 조정

                # 크기 오차 → x 전진/후진
                dist_err = (self.target_area - box_area_ratio)
                lin_cmd = self.k_dist * dist_err

                # 안전 한계
                lin_cmd = max(min(lin_cmd, 0.25), -0.1)  # 최대 0.25 m/s
                yaw_cmd = max(min(yaw_cmd, 1.0), -1.0)

                twist.linear.x  = lin_cmd
                twist.angular.z = yaw_cmd

                # 디버그 시각화
                cv2.rectangle(cv_img, (x1,y1), (x2,y2), (0,255,0), 2)
                cv2.circle(cv_img, (int(cx), int(cy)), 4, (0,0,255), -1)
                cv2.putText(cv_img, f"{TARGET_CLASS} {box_area_ratio:.2f}", (x1, y1-5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)

            else:
                # 못 찾으면 정지 / 회전 등 전략 선택
                twist.linear.x  = 0.0
                twist.angular.z = 0.2  # 천천히 회전하며 탐색

            self.cmd_pub.publish(twist)

            # 필요시 이미지 디스플레이 (디버그용)
            # cv2.imshow("debug", cv_img)
            # cv2.waitKey(1)

    def follow_cm(self, msg):
        self.follow_mod = msg


def main(args=None):
    rclpy.init(args=args)
    node = YoloFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
