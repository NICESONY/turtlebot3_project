#!/usr/bin/env python3
import json

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

from .utils import iou


class FollowerNode(Node):
    def __init__(self):
        super().__init__('yolo_follower')

        self.declare_parameters(
            '',
            [
                ('bbox_topic', '/yolo_follower/bboxes'),
                ('cmd_vel_topic', '/cmd_vel'),
                ('k_yaw', 0.005),
                ('k_dist', 0.002),
                ('target_area', 0.15),
                ('max_lin_vel', 0.25),
                ('max_ang_vel', 1.0),
                ('search_ang_vel', 0.2),
                ('max_lost', 15),
                ('min_iou_keep', 0.1),
            ],
        )

        # 파라미터
        self.k_yaw = self.get_parameter('k_yaw').value
        self.k_dist = self.get_parameter('k_dist').value
        self.target_area = self.get_parameter('target_area').value
        self.max_lin_vel = self.get_parameter('max_lin_vel').value
        self.max_ang_vel = self.get_parameter('max_ang_vel').value
        self.search_ang_vel = self.get_parameter('search_ang_vel').value
        self.max_lost = self.get_parameter('max_lost').value
        self.min_iou_keep = self.get_parameter('min_iou_keep').value

        # Pub/Sub
        bbox_topic = self.get_parameter('bbox_topic').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.create_subscription(String, bbox_topic, self.bbox_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)

        # 타깃 상태
        self.target_box = None
        self.target_lost_cnt = 0

        self.get_logger().info('FollowerNode started')

    # ---------- 콜백 ----------
    def bbox_cb(self, msg: String):
        det_list = json.loads(msg.data)
        twist = Twist()

        # 타깃 없으면 새로 선택
        if self.target_box is None:
            if not det_list:
                twist.angular.z = self.search_ang_vel
                self.cmd_pub.publish(twist)
                return
            target = max(det_list, key=lambda d: d['area'])
            self.target_box = target['bbox']
            self.target_lost_cnt = 0
            self._control_move(target, twist)
            return

        # 타깃 있는데 이번에 못 봄
        if not det_list:
            self._lose_target(twist)
            return

        # IoU 최대 매칭
        best = None
        best_score = 0.0
        for d in det_list:
            score = iou(self.target_box, d['bbox'])
            if score > best_score:
                best_score = score
                best = d

        if best is None or best_score < self.min_iou_keep:
            self._lose_target(twist)
            return

        # 타깃 업데이트 후 제어
        self.target_box = best['bbox']
        self.target_lost_cnt = 0
        self._control_move(best, twist)

    # ---------- helper ----------
    def _lose_target(self, twist):
        self.target_lost_cnt += 1
        if self.target_lost_cnt > self.max_lost:
            self.target_box = None
        twist.angular.z = self.search_ang_vel
        self.cmd_pub.publish(twist)

    def _control_move(self, det, twist: Twist):
        x1, y1, x2, y2 = det['bbox']
        w = det['img_w']
        h = det['img_h']
        cx = (x1 + x2) / 2.0
        area_ratio = det['area'] / (w * h)

        # 각속도
        error_x = cx - w / 2.0
        yaw_cmd = -self.k_yaw * error_x

        # 선속도
        dist_err = self.target_area - area_ratio
        lin_cmd = self.k_dist * dist_err

        # saturation
        lin_cmd = max(min(lin_cmd, self.max_lin_vel), -0.1)
        yaw_cmd = max(min(yaw_cmd, self.max_ang_vel), -self.max_ang_vel)

        twist.linear.x = lin_cmd
        twist.angular.z = yaw_cmd
        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = FollowerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
