#!/usr/bin/env python3
import json

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan

from .utils import iou

class FollowerNode(Node):
    def __init__(self):
        super().__init__('yolo_follower')

        self.declare_parameters('', [
            ('bbox_topic', '/yolo_follower/bboxes'),
            ('cmd_vel_topic', '/cmd_vel'),
            ('k_yaw', 0.01),
            ('target_area', 0.15),            # 남아있지만 사용 안 할 수도 있음
            ('yaw_deadband_px', 60),          # yaw 데드밴드 픽셀
            ('v_min', 0.08),                  # 기본 전진 속도
            ('stop_area', 0.25),              # 너무 가까울 때 멈춤 기준
            ('max_lin_vel', 0.25),
            ('max_ang_vel', 1.0),
            ('search_ang_vel', 0.2),
            ('max_lost', 50),
            ('min_iou_keep', 0.01),
            ('use_lidar_stop', False),        # LiDAR 안전정지 사용 여부
            ('lidar_topic', '/scan'),
            ('safe_dist', 0.35),              # m
        ])

        # 파라미터 로드
        self.k_yaw = self.get_parameter('k_yaw').value
        self.yaw_deadband_px = self.get_parameter('yaw_deadband_px').value
        self.v_min = self.get_parameter('v_min').value
        self.stop_area = self.get_parameter('stop_area').value
        self.max_lin_vel = self.get_parameter('max_lin_vel').value
        self.max_ang_vel = self.get_parameter('max_ang_vel').value
        self.search_ang_vel = self.get_parameter('search_ang_vel').value
        self.max_lost = self.get_parameter('max_lost').value
        self.min_iou_keep = self.get_parameter('min_iou_keep').value
        self.use_lidar_stop = self.get_parameter('use_lidar_stop').value
        self.safe_dist = self.get_parameter('safe_dist').value

        bbox_topic = self.get_parameter('bbox_topic').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.create_subscription(String, bbox_topic, self.bbox_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)

        # LiDAR 안전 정지
        self.front_blocked = False
        if self.use_lidar_stop:
            lidar_topic = self.get_parameter('lidar_topic').value
            self.create_subscription(LaserScan, lidar_topic, self.scan_cb, 10)

        # 타깃 락
        self.target_box = None
        self.target_lost_cnt = 0

        self.get_logger().info('FollowerNode started (Always-Follow mode)')

    # ---------- LiDAR 콜백 ----------
    def scan_cb(self, msg: LaserScan):
        # 전방 ±15° 정도를 확인 (간단 구현)
        n = len(msg.ranges)
        center = n // 2
        window = max(1, n // 24)
        vals = [r for r in msg.ranges[center-window:center+window] if 0.0 < r < float('inf')]
        if not vals:
            self.front_blocked = False
            return
        self.front_blocked = (min(vals) < self.safe_dist)

    # ---------- BBOX 콜백 ----------
    def bbox_cb(self, msg: String):
        det_list = json.loads(msg.data)
        twist = Twist()

        # 타깃 없으면 새로 선정
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

        # 타깃 있는데 이번 프레임 비었음
        if not det_list:
            self._lose_target(twist)
            return

        # IoU 매칭
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

        # 업데이트 후 이동
        self.target_box = best['bbox']
        self.target_lost_cnt = 0
        self._control_move(best, twist)

    # ---------- Helper ----------
    def _lose_target(self, twist):
        self.target_lost_cnt += 1
        if self.target_lost_cnt > self.max_lost:
            self.target_box = None
        twist.angular.z = self.search_ang_vel
        twist.linear.x = 0.0
        self.cmd_pub.publish(twist)

    def _control_move(self, det, twist: Twist):
        x1, y1, x2, y2 = det['bbox']
        w = det['img_w']; h = det['img_h']
        cx = (x1 + x2) / 2.0
        area_ratio = det['area'] / (w * h)

        # yaw 제어
        error_x = cx - w / 2.0
        yaw_cmd  = - self.k_yaw * error_x

        # 기본 전진 속도
        lin_cmd = self.v_min

        # yaw가 많이 틀어져 있으면 먼저 회전만
        if abs(error_x) > self.yaw_deadband_px:
            lin_cmd = 0.0

        # 너무 가까우면 정지
        if area_ratio > self.stop_area:
            lin_cmd = 0.0

        # LiDAR 안전 정지
        if self.use_lidar_stop and self.front_blocked:
            lin_cmd = 0.0

        # 제한
        lin_cmd = max(min(lin_cmd, self.max_lin_vel), 0.0)
        yaw_cmd  = max(min(yaw_cmd,  self.max_ang_vel), -self.max_ang_vel)

        twist.linear.x  = lin_cmd
        twist.angular.z = yaw_cmd
        self.cmd_pub.publish(twist)

        # 디버그 출력
        self.get_logger().info(
            f"err_x={error_x:.1f}, area={area_ratio:.3f}, lin={lin_cmd:.3f}, yaw={yaw_cmd:.3f}, blocked={self.front_blocked}")


def main(args=None):
    rclpy.init(args=args)
    node = FollowerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()