#!/usr/bin/env python3
import math
import json
from typing import List, Dict, Any

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus


# ====== 기본 WAYPOINTS (필요시 파라미터로 덮어씀) ======
DEFAULT_WAYPOINTS: List[Dict[str, float]] = [
    {'x': -1.26, 'y': -1.27, 'yaw_deg': 0},
    {'x': -1.31, 'y': -3.29, 'yaw_deg': 0},
    {'x': -1.47, 'y': -5.47, 'yaw_deg': 0},
    {'x':  1.66, 'y': -5.40, 'yaw_deg': 0},
    {'x':  1.61, 'y': -3.32, 'yaw_deg': 0},
    {'x':  1.44, 'y': -1.32, 'yaw_deg': 0},
    {'x':  3.26, 'y': -6.72, 'yaw_deg': 0},
    {'x':  4.06, 'y': -3.44, 'yaw_deg': 0},
]


def quat_from_yaw(yaw_rad: float) -> Quaternion:
    """yaw(rad) → Quaternion(z, w 값만 사용)."""
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw_rad / 2.0)
    q.w = math.cos(yaw_rad / 2.0)
    return q


class PatrolNode(Node):
    def __init__(self):
        super().__init__('patrol_node')

        # ---------- 파라미터 선언 ----------
        self.declare_parameter('init_x', 0.0)
        self.declare_parameter('init_y', 0.0)
        self.declare_parameter('init_yaw_deg', 0.0)
        self.declare_parameter('return_to_init', True)
        # 문자열(JSON) 또는 빈 문자열을 받을 수 있도록
        self.declare_parameter('waypoints', '')

        self.init_x = float(self.get_parameter('init_x').value)
        self.init_y = float(self.get_parameter('init_y').value)
        self.init_yaw = math.radians(float(
            self.get_parameter('init_yaw_deg').value))

        self.return_to_init = bool(
            self.get_parameter('return_to_init').value)

        wp_param_raw = self.get_parameter('waypoints').value
        if isinstance(wp_param_raw, str) and wp_param_raw.strip():
            try:
                self.waypoints = json.loads(wp_param_raw)
                assert isinstance(self.waypoints, list)
            except Exception as e:
                self.get_logger().warn(
                    f"waypoints 파라미터 파싱 실패. 기본값 사용. 에러: {e}")
                self.waypoints = DEFAULT_WAYPOINTS
        else:
            self.waypoints = DEFAULT_WAYPOINTS

        if len(self.waypoints) == 0:
            self.get_logger().error('WAYPOINTS가 비어 있습니다. 노드 종료.')
            rclpy.shutdown()
            return

        self.get_logger().info(f'WAYPOINTS 개수: {len(self.waypoints)}')
        self.get_logger().info(f'초기 위치: ({self.init_x}, {self.init_y}, {math.degrees(self.init_yaw)}°)')
        self.get_logger().info(f'마지막 후 초기 복귀 여부: {self.return_to_init}')

        # ---------- 초기 pose 퍼블리셔 ----------
        self.init_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)
        # 1초 후 한 번 실행
        self.once_timer = self.create_timer(1.0, self.publish_initial_pose)

        # ---------- Nav2 액션 클라이언트 ----------
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().info('Nav2 action 서버 대기 중…')
        self.nav_client.wait_for_server()
        self.get_logger().info('Nav2 action 서버 연결 완료')

        # ---------- AMCL pose 구독 ----------
        self.current_pose = None
        self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_cb,
            10
        )

        # ---------- 상태 변수 ----------
        self.wp_idx: int = 0
        self.goal_active: bool = False
        self.returning: bool = False  # 초기 위치로 돌아가는 중인지 여부

        # 메인 루프 타이머
        self.create_timer(0.5, self.main_loop)

    # ---------- 초기 pose 발행 ----------
    def publish_initial_pose(self):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.pose.pose.position.x = self.init_x
        msg.pose.pose.position.y = self.init_y
        msg.pose.pose.orientation = quat_from_yaw(self.init_yaw)

        # 간단한 공분산 값 (±0.25 m, ±10°)
        msg.pose.covariance[0] = msg.pose.covariance[7] = 0.25 ** 2
        msg.pose.covariance[35] = math.radians(10) ** 2

        # 두 번 발행(유실 방지)
        for _ in range(2):
            self.init_pub.publish(msg)

        self.get_logger().info('📍 초기 pose 발행 완료')
        self.once_timer.cancel()

    # ---------- AMCL pose 콜백 ----------
    def pose_cb(self, msg: PoseWithCovarianceStamped):
        self.current_pose = msg.pose.pose

    # ---------- 메인 루프 ----------
    def main_loop(self):
        # goal이 진행 중이거나, 아직 pose를 못 받았으면 대기
        if self.goal_active or self.current_pose is None:
            return

        if not self.returning and self.wp_idx < len(self.waypoints):
            self.send_wp_goal()
        elif self.returning:
            # 반환 골은 result 콜백에서 start
            pass
        else:
            # 여기까지 오는 일은 없음. 안전 로그
            self.get_logger().info('모든 작업 완료 상태.')
            rclpy.shutdown()

    # ---------- 웨이포인트 goal 전송 ----------
    def send_wp_goal(self):
        wp = self.waypoints[self.wp_idx]
        yaw_rad = math.radians(wp.get('yaw_deg', 0.0))

        ps = PoseStamped()
        ps.header.frame_id = 'map'
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = float(wp['x'])
        ps.pose.position.y = float(wp['y'])
        ps.pose.orientation = quat_from_yaw(yaw_rad)

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = ps

        self.get_logger().info(
            f"▶ Goal #{self.wp_idx}  (x={wp['x']:.2f}, y={wp['y']:.2f}, yaw={wp.get('yaw_deg', 0)}°)")
        self.goal_active = True

        self.nav_client.send_goal_async(goal_msg)\
            .add_done_callback(self.goal_resp_cb)

    # ---------- 액션 서버 응답 ----------
    def goal_resp_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal 거부')
            self.goal_active = False
            return
        goal_handle.get_result_async().add_done_callback(self.result_cb)

    # ---------- 웨이포인트 결과 ----------
    def result_cb(self, future):
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"✓ Waypoint {self.wp_idx} 도착")
        else:
            self.get_logger().warn(
                f"✗ Waypoint {self.wp_idx} 실패 (status={status})")

        self.goal_active = False

        # 다음 웨이포인트로 갈지, 초기 위치로 갈지 결정
        if not self.returning:
            if self.wp_idx < len(self.waypoints) - 1:
                # 다음 웨이포인트
                self.wp_idx += 1
            else:
                # 마지막 웨이포인트 완료
                if self.return_to_init:
                    self.get_logger().info('🔙 마지막 웨이포인트 완료, 초기 위치로 복귀합니다.')
                    self.returning = True
                    self.send_return_to_init()
                else:
                    self.get_logger().info('✅ 모든 웨이포인트 완료, 노드 종료')
                    rclpy.shutdown()
        else:
            # returning 상태인데 여기로 올 일은 거의 없음
            pass

    # ---------- 초기 위치로 복귀 ----------
    def send_return_to_init(self):
        ps = PoseStamped()
        ps.header.frame_id = 'map'
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = self.init_x
        ps.pose.position.y = self.init_y
        ps.pose.orientation = quat_from_yaw(self.init_yaw)

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = ps

        self.nav_client.send_goal_async(goal_msg)\
            .add_done_callback(self.return_goal_resp_cb)

    def return_goal_resp_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('초기위치 복귀 Goal 거부')
            rclpy.shutdown()
            return
        goal_handle.get_result_async()\
            .add_done_callback(self.return_result_cb)

    def return_result_cb(self, future):
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('✅ 초기 위치 복귀 성공, 노드 종료')
        else:
            self.get_logger().warn(f'✗ 초기 위치 복귀 실패 (status={status})')
        rclpy.shutdown()


def main():
    rclpy.init()
    node = PatrolNode()
    rclpy.spin(node)
    # shutdown은 Node 내부에서 호출될 수도 있으므로 여기서도 안전하게 호출
    rclpy.shutdown()


if __name__ == '__main__':
    main()
