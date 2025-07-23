#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PatrolNode (ROS 2 Humble)
- Nav2 NavigateToPose action 사용
- 모드 1: 외부(WebInput)에서 들어온 목표를 큐(Queue)로 처리
- 모드 2: 미리 정의된 WAYPOINTS를 순차 순찰
- 배터리 임계치 이하 시 도킹 지점으로 이동
- AMCL 초기 위치를 코드 내부에서 /initialpose 로 1회(또는 N회) 발행

필요 토픽/액션
Subscribe:
  /amcl_pose                  (geometry_msgs/PoseWithCovarianceStamped)
  /battery_state              (sensor_msgs/BatteryState)
  /input_data_web             (my_custom_msgs/WebInput)  # x, y, yaw_deg, mod 등

Publish:
  /initialpose                (geometry_msgs/PoseWithCovarianceStamped)
  /battery_low                (std_msgs/Bool)
  /robot_pose_xyyaw           (my_custom_msgs/WebOutput) # 현재 pose/batt/mod 등
  /goal_queue_info            (std_msgs/String)          # 선택: 큐 상태 디버그용

Action:
  navigate_to_pose            (nav2_msgs/action/NavigateToPose)

사용 예)
ros2 launch turtlebot3_gazebo smart_farm_model.launch.py
ros2 launch turtlebot3_navigation2 smart_farm_navigation2_simul.launch.py use_sim_time:=true
ros2 run turtle3_project smart_farm_manager_simul

초기 포즈 확인/수정:
ros2 topic echo /amcl_pose
ros2 topic pub --once /initialpose geometry_msgs/PoseWithCovarianceStamped "header: {frame_id: 'map'}\npose:\n  pose:\n    position:    {x: 0.1, y: 0.0, z: 0.0}\n    orientation: {w: 1.0, x: 0.0, y: 0.0, z: 0.0}"
"""

import math
from collections import deque
from typing import Deque, Tuple, Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.task import Future

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Quaternion, Twist
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, String
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus

# 커스텀 메시지 (사용자 정의 패키지)
from my_custom_msgs.msg import WebInput, WebOutput

# ---------------- Constants ----------------
MODE_QUEUE = 1      # 웹에서 들어온 좌표를 큐로 이동
MODE_WAYPOINT = 2   # 고정 웨이포인트 순찰

# 순찰용 웨이포인트 (예시)
WAYPOINTS = [
    {'x': -0.83, 'y': -1.11, 'yaw_deg': 180},  # 1
    {'x': -0.86, 'y': -2.84, 'yaw_deg':   0},  # 2
    {'x': -0.94, 'y': -4.22, 'yaw_deg':   0},  # 3
    {'x':  0.45, 'y': -4.41, 'yaw_deg':   0},  # 4
    {'x':  0.26, 'y': -2.83, 'yaw_deg':   0},  # 5
    {'x':  0.25, 'y': -1.06, 'yaw_deg':   0},  # 6
    {'x':  0.05, 'y': -5.92, 'yaw_deg':  90},  # 수확물 내리는 곳
    {'x':  0.69, 'y': -2.26, 'yaw_deg': 180},  # 충전소
    {'x':  0.81, 'y': -0.33, 'yaw_deg':   0},  # 창고
]

DOCK_POSE = {'x': 0.69, 'y': -2.26, 'yaw_deg': 180}   # 충전 스테이션 위치
LOW_BATT_PCT = 30.0                                    # 배터리 임계 퍼센트

# AMCL 초기 포즈 설정
INIT_X, INIT_Y, INIT_YAW = 0.0, 0.0, 0.0
USE_INIT_POSE = True
INIT_POSE_PUB_COUNT = 2

# 퍼블리시/타이머 주기
POSE_PUB_PERIOD = 1.0    # 현재 pose pub 주기(초)
IDLE_CHECK_PERIOD = 0.5  # 메인 루프 주기

# 토픽 이름
CURRENT_POSE_TOPIC = '/robot_pose_xyyaw'
QUEUE_INFO_TOPIC   = '/goal_queue_info'

# -------------------------------------------------

def quat_from_yaw(yaw_rad: float) -> Quaternion:
    return Quaternion(
        x=0.0, y=0.0,
        z=math.sin(yaw_rad / 2.0),
        w=math.cos(yaw_rad / 2.0)
    )


class PatrolNode(Node):
    def __init__(self):
        super().__init__('patrol_node')

        # -------- 상태 변수 --------
        self.mode: int = MODE_QUEUE
        self.goal_queue: Deque[Tuple[float, float, float]] = deque()
        self.wp_idx: int = 0

        self.current_pose: Optional[PoseStamped] = None
        self.goal_active: bool = False
        self.current_goal_handle = None  # cancel용 저장

        self.batt_pct: float = 100.0
        self.low_batt_sent: bool = False

        # -------- Publisher / Subscriber --------
        self.init_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.low_battery_pub = self.create_publisher(Bool, '/battery_low', 10)
        self.pose_simple_pub = self.create_publisher(WebOutput, CURRENT_POSE_TOPIC, 10)
        self.queue_info_pub = self.create_publisher(String, QUEUE_INFO_TOPIC, 10)

        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_cb, 10)
        self.create_subscription(BatteryState, '/battery_state', self.batt_cb, 10)
        self.create_subscription(WebInput, '/input_data_web', self.web_cb, 10)
        # 디버깅용 (선택)
        # self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 10)

        # -------- Timers --------
        if USE_INIT_POSE:
            self.once_timer = self.create_timer(1.0, self.publish_initial_pose)
        self.pose_timer = self.create_timer(POSE_PUB_PERIOD, self.publish_current_pose)
        self.main_timer = self.create_timer(IDLE_CHECK_PERIOD, self.idle_loop)

        # -------- Action Client --------
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().info('Nav2 action 서버 대기 중…')
        self.nav_client.wait_for_server()
        self.get_logger().info('Nav2 action 서버 연결 완료')

        self.get_logger().info('PatrolNode 초기화 완료')

    # ------------- 콜백들 -------------
    def publish_initial_pose(self):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = INIT_X
        msg.pose.pose.position.y = INIT_Y
        msg.pose.pose.orientation = quat_from_yaw(INIT_YAW)
        # covariance 예시
        msg.pose.covariance[0] = msg.pose.covariance[7] = 0.25 ** 2   # x, y
        msg.pose.covariance[35] = math.radians(10) ** 2               # yaw

        for _ in range(INIT_POSE_PUB_COUNT):
            self.init_pub.publish(msg)

        self.get_logger().info('📍 초기 pose 발행 완료')
        self.once_timer.cancel()

    def pose_cb(self, msg: PoseWithCovarianceStamped):
        self.current_pose = msg.pose.pose

    def batt_cb(self, msg: BatteryState):
        pct = msg.percentage
        # 드라이버마다 0~1 또는 0~100 으로 올 수 있으니 보정
        self.batt_pct = pct if pct > 1.0 else pct * 100.0

    def web_cb(self, msg: WebInput):
        """웹에서 들어온 명령 처리: 모드 전환 + 좌표 큐 적재"""
        # 1) 모드 전환
        incoming_mode = int(msg.mod) if hasattr(msg, 'mod') else None
        if incoming_mode in (MODE_QUEUE, MODE_WAYPOINT) and incoming_mode != self.mode:
            self._switch_mode(incoming_mode)

        # 2) 좌표 처리 (QUEUE 모드에서만 사용)
        if self.mode == MODE_QUEUE:
            wp = (msg.x, msg.y, msg.yaw_deg)
            self.goal_queue.append(wp)
            self.get_logger().info(f'큐에 goal 추가: {wp} (len={len(self.goal_queue)})')
            self._publish_queue_info()

        # 3) 즉시 실행 시도
        if not self.goal_active:
            self.idle_loop()

    def cmd_vel_cb(self, msg: Twist):
        self.get_logger().info(
            f"cmd_vel: lin({msg.linear.x:.2f},{msg.linear.y:.2f},{msg.linear.z:.2f}) "
            f"ang({msg.angular.x:.2f},{msg.angular.y:.2f},{msg.angular.z:.2f})"
        )

    def publish_current_pose(self):
        if self.current_pose is None:
            return
        x = self.current_pose.position.x
        y = self.current_pose.position.y
        q = self.current_pose.orientation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        yaw_deg = math.degrees(yaw)

        out = WebOutput()
        out.x = x
        out.y = y
        out.yaw_deg = yaw_deg
        out.id = 0
        out.mod = self.mode
        out.batt = int(self.batt_pct)
        self.pose_simple_pub.publish(out)

    # ------------- 메인 루프 -------------
    def idle_loop(self):
        # 현재 goal 진행 중이면 대기
        if self.goal_active or self.current_pose is None:
            return

        # 공통: 배터리 체크 → 도킹
        if self._dock_if_needed():
            return

        # 모드별 행동
        if self.mode == MODE_QUEUE:
            self._try_queue_goal()
        elif self.mode == MODE_WAYPOINT:
            self._try_waypoint_goal()

    # ------------- 모드/도킹/큐 헬퍼들 -------------
    def _switch_mode(self, new_mode: int):
        self.get_logger().info(f'🌐 모드 전환: {self.mode} → {new_mode}')

        # 현재 goal 취소 (선택)
        if self.goal_active:
            self._cancel_current_goal()

        # 상태 초기화
        if new_mode == MODE_QUEUE:
            self.goal_queue.clear()
            self._publish_queue_info()
        elif new_mode == MODE_WAYPOINT:
            self.wp_idx = 0

        self.mode = new_mode
        # 전환 직후 바로 루프 한 번 돌려서 다음 동작 시도
        self.idle_loop()

    def _dock_if_needed(self) -> bool:
        if (self.batt_pct <= LOW_BATT_PCT) and (not self.low_batt_sent):
            self.get_logger().warn(f'배터리 {self.batt_pct:.1f}% ↓ → 도킹 지점 이동')
            self.low_battery_pub.publish(Bool(data=True))
            self.low_batt_sent = True
            self._send_specific_goal(DOCK_POSE, '▶ LowBatt Dock Goal')
            return True
        return False

    def _try_queue_goal(self):
        if self.goal_queue:
            x, y, yaw = self.goal_queue.popleft()
            self._publish_queue_info()
            self._send_goal_tuple((x, y, yaw), '[Queue]')

    def _try_waypoint_goal(self):
        wp = WAYPOINTS[self.wp_idx]
        self._send_goal_dict(wp, f'[WP #{self.wp_idx}]')
        self.wp_idx = (self.wp_idx + 1) % len(WAYPOINTS)

    def _publish_queue_info(self):
        info = f'queue_len={len(self.goal_queue)}'
        self.queue_info_pub.publish(String(data=info))

    # ------------- goal 관련 공통 함수들 -------------
    def _send_specific_goal(self, wp_dict, log_prefix):
        self._send_goal_dict(wp_dict, log_prefix)

    def _send_goal_dict(self, wp, log_prefix):
        self._send_goal_tuple((wp['x'], wp['y'], wp['yaw_deg']), log_prefix)

    def _send_goal_tuple(self, wp_tuple, log_prefix):
        x, y, yaw_deg = wp_tuple
        yaw_rad = math.radians(yaw_deg)

        ps = PoseStamped()
        ps.header.frame_id = 'map'
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.orientation = quat_from_yaw(yaw_rad)

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = ps

        self.get_logger().info(
            f"{log_prefix} (x={x:.2f}, y={y:.2f}, yaw={yaw_deg}°)")

        self.goal_active = True
        send_future: Future = self.nav_client.send_goal_async(goal_msg)
        send_future.add_done_callback(self.goal_resp_cb)

    def goal_resp_cb(self, future: Future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal 거부됨')
            self.goal_active = False
            return
        # 현재 goal handle 저장 (cancel 대비)
        self.current_goal_handle = goal_handle
        goal_handle.get_result_async().add_done_callback(self.result_cb)

    def result_cb(self, future: Future):
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('✓ Goal 성공')
        else:
            self.get_logger().warn(f'✗ Goal 실패 (status={status})')

        self.goal_active = False
        self.current_goal_handle = None

        # 도킹이면 여기서 종료/대기, 그 외에는 다음 목표 시도는 idle_loop에서 처리
        if not self.low_batt_sent:
            self.idle_loop()

    def _cancel_current_goal(self):
        if self.current_goal_handle is None:
            return
        self.get_logger().info('현재 Goal 취소 요청')
        cancel_future = self.current_goal_handle.cancel_goal_async()
        # 결과를 기다릴 필요가 없으면 콜백 생략 가능
        cancel_future.add_done_callback(lambda f: self.get_logger().info('Goal 취소 완료'))
        # 즉시 상태 초기화 (취소 결과와 무관하게)
        self.goal_active = False
        self.current_goal_handle = None


# ------------- main -------------
def main():
    rclpy.init()
    node = PatrolNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
