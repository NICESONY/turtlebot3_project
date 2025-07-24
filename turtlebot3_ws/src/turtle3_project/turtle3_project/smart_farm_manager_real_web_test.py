#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PatrolNode for ROS 2 Humble
- Nav2 NavigateToPose action
- 자체 quat_from_yaw()
- AMCL 초기 위치를 코드 내부에서 /initialpose 로 1회 발행
"""

"""
terminal command

ros2 launch turtlebot3_gazebo smart_farm_model.launch.py

ros2 launch turtlebot3_navigation2 smart_farm_navigation2_simul.launch.py use_sim_time:=true

ros2 run turtle3_project smart_farm_manager_simul


debugging code

ros2 topic echo /amcl_pose


ros2 topic pub --once /initialpose geometry_msgs/PoseWithCovarianceStamped "header: {frame_id: 'map'}
pose:
  pose:
    position:    {x: 0.1, y: 0.0, z: 0.0}
    orientation: {w: 1.0, x: 0.0, y: 0.0, z: 0.0}"

"""



"""
Patrol + Low-battery docking (ROS 2 Humble)
- Subscribe:  /battery_state (sensor_msgs/BatteryState)
- Publish:    /battery_low   (std_msgs/Bool)
- Action:     nav2_msgs/action/NavigateToPose
- AMCL init:  publish /initialpose once
"""

#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
from collections import deque
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Quaternion
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist

from my_custom_msgs.msg import WebInput, WebOutput   # WebOutput을 "현재 상태 pub" 용도로 사용

# ---------------- Params ----------------
DOCK_POSE = {'x':  0.69, 'y': -2.26, 'yaw_deg': 180}  # 충전 스테이션 위치
LOW_BATT_PCT = 30.0                                   # 임계 퍼센트(%)

INIT_X, INIT_Y, INIT_YAW = 0.0, 0.0, 0.0               # 초기 AMCL pose (끄고 싶으면 USE_INIT_POSE=False)
USE_INIT_POSE = True
INIT_POSE_PUB_COUNT = 10                                # 몇 번 쏠지

POSE_PUB_PERIOD = 0.5                                  # 현재 pose pub 주기(초)
IDLE_CHECK_PERIOD = 0.5                                # goal 끝났는지 체크 주기

CURRENT_POSE_TOPIC = '/robot_pose_xyyaw'               # 현재 pose pub 토픽
QUEUE_INFO_TOPIC   = '/goal_queue_info'                # 큐 상태를 문자열로 알려줄 토픽(선택)
# ----------------------------------------


def quat_from_yaw(yaw_rad: float) -> Quaternion:
    return Quaternion(
        x=0.0, y=0.0,
        z=math.sin(yaw_rad / 2.0),
        w=math.cos(yaw_rad / 2.0)
    )


class PatrolNode(Node):
    def __init__(self):
        super().__init__('patrol_node')

        # --- Queue for incoming goals ---
        self.goal_queue = deque()

        # --- Publishers / Subscribers ---
        self.init_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.low_battery_pub = self.create_publisher(Bool, '/battery_low', 10)
        self.pose_simple_pub = self.create_publisher(WebOutput, CURRENT_POSE_TOPIC, 10)
        self.queue_info_pub = self.create_publisher(Bool, QUEUE_INFO_TOPIC, 10)  # Bool가 아니라면 std_msgs/String 권장
        # self.follow_pub = self.create_publisher(Bool, '/follow_mod', 10)
        self.drop_pub = self.create_publisher(Bool, '/drop', 10)

        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_cb, 10)
        self.create_subscription(BatteryState, '/battery_state', self.batt_cb, 10)
        self.create_subscription(WebInput, '/input_data_web', self.enqueue_goal_cb, 10)

        # 선택: cmd_vel 감시
        # self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 10)

        # --- Timers ---
        if USE_INIT_POSE:
            self.once_timer = self.create_timer(1.0, self.publish_initial_pose)
        self.pose_timer = self.create_timer(POSE_PUB_PERIOD, self.publish_current_pose)
        self.main_timer = self.create_timer(IDLE_CHECK_PERIOD, self.idle_loop)

        # --- Action Client ---
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().info('Nav2 action 서버 대기 중…')
        self.nav_client.wait_for_server()
        self.get_logger().info('Nav2 action 서버 연결 완료')

        # --- States ---
        self.current_pose = None
        self.goal_active = False

        self.mode = 0
        
        self.batt_pct = 100.0
        self.low_batt_sent = False

        self.get_logger().info('PatrolNode 초기화 완료')

    # -------- callbacks --------
    def publish_initial_pose(self):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = INIT_X
        self.get_logger().info(f'Pose X: {msg.pose.pose.position.x} 완료')

        msg.pose.pose.position.y = INIT_Y
        self.get_logger().info(f'Pose Y: {msg.pose.pose.position.y} 완료')

        msg.pose.pose.orientation = quat_from_yaw(INIT_YAW)
        msg.pose.covariance[0] = msg.pose.covariance[7] = 0.25 ** 2
        msg.pose.covariance[35] = math.radians(10) ** 2

        for i in range(INIT_POSE_PUB_COUNT):
            self.get_logger().info(f'{i}')
            self.init_pub.publish(msg)

        self.get_logger().info('📍 초기 pose 발행 완료')
        self.once_timer.cancel()

    def pose_cb(self, msg: PoseWithCovarianceStamped):
        self.current_pose = msg.pose.pose

    def batt_cb(self, msg: BatteryState):
        pct = msg.percentage
        self.batt_pct = pct if pct > 1.0 else pct * 100.0

    def enqueue_goal_cb(self, msg: WebInput):
        self.mode = msg.mod
        if self.mode == 1:
            # self.follow_pub.publish(Bool(data=False))
            """웹에서 들어온 좌표를 queue에 저장"""
            wp = (msg.x, msg.y, msg.yaw_deg)
            self.goal_queue.append(wp)
            self.get_logger().info(f'큐에 goal 추가: {wp} (현재 큐 길이={len(self.goal_queue)})')
            # 큐 상태를 퍼블리시 (원하면 형식 바꾸세요)
            self.queue_info_pub.publish(Bool(data=True))

            # goal이 비활성 상태면 바로 처리 시도
            if not self.goal_active:
                self.try_send_next_goal()
            else:
                # time.sleep(10)
                self.low_batt_sent = False
                self.try_send_next_goal()

            self.drop_pub.publish(Bool(data=True))

        elif self.mode == 2:
            # self.follow_pub.publish(Bool(data=False))
            self.get_logger().warn(f'배터리 {self.batt_pct:.1f}% ↓ → 도킹 지점 이동')
            self.low_battery_pub.publish(Bool(data=True))
            self.low_batt_sent = True
            self.send_specific_goal(DOCK_POSE)

        elif self.mode == 3:
            # self.follow_pub.publish(Bool(data=False))
            self.goal_queue.clear()
            x = self.current_pose.position.x
            y = self.current_pose.position.y
            # yaw 추출
            q = self.current_pose.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            yaw_deg = math.degrees(yaw)
            wp = (x, y, yaw_deg)
            self._send_goal_list(wp, "▶ Stop")

        """
        elif self.mode == 4:
            self.goal_queue.clear()
            x = self.current_pose.position.x
            y = self.current_pose.position.y
            # yaw 추출
            q = self.current_pose.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            yaw_deg = math.degrees(yaw)
            wp = (x, y, yaw_deg)
            self._send_goal_list(wp, "▶ Stop")
            # self.follow_pub.publish(Bool(data=True))
        """

        elif self.mode == 4:
            # self.follow_pub.publish(Bool(data=False))
            """웹에서 들어온 좌표를 queue에 저장"""
            wp = (msg.x, msg.y, msg.yaw_deg)
            self.goal_queue.append(wp)
            self.get_logger().info(f'큐에 goal 추가: {wp} (현재 큐 길이={len(self.goal_queue)})')
            # 큐 상태를 퍼블리시 (원하면 형식 바꾸세요)
            self.queue_info_pub.publish(Bool(data=True))

            # goal이 비활성 상태면 바로 처리 시도
            if not self.goal_active:
                self.try_send_next_goal()
            else:
                # time.sleep(10)
                self.low_batt_sent = False
                self.try_send_next_goal()

            self.drop_pub.publish(Bool(data=True))

    def cmd_vel_cb(self, msg: Twist):
        self.get_logger().info(
            f"cmd_vel: lin({msg.linear.x:.2f},{msg.linear.y:.2f},{msg.linear.z:.2f}) "
            f"ang({msg.angular.x:.2f},{msg.angular.y:.2f},{msg.angular.z:.2f})"
        )

    def publish_current_pose(self):
        """현재 위치를 0.5초마다 pub"""
        if self.current_pose is None:
            return
        x = self.current_pose.position.x
        y = self.current_pose.position.y
        # yaw 추출
        q = self.current_pose.orientation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        yaw_deg = math.degrees(yaw)

        out = WebOutput()
        out.x = x
        out.y = y
        out.yaw_deg = yaw_deg
        out.id = 0  # 예시 ID, 필요에 따라 변경  
        out.mod = self.mode  # 예시 모드, 필요에 따라 변경
        out.batt = int(self.batt_pct)
        self.pose_simple_pub.publish(out)

    # -------- main idle loop --------
    def idle_loop(self):
        # goal 진행 중이면 아무 것도 안 함
        if self.goal_active or self.current_pose is None:
            return

        # 배터리 체크 -> 도킹
        if (self.batt_pct <= LOW_BATT_PCT) and (not self.low_batt_sent):
            self.mode = 2
            # self.follow_pub.publish(Bool(data=False))
            self.get_logger().warn(f'배터리 {self.batt_pct:.1f}% ↓ → 도킹 지점 이동')
            self.low_battery_pub.publish(Bool(data=True))
            self.low_batt_sent = True
            self.send_specific_goal(DOCK_POSE)
            return

        # 평상시: 큐에 목표가 있으면 다음거 보냄, 없으면 그냥 대기
        if not self.low_batt_sent:  # 충전 중/대기 중이 아니면
            self.try_send_next_goal()
        else:
            # time.sleep(10)
            self.low_batt_sent = False
            self.try_send_next_goal()

    # -------- goal helpers --------
    def try_send_next_goal(self):
        if self.goal_queue and (not self.goal_active):
            wp = self.goal_queue.popleft()
            self._send_goal_list(wp, f"▶ Goal {wp}")
        else:
            # 큐가 비었으면 그냥 대기
            pass

    def send_specific_goal(self, wp_dict):
        wp = (wp_dict['x'], wp_dict['y'], wp_dict['yaw_deg'])
        self._send_goal_list(wp, "▶ LowBatt Dock Goal")

    def _send_goal_list(self, wp_tuple, log_prefix):
        yaw_rad = math.radians(wp_tuple[2])
        ps = PoseStamped()
        ps.header.frame_id = 'map'
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = wp_tuple[0]
        ps.pose.position.y = wp_tuple[1]
        ps.pose.orientation = quat_from_yaw(yaw_rad)

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = ps

        self.get_logger().info(
            f"{log_prefix} (x={wp_tuple[0]:.2f}, y={wp_tuple[1]:.2f}, yaw={wp_tuple[2]}°)")
        self.goal_active = True
        self.nav_client.send_goal_async(goal_msg).add_done_callback(self.goal_resp_cb)

    # -------- action callbacks --------
    def goal_resp_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal 거부됨')
            self.goal_active = False
            return
        goal_handle.get_result_async().add_done_callback(self.result_cb)

    def result_cb(self, future):
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('✓ Goal 성공')
        else:
            self.get_logger().warn(f'✗ Goal 실패 (status={status})')

        self.goal_active = False

        # 도킹이 아닌 경우라면 다음 목표 시도
        if not self.low_batt_sent:
            self.try_send_next_goal()
        else:
            # time.sleep(10)
            self.low_batt_sent = False
            self.try_send_next_goal()

def main():
    rclpy.init()
    rclpy.spin(PatrolNode())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
