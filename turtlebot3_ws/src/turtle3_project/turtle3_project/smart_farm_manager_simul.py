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

ros2 launch turtlebot3_navigation2 smart_farm_navigation2.launch.py use_sim_time:=true

ros2 run turtle3_project smart_farm_manager


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

import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Quaternion
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus


from my_custom_msgs.msg import WebInput, WebOutput


# ---------------- Params ----------------
WAYPOINTS = [
    {'x': -1.39, 'y': -1.23, 'yaw_deg': 180}, ## 1
    {'x': -1.46, 'y': -3.35, 'yaw_deg': 0.00}, ## 2
    {'x': -1.48, 'y': -5.41, 'yaw_deg': 0.00}, ## 3
    {'x':  1.74, 'y': -5.38, 'yaw_deg': 0.00}, ## 4
    {'x':  1.71, 'y': -3.31, 'yaw_deg': 0.00}, ## 5
    {'x':  1.69, 'y': -1.16, 'yaw_deg': 0.00}, ## 6
    {'x':  2.94, 'y': -6.68, 'yaw_deg': 90}, ## 수화물 내리는 곳
    {'x':  4.10, 'y': -3.48, 'yaw_deg': 180}, ## 충전소
    {'x':  3.67, 'y': -1.02, 'yaw_deg': 0.00}, ## 창고
]

DOCK_POSE = {'x': 4.10, 'y': -3.48, 'yaw_deg': 180}   # 충전 스테이션 위치
LOW_BATT_PCT = 30.0                               # 임계 퍼센트(%)

INIT_X, INIT_Y, INIT_YAW = 0.0, 0.0, 0.0           # 초기 AMCL pose

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

        self.sub_data = self.create_subscription(WebInput, '/input_data_web', self.send_wp_goal, 10)
        print(self.sub_data)

        # --- Publishers / Subscribers ---
        self.init_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)
        self.low_battery_pub = self.create_publisher(Bool, '/battery_low', 10)

        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.pose_cb, 10)
        self.create_subscription(
            BatteryState, '/battery_state', self.batt_cb, 10)

        # --- Timers ---
        self.once_timer = self.create_timer(1.0, self.publish_initial_pose)
        self.main_timer = self.create_timer(0.5, self.main_loop)

        # --- Action Client ---
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().info('Nav2 action 서버 대기 중…')
        self.nav_client.wait_for_server()
        self.get_logger().info('Nav2 action 서버 연결 완료')

        # --- States ---
        self.current_pose = None
        self.wp_idx = 0
        self.goal_active = False

        self.batt_pct = 100.0
        self.low_batt_sent = False
        self.inputdata = WebInput()
        self.waypoint = []

    # -------- callbacks --------
    def publish_initial_pose(self):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = INIT_X
        msg.pose.pose.position.y = INIT_Y
        msg.pose.pose.orientation = quat_from_yaw(INIT_YAW)
        msg.pose.covariance[0] = msg.pose.covariance[7] = 0.25 ** 2
        msg.pose.covariance[35] = math.radians(10) ** 2

        for _ in range(2):
            self.init_pub.publish(msg)

        self.get_logger().info('📍 초기 pose 발행 완료')
        self.once_timer.cancel()

    def pose_cb(self, msg: PoseWithCovarianceStamped):
        self.current_pose = msg.pose.pose

    def batt_cb(self, msg: BatteryState):
        pct = msg.percentage
        # 어떤 드라이버는 0~1.0 범위로 주기도 함 -> 보정
        self.batt_pct = pct if pct > 1.0 else pct * 100.0

    def main_loop(self):
        # 아직 pose 못 받았거나 goal 진행 중이면 skip
        if self.goal_active or self.current_pose is None:
            return

        # 배터리 체크
        if (self.batt_pct <= LOW_BATT_PCT) and (not self.low_batt_sent):
            self.get_logger().warn(f'배터리 {self.batt_pct:.1f}% ↓ → 도킹 지점 이동')
            self.low_battery_pub.publish(Bool(data=True))
            self.low_batt_sent = True
            self.send_specific_goal(DOCK_POSE)
            return

        # 평상시 순찰
        if not self.low_batt_sent:
            self.inputdata.x = 4.10
            self.inputdata.y = -3.48
            self.inputdata.yaw_deg = 180.0
            self.send_wp_goal(msg= self.inputdata)

    # -------- goal send helpers --------
    def send_wp_goal(self, msg):
        self.inputdata.x = msg.x
        self.inputdata.y = msg.y
        self.inputdata.yaw_deg = msg.yaw_deg
        self.waypoint = [self.inputdata.x , self.inputdata.y, self.inputdata.yaw_deg]
        # wp = waypoint
        self._send_goal_common(self.waypoint, f"▶ Goal #{self.waypoint}")

    def callback_inputData(self, msg):
        self.inputdata.x = msg.x
        self.inputdata.y = msg.y
        self.inputdata.yaw_deg = msg.yaw_deg
        self.get_logger().info(f"x: {self.inputdata.x}, y: {self.inputdata.y}, yaw_deg: {self.inputdata.yaw_deg}, id: {self.inputdata.id}, mod: {self.inputdata.mod}")


    def send_specific_goal(self, wp_dict):
        self._send_goal_common(wp_dict, "▶ LowBatt Dock Goal")

    def _send_goal_common(self, wp, log_prefix):
        yaw_rad = math.radians(wp[2]) # 'yaw_deg'
        ps = PoseStamped()
        ps.header.frame_id = 'map'
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = wp[0] # 'x'
        ps.pose.position.y = wp[1] # 'y'
        ps.pose.orientation = quat_from_yaw(yaw_rad) # 'yaw_deg'

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = ps

        self.get_logger().info(
            f"{log_prefix} (x={wp[0]:.2f}, y={wp[1]:.2f}, yaw={wp[2]}°)")
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

        # 순찰 중이었다면 다음 WP로
        if not self.low_batt_sent:
            self.wp_idx = (self.wp_idx + 1) % len(WAYPOINTS)

        # 도킹 후에는 정지(필요 시 여기서 충전 완료 이벤트 기다리는 로직 추가 가능)
        self.goal_active = False


def main():
    rclpy.init()
    rclpy.spin(PatrolNode())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
