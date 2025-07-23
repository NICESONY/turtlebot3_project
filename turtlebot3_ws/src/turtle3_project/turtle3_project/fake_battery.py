#!/usr/bin/env python3


"""

# 0.25 (=25%)를 1Hz로 발행

ros2 topic pub -r 1 /battery_state sensor_msgs/msg/BatteryState "
header: {frame_id: 'base_link'}
voltage: 12.0
current: -0.5
percentage: 0.25
power_supply_status: 2    # DISCHARGING
power_supply_health: 1    # GOOD
power_supply_technology: 3
present: true
"

"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState

class FakeBattery(Node):
    def __init__(self):
        super().__init__('fake_battery')
        self.pub = self.create_publisher(BatteryState, '/battery_state', 10)
        self.pct = 1.0  # 100%
        self.timer = self.create_timer(2.0, self.tick)

    def tick(self):
        msg = BatteryState()
        msg.percentage = self.pct
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        msg.present = True
        self.pub.publish(msg)
        self.get_logger().info(f'publish battery: {self.pct*100:.1f}%')
        self.pct = max(0.0, self.pct - 0.05)  # 5%씩 감소

def main():
    rclpy.init()
    rclpy.spin(FakeBattery())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
