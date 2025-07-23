#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
import argparse
from my_custom_msgs.msg import WebInput

'''

ros2 run turtle3_project webinputonce.py --x -1.0 --y 2.0 --yaw 90 --id 7 --mod 1

'''

class WebInputOnce(Node):
    def __init__(self, x, y, yaw, mid, mod):
        super().__init__('webinput_once')
        pub = self.create_publisher(WebInput, '/input_data_web', 10)

        msg = WebInput()
        msg.x = x
        msg.y = y
        msg.yaw_deg = yaw
        msg.id = mid
        msg.mod = mod

        pub.publish(msg)
        self.get_logger().info(
            f'📤 /input_data_web -> x={x:.2f}, y={y:.2f}, yaw={yaw}, id={mid}, mod={mod}')
        rclpy.shutdown()

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--x', type=float, default=0.0)
    parser.add_argument('--y', type=float, default=0.0)
    parser.add_argument('--yaw', type=float, default=0.0)
    parser.add_argument('--id', type=int,   default=0)
    parser.add_argument('--mod', type=int,  required=True, help='1=QUEUE, 2=WAYPOINT')
    args, _ = parser.parse_known_args()

    rclpy.init()
    WebInputOnce(args.x, args.y, args.yaw, args.id, args.mod)

if __name__ == '__main__':
    main()
