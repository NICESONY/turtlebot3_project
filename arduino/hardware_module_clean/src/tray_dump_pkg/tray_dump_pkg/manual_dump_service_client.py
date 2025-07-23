#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

class DumpServiceClient(Node):
    def __init__(self):
        super().__init__('dump_service_client')
        self.cli = self.create_client(Trigger, 'trigger_dump')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('서비스 대기 중...')

    def send_request(self):
        req = Trigger.Request()
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

def main(args=None):
    rclpy.init(args=args)
    node = DumpServiceClient()
    try:
        while True:
            cmd = input("▶ Dump 실행하려면 'd' 입력: ").strip()
            if cmd == 'd':
                result = node.send_request()
                print(f"응답: {result.success}, 메시지: {result.message}")
            else:
                print("'d'만 인식됩니다.")
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
