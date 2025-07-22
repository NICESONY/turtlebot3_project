import rclpy
from rclpy.node import Node
import serial
import threading

class ManualDumpNode(Node):
    def __init__(self):
        super().__init__('manual_dump_node')

        self.ser = serial.Serial('/dev/ttyACM0', 9600)  # 포트는 실제 환경에 맞게 수정
        self.get_logger().info("Dump 노드 시작됨. 'd' 입력 시 dump 실행")

        # input은 블로킹이므로, 별도 스레드에서 처리
        self.input_thread = threading.Thread(target=self.wait_for_input)
        self.input_thread.start()

    def wait_for_input(self):
        while True:
            user_input = input("▶ Dump 실행하려면 'd' 입력: ").strip()
            if user_input == 'd':
                self.ser.write(b'1')
                self.get_logger().info("Dump 신호 전송 완료 (b'1')")
            else:
                print("⚠️ 잘못된 입력. 'd'만 인식됨.")

def main(args=None):
    rclpy.init(args=args)
    node = ManualDumpNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
