import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import serial
import serial.tools.list_ports

class DumpServiceServer(Node):
    def __init__(self):
        super().__init__('dump_service_server')
        port = self.find_arduino_port()
        if not port:
            self.get_logger().error("아두이노 포트를 찾을 수 없습니다.")
            raise RuntimeError("아두이노 연결 실패")

        self.ser = serial.Serial(port, 9600, timeout=1)
        self.get_logger().info(f"아두이노에 연결됨: {port}")
        self.srv = self.create_service(Trigger, 'trigger_dump', self.handle_dump_request)
        self.get_logger().info("Dump 서비스 서버 실행 중 (trigger_dump 서비스 대기)")

    def find_arduino_port(self):
        import serial.tools.list_ports

        ports = serial.tools.list_ports.comports()
        for port in ports:
            # 디바이스 설명 및 ID 정보 가져오기
            desc = port.description.lower()
            vid = f"{port.vid:04x}" if port.vid else ""
            pid = f"{port.pid:04x}" if port.pid else ""
            serial_number = port.serial_number or ""

            # Uno 기준: Arduino SA Uno R3 = idVendor=2341, idProduct=0043
            if port.vid == 0x2341 and port.pid == 0x0043:
                print(f"아두이노 Uno 감지됨: {port.device} ({desc})")
                return port.device

        print("아두이노 포트를 찾지 못했습니다.")
        return None

    def handle_dump_request(self, request, response):
        try:
            self.ser.write(b'1')
            self.get_logger().info("Dump 신호 전송 완료")
            response.success = True
            response.message = 'Dump 명령 전송 성공'
        except Exception as e:
            self.get_logger().error(f"전송 실패: {e}")
            response.success = False
            response.message = 'Dump 명령 전송 실패'
        return response

def main(args=None):
    rclpy.init(args=args)
    try:
        node = DumpServiceServer()
        rclpy.spin(node)
    except Exception as e:
        print(f"노드 실행 실패: {e}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
