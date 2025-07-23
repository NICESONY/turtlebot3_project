#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import serial
import serial.tools.list_ports
import time

class DumpServiceServer(Node):
    def __init__(self):
        super().__init__('dump_service_server')

        # 1) Arduino 포트 찾기
        port = self.find_arduino_port()
        if not port:
            self.get_logger().error("아두이노 포트를 찾을 수 없습니다.")
            raise RuntimeError("아두이노 연결 실패")

        # 2) Serial 연결 설정
        self.ser = serial.Serial(port, 9600, timeout=1)
        self.get_logger().info(f"아두이노에 연결됨: {port}")

        # 3) 서비스 서버 생성 (이름: 'servo_control')
        self.srv = self.create_service(
            Trigger,
            'servo_control',
            self.handle_dump_request
        )
        self.get_logger().info("Dump 서비스 서버 실행 중: 'servo_control'")

        # 4) 부팅 메시지 한 줄 버리기
        #    (Arduino 리셋 시 올라오는 초기 헬로 로그를 무시)
        time_start = time.time()
        while time.time() - time_start < 5.0:
            line = self.ser.readline().decode(errors='ignore').strip()
            if line:
                self.get_logger().info(f"[부팅 메시지 무시] '{line}'")
                break

    def find_arduino_port(self):
        ports = serial.tools.list_ports.comports()
        for port in ports:
            # Arduino Uno R3 기본 VID/PID
            if port.vid == 0x2341 and port.pid == 0x0043:
                self.get_logger().info(f"Arduino Uno 감지: {port.device}")
                return port.device
        return None

    def handle_dump_request(self, request, response):
        try:
            # 입력 버퍼 정리
            self.ser.reset_input_buffer()

            # 1) 서보 명령 전송
            self.get_logger().info("▶ Serial.write: b'1'")
            sent = self.ser.write(b'1')
            self.get_logger().info(f"◀ {sent} bytes 전송 완료")

            # 2) CMD_RECEIVED ACK 대기
            line = self.ser.readline().decode(errors='ignore').strip()
            self.get_logger().info(f"◀ Arduino 응답1: '{line}'")
            if line != 'CMD_RECEIVED':
                response.success = False
                response.message = f"CMD_RECEIVED ACK 미수신 (got '{line}')"
                return response

            # 3) DUMP_DONE ACK 대기
            line = self.ser.readline().decode(errors='ignore').strip()
            self.get_logger().info(f"◀ Arduino 응답2: '{line}'")
            if line != 'DUMP_DONE':
                response.success = False
                response.message = f"DUMP_DONE ACK 미수신 (got '{line}')"
                return response

            # 4) 모두 정상 수신
            response.success = True
            response.message = 'Dump 완료 (ACK 모두 수신)'
            self.get_logger().info("✅ Dump 동작 완료 ACK 확인")
            return response

        except Exception as e:
            self.get_logger().error(f"Dump 명령 전송 실패: {e}")
            response.success = False
            response.message = 'Dump 명령 전송 중 예외 발생'
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
