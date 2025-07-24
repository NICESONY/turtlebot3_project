#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage # CompressedImage 메시지 타입을 사용합니다.
from std_msgs.msg import Int32 # 다운샘플링 인자 제어를 위해 Int32를 임포트합니다.
from cv_bridge import CvBridge
import cv2
import numpy as np

# --- 설정 상수 (가독성을 위해 상단에 정의합니다) ---
INPUT_COMPRESSED_IMAGE_TOPIC = '/camera/image_raw/compressed' # 압축된 이미지 토픽을 구독합니다.
OUTPUT_COMPRESSED_IMAGE_TOPIC = '/follow_img/compressed'      # 압축된 이미지 토픽으로 발행합니다.
DOWNSAMPLE_FACTOR_CONTROL_TOPIC = '/image_processor/downsample_factor'
# ---------------------------------------------------------------------

class ImageProcessor(Node):
    def __init__(self):
        super().__init__('image_processor')

        self.bridge = CvBridge()
        self._downsample_factor = 2 # 다운샘플링 초기 값 (기본값)
        self.publish_frame_skip = 6 # 6 프레임마다 한 번 발행합니다 (i % 6 == 0).
        self.i = 0 # 프레임 카운터

        # CompressedImage 토픽 구독 설정
        self.image_sub = self.create_subscription(
            CompressedImage,
            INPUT_COMPRESSED_IMAGE_TOPIC,
            self.image_callback,
            10 # QoS history depth
        )

        # 다운샘플링 인자 제어를 위한 구독 설정
        self.downsample_factor_sub = self.create_subscription(
            Int32,
            DOWNSAMPLE_FACTOR_CONTROL_TOPIC,
            self.downsample_factor_callback,
            10 # QoS history depth
        )

        # 처리된 이미지를 CompressedImage 토픽으로 발행 설정
        self.image_pub = self.create_publisher(CompressedImage, OUTPUT_COMPRESSED_IMAGE_TOPIC, 10)

        # 노드 시작 시 정보 로깅 (디버깅에 유용합니다)
        self.get_logger().info(f'📍 ImageProcessor 노드가 시작되었습니다.')
        self.get_logger().info(f'  압축 이미지 구독 중: {INPUT_COMPRESSED_IMAGE_TOPIC}')
        self.get_logger().info(f'  압축 이미지 발행 중: {OUTPUT_COMPRESSED_IMAGE_TOPIC}')
        self.get_logger().info(f'  다운샘플링 인자 제어를 위해 다음 토픽을 수신 중: {DOWNSAMPLE_FACTOR_CONTROL_TOPIC}')
        self.get_logger().info(f'  초기 다운샘플링 인자: {self._downsample_factor}')
        self.get_logger().info(f'  이미지 처리: CompressedImage 수신 -> Raw Image 변환 -> 그레이스케일 -> 해상도 다운샘플링 -> CompressedImage 발행.')


    def downsample_factor_callback(self, msg: Int32):
        """
        다운샘플링 인자 업데이트를 위한 콜백 함수입니다.
        """
        new_factor = msg.data
        if new_factor > 0: # 0보다 큰 유효한 값인지 확인
            if new_factor != self._downsample_factor: # 값이 변경되었을 때만 로깅
                self.get_logger().info(f'📝 다운샘플링 인자 업데이트: {self._downsample_factor} -> {new_factor}.')
                self._downsample_factor = new_factor
        else:
            self.get_logger().warn(f'🚫 유효하지 않은 다운샘플링 인자 수신: {new_factor}. 양의 정수여야 합니다. 현재 인자 유지: {self._downsample_factor}')


    def image_callback(self, msg: CompressedImage): # CompressedImage 타입으로 명시
        """
        CompressedImage 구독을 위한 콜백 함수입니다.
        수신된 압축 이미지를 처리합니다.
        """
        self.i += 1 # 프레임 카운터 증가

        # 1. CompressedImage 메시지를 OpenCV 이미지 (NumPy 배열)로 변환
        # cv_bridge의 imgmsg_to_cv2는 CompressedImage를 직접 지원하지 않으므로,
        # cv2.imdecode를 사용하여 메시지의 data를 디코딩합니다.
        try:
            # CompressedImage의 data는 bytes이므로 numpy.frombuffer로 배열화
            np_arr = np.frombuffer(msg.data, np.uint8)
            # cv2.imdecode를 사용하여 압축된 이미지를 OpenCV 이미지(NumPy 배열)로 디코딩
            # 1은 컬러 이미지로 디코딩 (BGR), 0은 그레이스케일로 디코딩
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            if cv_image is None:
                self.get_logger().error(f'❌ CompressedImage 디코딩 실패: cv2.imdecode가 None을 반환했습니다. 이미지 데이터가 유효한지 확인하세요.')
                return

        except Exception as e:
            self.get_logger().error(f'❌ CompressedImage -> OpenCV 이미지 변환 실패: {e}')
            return

        # 2. 그레이스케일로 변환
        # 디코딩된 컬러 이미지를 그레이스케일로 변환
        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        # self.get_logger().debug('✅ 이미지가 그레이스케일로 변환되었습니다.')


        # 3. 해상도 다운샘플링
        current_downsample_factor = self._downsample_factor
        original_height, original_width = gray_image.shape
        new_width = int(original_width / current_downsample_factor)
        new_height = int(original_height / current_downsample_factor)

        # 최소 1x1 픽셀 크기 보장
        if new_width < 1: new_width = 1
        if new_height < 1: new_height = 1

        downsampled_image = cv2.resize(gray_image, (new_width, new_height),
                                        interpolation=cv2.INTER_AREA)
        # self.get_logger().debug(f'✅ 이미지가 {original_width}x{original_height}에서 {new_width}x{new_height}로 다운샘플링 되었습니다 (인자: {current_downsample_factor}).')


        # 4. 처리된 OpenCV 이미지를 다시 CompressedImage 메시지로 변환하여 발행
        if self.i % self.publish_frame_skip == 0: # 지정된 프레임마다 한 번씩 발행
            try:
                # 그레이스케일 이미지를 JPEG 형식으로 압축합니다.
                # cv2.imencode는 이미지와 파일 확장자를 받아 압축된 바이트 스트림과 성공 여부를 반환합니다.
                # 'mono8' 이미지는 JPEG로 압축하기에 가장 적합한 형식이므로 그대로 사용합니다.
                # JPEG 압축 품질은 두 번째 튜플 인자로 조정할 수 있습니다 (0-100, 기본 95).
                encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 80] # JPEG 품질 80으로 설정
                _result, encoded_image_buffer = cv2.imencode('.jpg', downsampled_image, encode_param)

                if not _result:
                    self.get_logger().error('❌ OpenCV 이미지 JPEG 인코딩 실패.')
                    return

                # 새로운 CompressedImage 메시지 생성
                compressed_msg = CompressedImage()
                compressed_msg.header = msg.header # 원본 헤더 (타임스탬프, 프레임 ID) 복사
                compressed_msg.format = "jpeg" # 압축 형식 지정
                compressed_msg.data = encoded_image_buffer.tobytes() # 인코딩된 바이트 데이터를 할당

                self.image_pub.publish(compressed_msg)
                self.get_logger().debug(f'✅ 프레임 {self.i} 처리된 압축 이미지가 발행되었습니다.')

            except Exception as e:
                self.get_logger().error(f'❌ 처리된 이미지 재압축 및 발행 실패: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = ImageProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()