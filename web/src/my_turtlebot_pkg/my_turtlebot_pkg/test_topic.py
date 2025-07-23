import rclpy
from rclpy.node import Node
from my_custom_msgs.msg import WebInput, WebOutput

class TestTopic(Node):

    def __init__(self):
        super().__init__('test_topic_web')
        self.sub_data = self.create_subscription(WebInput, '/robot_pose_xyyaw', self.callback_inputData, 10)
        self.timer_period = 1.0
        # self.pub_data = self.create_publisher(WebOutput, '/robot_pose_xyyaw', 10)
        # self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.inputdata = WebInput()
        # self.outputdata = WebOutput()
        # self.outputdata.x = 0.0
        # self.outputdata.y = 0.0
        # self.outputdata.yaw_deg = 0.0
        # self.outputdata.id = 5
        # self.outputdata.mod = 2
        # self.outputdata.batt = 87

    def callback_inputData(self, msg):
        self.inputdata.x = msg.x
        self.inputdata.y = msg.y
        self.inputdata.yaw_deg = msg.yaw_deg
        self.inputdata.id = msg.id
        self.inputdata.mod = msg.mod
        self.get_logger().info(f"x: {self.inputdata.x}, y: {self.inputdata.y}, yaw_deg: {self.inputdata.yaw_deg}, id: {self.inputdata.id}, mod: {self.inputdata.mod}")

    # def timer_callback(self):
    #     if self.outputdata.y > -4.5:
    #         self.outputdata.y -= 0.1
    #     else:
    #         self.outputdata.x += 0.1
    #     self.get_logger().info(f"x: {self.outputdata.x}, y: {self.outputdata.y}, yaw_deg: {self.outputdata.yaw_deg}, id: {self.outputdata.id}, mod: {self.outputdata.mod}, batt: {self.outputdata.batt}")
    #     self.pub_data.publish(self.outputdata)

def main(args=None):
    rclpy.init(args=args)
    node = TestTopic()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
