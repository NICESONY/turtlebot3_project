import rclpy
from rclpy.node import Node
import websocket
#import rel
import json
import time
import threading

from my_custom_msgs.msg import WebInput, WebOutput

WEBSOCKET_SERVER_URL = "ws://192.168.0.102:8080/websocket-endpoint"

class WebSocketRos2Node(Node):
    def __init__(self):
        super().__init__('websocket_ros2_node')
        self.get_logger().info("WebSocket ROS 2 node starting...")

        self.ws = None
        self.connected = False
        self.reconnect_delay_seconds = 5

        self.sub_data = self.create_subscription(WebOutput, '/output_data_web', self.publish_message_to_server, 10)
        self.pub_data = self.create_publisher(WebInput, '/input_data_web', 10)

        self.inputdata = WebInput()

        self.start_websocket_client()

    def start_websocket_client(self):
        self.get_logger().info(f"Attempting to connect to WebSocket server: {WEBSOCKET_SERVER_URL}")
        self.ws = websocket.WebSocketApp(
            WEBSOCKET_SERVER_URL,
            on_open=self.on_open,
            on_message=self.on_message,
            on_error=self.on_error,
            on_close=self.on_close
        )

        self.ws_thread = threading.Thread(target=self.ws_run_forever, daemon=True)
        self.ws_thread.start()

    def ws_run_forever(self):
        while rclpy.ok():
            try:
                self.ws.run_forever()
            except Exception as e:
                self.get_logger().error(f"WebSocket run_forever encountered an error: {e}")
            finally:
                self.get_logger().info("WebSocket run_forever loop ended. Attempting to reconnect...")
                time.sleep(self.reconnect_delay_seconds)

    def on_message(self, ws, message):
        try:
            data = json.loads(message)
            self.inputdata.x = data.get("x", 0.0)
            self.inputdata.y = data.get("y", 0.0)
            self.inputdata.yaw_deg = data.get("yaw_deg", 0.0)
            self.inputdata.id = data.get("id", 0)
            self.inputdata.mod = data.get("mod", 0)

            self.get_logger().info(f"x: {self.inputdata.x}, y: {self.inputdata.y}, yaw_deg: {self.inputdata.yaw_deg}, id: {self.inputdata.id}, mod: {self.inputdata.mod}")

            self.pub_data.publish(self.inputdata)

        except json.JSONDecodeError:
            self.get_logger().warn(f"Received non-JSON message: {message}")
        except Exception as e:
            self.get_logger().error(f"Error processing received message: {e}")

    def on_error(self, ws, error):
        if isinstance(error, websocket._exceptions.WebSocketConnectionClosedException):
            self.get_logger().warn(f"WebSocket connection unexpectedly closed: {error}")
        else:
            self.get_logger().error(f"WebSocket error: {error}")
        self.connected = False

    def on_close(self, ws, close_status_code, close_msg):
        self.get_logger().warn(f"WebSocket connection closed. Status: {close_status_code}, Message: {close_msg}")
        self.connected = False

    def on_open(self, ws):
        self.get_logger().info("WebSocket connection opened successfully.")
        self.connected = True

    def send_message(self, data):
        if self.connected and self.ws:
            try:
                json_data = json.dumps(data)
                self.ws.send(json_data)
                self.get_logger().info(f"Sent to WebSocket server: {json_data}")
            except Exception as e:
                self.get_logger().error(f"Failed to send message over WebSocket: {e}")
                self.connected = False
        else:
            self.get_logger().warn("Not connected to WebSocket server. Message not sent.")

    def publish_message_to_server(self, msg):
        if self.connected:
            message_to_send = {
                "id": msg.id,
                "x": msg.x,
                "y": msg.y,
                "yaw_deg": msg.yaw_deg,
                "mod": msg.mod,
                "batt": msg.batt
            }
            self.send_message(message_to_send)
        else:
            self.get_logger().warn("WebSocket not connected. Skipping message send from timer.")

    def destroy_node(self):
        self.get_logger().info("ROS 2 node shutdown requested. Closing WebSocket connection...")
        if self.ws:
            self.ws.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = WebSocketRos2Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt received. Shutting down node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
