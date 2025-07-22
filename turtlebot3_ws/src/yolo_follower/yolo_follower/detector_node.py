#!/usr/bin/env python3
import json
import os
import cv2

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from ultralytics import YOLO

class DetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        self.declare_parameters('', [
            ('model_path', 'yolov8n.pt'),
            ('image_topic', '/camera/image_raw'),
            ('bbox_topic', '/yolo_follower/bboxes'),
            ('debug_image_topic', '/yolo_follower/debug_image'),
            ('enable_debug_image', True),
            ('target_class', 'person'),
            ('conf_thres', 0.5),
            ('iou_thres', 0.45),
        ])

        model_path = self.get_parameter('model_path').value
        self.target_class = self.get_parameter('target_class').value
        self.conf_thres = self.get_parameter('conf_thres').value
        self.iou_thres = self.get_parameter('iou_thres').value
        self.enable_debug = self.get_parameter('enable_debug_image').value

        if not os.path.isabs(model_path):
            from ament_index_python.packages import get_package_share_directory
            pkg_share = get_package_share_directory('yolo_follower')
            model_path = os.path.join(pkg_share, 'models', model_path)

        self.model = YOLO(model_path)
        self.model.conf = self.conf_thres
        self.model.iou = self.iou_thres

        self.bridge = CvBridge()
        image_topic = self.get_parameter('image_topic').value
        bbox_topic = self.get_parameter('bbox_topic').value
        debug_topic = self.get_parameter('debug_image_topic').value

        self.create_subscription(Image, image_topic, self.image_cb, 10)
        self.pub_bbox = self.create_publisher(String, bbox_topic, 10)
        self.pub_debug = self.create_publisher(Image, debug_topic, 10) if self.enable_debug else None

        self.get_logger().info(f"DetectorNode started with model {model_path}")

    def image_cb(self, msg: Image):
        cv_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        h, w, _ = cv_img.shape

        results = self.model(cv_img, verbose=False)
        boxes = results[0].boxes

        det_list = []
        for box, cls_id, conf in zip(boxes.xyxy, boxes.cls, boxes.conf):
            cls_name = self.model.names[int(cls_id)]
            if cls_name != self.target_class:
                continue
            x1, y1, x2, y2 = box.cpu().numpy().astype(int)
            area = (x2 - x1) * (y2 - y1)
            det_list.append({
                'cls': cls_name,
                'conf': float(conf),
                'bbox': [int(x1), int(y1), int(x2), int(y2)],
                'area': float(area),
                'img_w': w,
                'img_h': h,
            })
            if self.enable_debug:
                cv2.rectangle(cv_img, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(cv_img, f"{cls_name} {conf:.2f}", (x1, max(0, y1 - 5)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        self.pub_bbox.publish(String(data=json.dumps(det_list)))

        if self.enable_debug and self.pub_debug:
            dbg_msg = self.bridge.cv2_to_imgmsg(cv_img, encoding='bgr8')
            self.pub_debug.publish(dbg_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()