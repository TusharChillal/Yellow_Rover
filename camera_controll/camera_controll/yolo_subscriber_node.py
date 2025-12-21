#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import torch
import cv2
import numpy as np

class YoloSubscriberNode(Node):
    def __init__(self):
        super().__init__('yolo_subscriber_node')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',  # your camera topic
            self.image_callback,
            10
        )
        self.bridge = CvBridge()

        # Load PyTorch YOLO model
        self.device = torch.device('cpu')
        checkpoint = torch.load('/home/rover/yolo11s.pt', map_location=self.device)
        self.model = checkpoint['model'].float().eval()  # get the actual model
        self.get_logger().info('YOLO PyTorch model loaded.')

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Preprocess for YOLO
        img = cv2.resize(cv_image, (640, 640))
        img = img[:, :, ::-1].transpose(2, 0, 1)  # BGR->RGB, HWC->CHW
        img = torch.from_numpy(img).unsqueeze(0).float() / 255.0
        img = img.to(self.device)

        # Inference
        with torch.no_grad():
            preds = self.model(img)[0]  # raw YOLO predictions

        # Draw simple boxes (assuming [x1, y1, x2, y2, conf, cls])
        for det in preds:
            if det[4] > 0.3:  # confidence threshold
                x1, y1, x2, y2 = det[:4].cpu().numpy()
                cv2.rectangle(cv_image, (int(x1), int(y1)), (int(x2), int(y2)), (0,255,0), 2)

        cv2.imshow("YOLO Detection", cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = YoloSubscriberNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
