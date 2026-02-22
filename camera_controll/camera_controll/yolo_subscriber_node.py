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
        self.model_loaded = False
        try:
            checkpoint = torch.load('/home/rover/yolo11s.pt', map_location=self.device)
            self.model = checkpoint['model'].float().eval()  # get the actual model
            self.model_loaded = True
            self.get_logger().info('YOLO PyTorch model loaded.')
        except FileNotFoundError:
            self.get_logger().error('Model file not found at /home/rover/yolo11s.pt')
        except Exception as e:
            self.get_logger().error(f'Failed to load YOLO model: {e}')

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        if not self.model_loaded:
            cv2.putText(cv_image, "Model Failed to Load", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            cv2.imshow("YOLO Detection", cv_image)
            cv2.waitKey(1)
            return

        # Preprocess for YOLO
        img = cv2.resize(cv_image, (640, 640))
        img = img[:, :, ::-1].transpose(2, 0, 1)  # BGR->RGB, HWC->CHW
        img = torch.from_numpy(img).unsqueeze(0).float() / 255.0
        img = img.to(self.device)

        # Inference
        with torch.no_grad():
            preds = self.model(img)[0]  # raw YOLO predictions

        # Draw improved boxes with labels
        for det in preds:
            if det[4] > 0.3:  # confidence threshold
                x1, y1, x2, y2 = map(int, det[:4].cpu().numpy())
                conf = float(det[4])

                label = f"{conf:.2f}"
                if len(det) > 5:
                    cls_id = int(det[5])
                    label = f"Class {cls_id}: {label}"

                # Draw box
                cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)

                # Draw label background
                (w, h), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
                cv2.rectangle(cv_image, (x1, y1 - 20), (x1 + w, y1), (0, 255, 0), -1)

                # Draw text
                cv2.putText(cv_image, label, (x1, y1 - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)

        # Add status overlay
        cv2.putText(cv_image, "YOLO Active", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

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
