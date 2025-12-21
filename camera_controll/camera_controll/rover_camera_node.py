import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage # Change import
from cv_bridge import CvBridge
import cv2
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class RoverCameraNode(Node):
    def __init__(self):
        super().__init__('rover1_camera_node')

        # QoS: Best Effort is better for lossy WiFi video
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publisher for COMPRESSED camera feed
        # Note the topic name change and message type change
        self.publisher_ = self.create_publisher(CompressedImage, '/rover/camera/image_raw/compressed', qos_profile)

        self.cap = cv2.VideoCapture('/dev/video0')
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 480)  
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 360)  
        self.cap.set(cv2.CAP_PROP_FPS, 20)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))

        self.bridge = CvBridge()
        self.timer = self.create_timer(1.0 / 20.0, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        # COMPRESS the image to JPEG before sending
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.format = "jpeg"
        
        # cv2.imencode returns (success, encoded_image)
        success, encoded_image = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 50]) # 50% Quality
        
        if success:
            msg.data = encoded_image.tobytes()
            self.publisher_.publish(msg)

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = RoverCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()