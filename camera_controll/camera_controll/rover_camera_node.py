import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage # Change import
from cv_bridge import CvBridge
import cv2
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class RoverCameraNode(Node):
    def __init__(self):
        super().__init__('rover1_camera_node')

        # Declare parameters with default values
        self.declare_parameter('device_path', '/dev/video0')
        self.declare_parameter('width', 480)
        self.declare_parameter('height', 360)
        self.declare_parameter('fps', 20.0)

        # Get parameter values
        device_path = self.get_parameter('device_path').get_parameter_value().string_value
        width = self.get_parameter('width').get_parameter_value().integer_value
        height = self.get_parameter('height').get_parameter_value().integer_value
        fps = self.get_parameter('fps').get_parameter_value().double_value

        # QoS: Best Effort is better for lossy WiFi video
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publisher for COMPRESSED camera feed
        # Note the topic name change and message type change
        self.publisher_ = self.create_publisher(CompressedImage, '/rover/camera/image_raw/compressed', qos_profile)

        self.cap = cv2.VideoCapture(device_path)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))

        # Log configuration on startup
        if self.cap.isOpened():
            self.get_logger().info(
                f'Camera initialized: {device_path} at {width}x{height} @ {fps}fps'
            )
        else:
            self.get_logger().error(f'Failed to open camera device: {device_path}')

        self.bridge = CvBridge()
        self.timer = self.create_timer(1.0 / fps, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Failed to read frame from camera', throttle_duration_sec=5.0)
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
