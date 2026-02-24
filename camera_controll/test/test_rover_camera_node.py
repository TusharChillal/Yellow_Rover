import unittest
from unittest.mock import MagicMock, patch
import sys

# MOCKING BEFORE IMPORT
# We need to mock rclpy and cv2 because they might not be installed or we want to isolate
mock_rclpy = MagicMock()
sys.modules['rclpy'] = mock_rclpy
sys.modules['rclpy.qos'] = MagicMock()

# Mock Node class
class MockNode:
    def __init__(self, name):
        self.name = name
        self.logger = MagicMock()
        self.params = {}

    def get_logger(self):
        return self.logger

    def create_publisher(self, msg_type, topic, qos):
        return MagicMock()

    def create_timer(self, period, callback):
        return MagicMock()

    def declare_parameter(self, name, value):
        self.params[name] = value

    def get_parameter(self, name):
        val = self.params.get(name)
        param_obj = MagicMock()
        if isinstance(val, int):
            param_obj.get_parameter_value.return_value.integer_value = val
        elif isinstance(val, str):
            param_obj.get_parameter_value.return_value.string_value = val
        return param_obj

    def destroy_node(self):
        pass

sys.modules['rclpy.node'] = MagicMock()
sys.modules['rclpy.node'].Node = MockNode

sys.modules['cv2'] = MagicMock()
sys.modules['cv_bridge'] = MagicMock()
sys.modules['sensor_msgs'] = MagicMock()
sys.modules['sensor_msgs.msg'] = MagicMock()

# Import after mocking
from camera_controll.rover_camera_node import RoverCameraNode

class TestRoverCameraNode(unittest.TestCase):
    @patch('cv2.VideoCapture')
    def test_init_success(self, mock_video_capture):
        # Setup mock camera
        mock_cap = MagicMock()
        mock_cap.isOpened.return_value = True
        mock_video_capture.return_value = mock_cap

        node = RoverCameraNode()

        # Verify startup log
        # We check if info was called with a string containing specific substrings
        # because the exact string might change slightly
        calls = [args[0] for args, _ in node.get_logger().info.call_args_list]
        self.assertTrue(any("Starting RoverCameraNode" in call for call in calls))
        self.assertTrue(any("Camera opened successfully" in call for call in calls))

    @patch('cv2.VideoCapture')
    def test_init_failure(self, mock_video_capture):
        # Setup mock camera failure
        mock_cap = MagicMock()
        mock_cap.isOpened.return_value = False
        mock_video_capture.return_value = mock_cap

        # Initialize node
        node = RoverCameraNode()

        # Check if error logged
        calls = [args[0] for args, _ in node.get_logger().error.call_args_list]
        self.assertTrue(any("Failed to open camera device" in call for call in calls))

if __name__ == '__main__':
    unittest.main()
