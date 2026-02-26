import unittest
from unittest.mock import MagicMock
import sys
import os

# 1. Mock modules hierarchy
mock_rclpy = MagicMock()
sys.modules['rclpy'] = mock_rclpy
sys.modules['rclpy.node'] = MagicMock()
sys.modules['rclpy.qos'] = MagicMock()
sys.modules['sensor_msgs'] = MagicMock()
sys.modules['sensor_msgs.msg'] = MagicMock()
sys.modules['cv_bridge'] = MagicMock()
sys.modules['cv2'] = MagicMock()

# Define a real class for Node to avoid MagicMock base class issues
class MockNode:
    def __init__(self, node_name, **kwargs):
        self.node_name = node_name
        self.get_logger = MagicMock()
        self.declare_parameter = MagicMock()
        self.get_parameter = MagicMock()
        self.create_publisher = MagicMock()
        self.create_timer = MagicMock()
        self.create_subscription = MagicMock()
        self.get_clock = MagicMock()
        self.destroy_node = MagicMock()

        # Setup default return values for logger
        self.logger = MagicMock()
        self.get_logger.return_value = self.logger

        # Setup parameter return values
        def get_parameter_side_effect(name):
            param_mock = MagicMock()
            value_mock = MagicMock()
            if name == 'device_path':
                value_mock.string_value = '/dev/video0'
            elif name == 'width':
                value_mock.integer_value = 480
            elif name == 'height':
                value_mock.integer_value = 360
            elif name == 'fps':
                value_mock.double_value = 20.0

            param_mock.get_parameter_value.return_value = value_mock
            return param_mock

        self.get_parameter.side_effect = get_parameter_side_effect

# Inject our MockNode into the sys.modules
sys.modules['rclpy.node'].Node = MockNode

# 2. Add package to path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../')))

# Import the node class
try:
    from camera_controll.rover_camera_node import RoverCameraNode
except ImportError:
    # Adjust path if needed
    sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../')))
    from camera_controll.camera_controll.rover_camera_node import RoverCameraNode

class TestRoverCameraNode(unittest.TestCase):
    def setUp(self):
        # Reset cv2 mock to default "success" state for every test
        sys.modules['cv2'].VideoCapture.return_value.isOpened.return_value = True

    def test_initialization_logs_config(self):
        """Test that the node logs its configuration on startup."""
        node = RoverCameraNode()

        # Check if info logging was called
        logger = node.get_logger.return_value

        if not logger.info.called:
             self.fail("Logger info was not called. Configuration log missing.")

        calls = logger.info.call_args_list
        found_config_log = False
        for call in calls:
            msg = call[0][0]
            # Flexible matching for float representation (20 or 20.0)
            if "480x360" in msg and ("20fps" in msg or "20.0fps" in msg):
                found_config_log = True
                break

        self.assertTrue(found_config_log, "Configuration log message not found")

    def test_parameters_declared(self):
        """Test that ROS parameters are declared."""
        node = RoverCameraNode()

        if not node.declare_parameter.called:
             self.fail("Parameters were not declared.")

        calls = node.declare_parameter.call_args_list
        declared_params = [call[0][0] for call in calls]

        self.assertIn('device_path', declared_params)
        self.assertIn('width', declared_params)
        self.assertIn('height', declared_params)
        self.assertIn('fps', declared_params)

    def test_camera_failure_logs_error(self):
        """Test that if camera fails to open, an error is logged."""
        # Setup cv2 mock to say camera FAILED to open
        sys.modules['cv2'].VideoCapture.return_value.isOpened.return_value = False

        node = RoverCameraNode()
        logger = node.get_logger.return_value

        self.assertTrue(logger.error.called, "Logger error should be called when camera fails to open")

        # Verify message
        args, _ = logger.error.call_args
        self.assertIn("Failed to open camera", args[0])

if __name__ == '__main__':
    unittest.main()
