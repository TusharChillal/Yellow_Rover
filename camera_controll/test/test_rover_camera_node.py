import unittest
from unittest.mock import MagicMock, patch
import sys

# Define Mock classes to simulate ROS 2 parameter behavior
class MockParameterValue:
    def __init__(self, value):
        self.value = value

    @property
    def string_value(self):
        return str(self.value)

    @property
    def integer_value(self):
        return int(self.value)

    @property
    def double_value(self):
        return float(self.value)

class MockParameter:
    def __init__(self, name, value):
        self.name = name
        self.value = value

    def get_parameter_value(self):
        return MockParameterValue(self.value)

# Define a MockNode class to replace rclpy.node.Node
class MockNode:
    def __init__(self, node_name):
        self.node_name = node_name
        self.logger = MagicMock()
        self.parameters = {} # storage for declared parameters

    def create_publisher(self, msg_type, topic, qos_profile):
        return MagicMock()

    def create_subscription(self, msg_type, topic, callback, qos_profile):
        return MagicMock()

    def create_timer(self, timer_period_sec, callback):
        return MagicMock()

    def get_clock(self):
        clock = MagicMock()
        clock.now.return_value.to_msg.return_value = "time"
        return clock

    def get_logger(self):
        return self.logger

    def destroy_node(self):
        pass

    def declare_parameter(self, name, value):
        # Store the default value
        if name not in self.parameters:
            self.parameters[name] = value
        return MockParameter(name, value)

    def get_parameter(self, name):
        if name in self.parameters:
            return MockParameter(name, self.parameters[name])
        return MagicMock()

    def set_parameters(self, params_dict):
        # Helper for tests to override parameters before __init__ runs?
        # No, because __init__ runs immediately.
        # We need a way to pre-populate parameters.
        self.parameters.update(params_dict)

# Mock rclpy and other dependencies BEFORE importing the module under test
sys.modules['rclpy'] = MagicMock()
sys.modules['rclpy.node'] = MagicMock()
sys.modules['rclpy.node'].Node = MockNode # Inject our MockNode class
sys.modules['rclpy.qos'] = MagicMock()
sys.modules['sensor_msgs.msg'] = MagicMock()
sys.modules['cv_bridge'] = MagicMock()
sys.modules['cv2'] = MagicMock()

# Import the module under test
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2

import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from camera_controll.rover_camera_node import RoverCameraNode

class TestRoverCameraNode(unittest.TestCase):
    def setUp(self):
        # Reset mocks before each test
        rclpy.reset_mock()
        cv2.reset_mock()

        # Ensure isOpened returns True by default so tests assume camera works unless specified otherwise
        cv2.VideoCapture.return_value.isOpened.return_value = True

    def test_init_defaults(self):
        """Test that the node initializes with default values."""
        # Instantiate the node
        node = RoverCameraNode()

        # Verify default parameters were declared
        # We can't easily access the node instance inside RoverCameraNode unless we capture it,
        # but RoverCameraNode IS the instance.
        self.assertEqual(node.parameters['device_path'], '/dev/video0')
        self.assertEqual(node.parameters['width'], 480)
        self.assertEqual(node.parameters['height'], 360)
        self.assertEqual(node.parameters['fps'], 20.0)

        # Verify cv2.VideoCapture was called with default /dev/video0
        cv2.VideoCapture.assert_called_with('/dev/video0')

        # Verify logger info was called
        node.get_logger().info.assert_any_call("Starting camera node on /dev/video0 at 480x360@20.0fps")

        # Verify settings
        cap_instance = cv2.VideoCapture.return_value
        self.assertEqual(cap_instance.set.call_count, 4)

    def test_init_custom_parameters(self):
        """Test that the node respects custom parameters."""
        # We need to inject parameters BEFORE __init__ calls declare_parameter/get_parameter.
        # But declare_parameter uses the passed value as default.
        # get_parameter should return the value from the parameter server (which we simulate with self.parameters).
        # However, typically declare_parameter(name, default_value) registers the parameter.
        # Then get_parameter(name) retrieves it.
        # In a real node, parameter overrides happen at launch.
        # Here, we need to modify how declare_parameter works in our MockNode to prefer pre-existing values.

        # Monkey patch MockNode to pre-populate parameters for the next instance
        original_init = MockNode.__init__
        def pre_populated_init(self, node_name):
            original_init(self, node_name)
            self.parameters['device_path'] = '/dev/video1'
            self.parameters['width'] = 1920
            self.parameters['height'] = 1080
            self.parameters['fps'] = 60.0

        with patch.object(MockNode, '__init__', pre_populated_init):
            node = RoverCameraNode()

            # Verify cv2.VideoCapture was called with custom path
            cv2.VideoCapture.assert_called_with('/dev/video1')

            # Verify logger info was called with custom values
            node.get_logger().info.assert_any_call("Starting camera node on /dev/video1 at 1920x1080@60.0fps")

            # Verify settings used new values
            cap_instance = cv2.VideoCapture.return_value
            # We can't check arguments directly easily because they are mocked constants
            # But we can verify set was called.
            self.assertEqual(cap_instance.set.call_count, 4)

    def test_camera_failure(self):
        """Test that the node logs an error if the camera fails to open."""
        # Mock isOpened to return False
        cv2.VideoCapture.return_value.isOpened.return_value = False

        node = RoverCameraNode()

        # Verify error log
        node.get_logger().error.assert_called_with("Failed to open camera device: /dev/video0")

        # Verify set was NOT called
        cap_instance = cv2.VideoCapture.return_value
        cap_instance.set.assert_not_called()

if __name__ == '__main__':
    unittest.main()
