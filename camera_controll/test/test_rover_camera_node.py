import unittest
from unittest.mock import MagicMock, patch, call
import sys

# Define a fake base Node class that has the methods we need
class FakeNode:
    def __init__(self, name):
        self.node_name = name
        self.declared_params = {}
        self.params = {
            'device_path': '/dev/video0',
            'width': 480,
            'height': 360,
            'fps': 20.0
        }
        self.logger_mock = MagicMock()
        self.publisher_mock = MagicMock()

    def declare_parameter(self, name, default_value):
        self.declared_params[name] = default_value

    def get_parameter(self, name):
        # Return a mock that has the value
        param_mock = MagicMock()
        if name in self.params:
            val = self.params[name]
        else:
            val = None

        param_mock.get_parameter_value.return_value.string_value = val if isinstance(val, str) else str(val)
        param_mock.get_parameter_value.return_value.integer_value = val if isinstance(val, int) else 0
        param_mock.get_parameter_value.return_value.double_value = val if isinstance(val, float) else 0.0
        return param_mock

    def get_logger(self):
        return self.logger_mock

    def create_publisher(self, msg_type, topic, qos_profile):
        return self.publisher_mock

    def create_timer(self, period, callback):
        return MagicMock()

    def destroy_node(self):
        pass

    def get_clock(self):
        return MagicMock()

# Mock modules before importing the node
sys.modules['rclpy'] = MagicMock()
sys.modules['rclpy.node'] = MagicMock()
sys.modules['rclpy.node'].Node = FakeNode # Inject our fake base class
sys.modules['rclpy.qos'] = MagicMock()
sys.modules['sensor_msgs'] = MagicMock()
sys.modules['sensor_msgs.msg'] = MagicMock()
sys.modules['cv_bridge'] = MagicMock()
sys.modules['cv2'] = MagicMock()

# Now import the node
# Since we injected FakeNode into rclpy.node.Node, RoverCameraNode will inherit from FakeNode
from camera_controll.rover_camera_node import RoverCameraNode
import rclpy
import cv2

class TestRoverCameraNode(unittest.TestCase):
    def test_parameters_and_logging(self):
        # Mock cv2.VideoCapture since it's called in __init__
        cap_mock = MagicMock()
        cv2.VideoCapture.return_value = cap_mock

        # Instantiate the node
        node = RoverCameraNode()

        # 1. Verify declare_parameter calls via our fake storage
        self.assertIn('device_path', node.declared_params)
        self.assertEqual(node.declared_params['device_path'], '/dev/video0')
        self.assertIn('width', node.declared_params)
        self.assertEqual(node.declared_params['width'], 480)
        self.assertIn('height', node.declared_params)
        self.assertEqual(node.declared_params['height'], 360)
        self.assertIn('fps', node.declared_params)
        self.assertEqual(node.declared_params['fps'], 20.0)

        # 2. Verify logging
        # Check if info was called with a string containing our config
        self.assertTrue(node.logger_mock.info.called)
        log_args = node.logger_mock.info.call_args[0][0]
        self.assertIn("Camera Config", log_args)
        self.assertIn("device=/dev/video0", log_args)
        self.assertIn("resolution=480x360", log_args)
        self.assertIn("fps=20.0", log_args)

        # 3. Verify OpenCV initialization uses the retrieved parameters
        cv2.VideoCapture.assert_called_with('/dev/video0')
        cap_mock.set.assert_any_call(cv2.CAP_PROP_FRAME_WIDTH, 480)
        cap_mock.set.assert_any_call(cv2.CAP_PROP_FRAME_HEIGHT, 360)
        cap_mock.set.assert_any_call(cv2.CAP_PROP_FPS, 20.0)

if __name__ == '__main__':
    unittest.main()
