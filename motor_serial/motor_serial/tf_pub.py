import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry

from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

import math  # Using math for sin/cos


class StaticAndOdomTFPublisher(Node):
    """
    - Publishes static transforms from base_link to the lidar frames.
    - Publishes static transform base_link -> base_footprint (for nodes that expect base_footprint).
    - Publishes dynamic TF from odom -> base_link using /odom messages.
    """

    def __init__(self):
        super().__init__('tf_publisher_node')

        # --- Frames (keep these consistent with slam_toolbox params) ---
        self.base_frame = 'base_link'
        self.base_footprint_frame = 'base_footprint'
        self.lidar_link_frame = 'lidar_lidar_link'
        self.laser_frame = 'laser_frame'
        self.odom_frame = 'odom'

        # 1) Static TF broadcaster (base_link -> lidar frames, base_link -> base_footprint)
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)

        # Lidar Position (x, y, z) relative to base_link
        translation_x = 0.1
        translation_y = 0.0
        translation_z = 0.088  # Base height (0.078) + offset (0.01)

        # Lidar Orientation (Roll, Pitch, Yaw)
        roll = 0.0
        pitch = 0.0
        yaw = 0.0

        # Convert RPY -> quaternion
        quat = self.eul_to_quat(roll, pitch, yaw)

        # base_link -> lidar_lidar_link
        self.publish_static_transform(
            self.base_frame,
            self.lidar_link_frame,
            translation_x, translation_y, translation_z,
            quat
        )

        # base_link -> laser_frame
        self.publish_static_transform(
            self.base_frame,
            self.laser_frame,
            translation_x, translation_y, translation_z,
            quat
        )

        # base_link -> base_footprint (identity, so both frames exist)
        self.publish_static_transform(
            self.base_frame,
            self.base_footprint_frame,
            0.0, 0.0, 0.0,
            [0.0, 0.0, 0.0, 1.0]  # x, y, z, w (no rotation)
        )

        # 2) Dynamic TF broadcaster (odom -> base_link)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscribe to /odom to publish odom -> base_link TF
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.get_logger().info(
            'TF publisher node started: '
            'static base_link->{lidar_lidar_link, laser_frame, base_footprint} '
            '+ dynamic odom->base_link'
        )

    # ---------- Static TF helpers ----------

    def eul_to_quat(self, roll, pitch, yaw):
        """Converts Euler angles (roll, pitch, yaw) to a Quaternion [x, y, z, w]."""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        qx = cy * cp * sr - sy * sp * cr  # x
        qy = sy * cp * sr + cy * sp * cr  # y
        qz = sy * cp * cr - cy * sp * sr  # z
        qw = cy * cp * cr + sy * sp * sr  # w

        return [qx, qy, qz, qw]

    def publish_static_transform(self, parent_frame, child_frame, x, y, z, quat):
        """Publish a single static transform."""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = parent_frame
        t.child_frame_id = child_frame

        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z

        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        self.tf_static_broadcaster.sendTransform(t)
        self.get_logger().info(f'Published STATIC transform: {parent_frame} -> {child_frame}')

    # ---------- Dynamic TF from /odom ----------

    def odom_callback(self, odom_msg: Odometry):
        """
        Broadcast odom -> base_link TF based on incoming /odom message.
        slam_toolbox expects this TF chain:
            map -> odom -> base_frame -> laser_frame
        """
        t = TransformStamped()
        t.header.stamp = odom_msg.header.stamp
        t.header.frame_id = self.odom_frame      # parent
        t.child_frame_id = self.base_frame       # child

        # Copy pose from Odometry into TF
        t.transform.translation.x = odom_msg.pose.pose.position.x
        t.transform.translation.y = odom_msg.pose.pose.position.y
        t.transform.translation.z = odom_msg.pose.pose.position.z
        t.transform.rotation = odom_msg.pose.pose.orientation

        self.tf_broadcaster.sendTransform(t)
        # self.get_logger().info('Published dynamic TF: odom -> base_link')


def main(args=None):
    rclpy.init(args=args)
    node = StaticAndOdomTFPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
