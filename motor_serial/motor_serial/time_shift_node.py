import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from datetime import datetime
import time

class TimeShiftNode(Node):
    """
    Corrects a constant time offset in laser scan messages.
    
    This node subscribes to the raw LiDAR topic and shifts the timestamp back by the 
    calculated offset to align T_Scan (LiDAR clock) with T_Pi (System clock).
    """
    def __init__(self):
        super().__init__('time_shift_node')
        
        # 1. Define the constant time difference (in seconds)
        # Recalculated offset based on current log data: 1765375414 - 1765375243 = 171.0 seconds.
        self.time_offset_s = 171.0 # **CRITICAL UPDATED OFFSET VALUE**
        
        self.get_logger().info(f"Initialized Time Shift Node with offset: -{self.time_offset_s} seconds.")

        # 2. Publishers and Subscribers
        # We subscribe to the existing /scan topic, as that is where your raw Lidar data is.
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',  # Subscribing to the raw, existing /scan topic
            self.scan_callback,
            10)
            
        # We publish the corrected data to a NEW topic to avoid collisions.
        # You MUST update your SLAM Toolbox parameters to subscribe to '/scan_corrected'.
        self.publisher = self.create_publisher(LaserScan, '/scan_corrected', 10) # New corrected topic

    def scan_callback(self, msg):
        # 1. Convert the original (future) timestamp to nanoseconds
        original_stamp_ns = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        
        # 2. Subtract the offset to bring the timestamp in line with the Pi's clock
        corrected_stamp_ns = original_stamp_ns - int(self.time_offset_s * 10**9)

        # 3. Apply the corrected stamp to the message header
        msg.header.stamp.sec = int(corrected_stamp_ns // 10**9)
        msg.header.stamp.nanosec = int(corrected_stamp_ns % 10**9)

        # 4. Publish the time-corrected scan
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    time_shift_node = TimeShiftNode()
    rclpy.spin(time_shift_node)
    time_shift_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()