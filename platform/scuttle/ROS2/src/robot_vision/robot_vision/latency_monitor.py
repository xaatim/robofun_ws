import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

class LatencyMonitor(Node):
    def __init__(self):
        super().__init__('latency_monitor')
        
        # 1. Measure Camera -> Network -> Here
        self.raw_sub = self.create_subscription(
            Image, 
            'video_frames', 
            self.raw_callback, 
            10)

        # 2. Measure Camera -> FaceRec (Processing) -> Network -> Here
        self.processed_sub = self.create_subscription(
            Image, 
            'live_frame', 
            self.processed_callback, 
            10)

    def calculate_delay(self, msg, source_name):
        # Current time (Now)
        now = self.get_clock().now()
        
        # Message creation time (Header)
        # Note: We must convert both to nanoseconds to do math
        now_nanos = now.nanoseconds
        msg_nanos = (msg.header.stamp.sec * 1_000_000_000) + msg.header.stamp.nanosec
        
        # Calculate difference in milliseconds
        latency_ms = (now_nanos - msg_nanos) / 1_000_000.0
        
        # Log it clearly for your FYP data
        if latency_ms > 1000:
            self.get_logger().warn(f'High Latency [{source_name}]: {latency_ms:.2f} ms')
        else:
            self.get_logger().info(f'Latency [{source_name}]: {latency_ms:.2f} ms')

    def raw_callback(self, msg):
        self.calculate_delay(msg, "Camera Raw")

    def processed_callback(self, msg):
        self.calculate_delay(msg, "FaceRec Stream")

def main(args=None):
    rclpy.init(args=args)
    monitor = LatencyMonitor()
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        pass
    finally:
        monitor.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()