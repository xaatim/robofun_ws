#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import numpy as np

class WebTeleopNode(Node):
    """
    A ROS2 node that captures video from a webcam, publishes it as a compressed image stream,
    and subscribes to Twist commands to print received movements.
    """
    def __init__(self):
        super().__init__('web_teleop_node')

        # --- Parameters ---
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('publish_rate', 30.0)
        
        camera_index = self.get_parameter('camera_index').get_parameter_value().integer_value
        publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value

        # --- Video Capture ---
        self.cap = cv2.VideoCapture(camera_index)
        if not self.cap.isOpened():
            self.get_logger().error(f"Could not open camera at index {camera_index}")
            rclpy.shutdown()
            return

        self.get_logger().info(f"Successfully opened camera at index {camera_index}")
        self.bridge = CvBridge()

        # --- Publisher for Compressed Image ---
        self.image_publisher = self.create_publisher(
            CompressedImage,
            '/camera/image/compressed',
            10
        )

        # --- Subscriber for Command Velocity ---
        self.cmd_vel_subscriber = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # --- Timer for publishing frames ---
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_frame)

        self.get_logger().info('Web Teleop Node has been started.')
        self.get_logger().info(f'Publishing compressed images to /camera/image/compressed at {publish_rate} Hz.')
        self.get_logger().info('Listening for commands on /cmd_vel.')

    def publish_frame(self):
        """ Captures a frame, compresses it to JPEG, and publishes it. """
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Failed to capture frame from camera.')
            return

        # Create the CompressedImage message
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', frame)[1]).tobytes()

        self.image_publisher.publish(msg)

    def cmd_vel_callback(self, msg: Twist):
        """ Callback function for the /cmd_vel topic. Prints the received command. """
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        command_str = "Received command: "
        if linear_x > 0:      command_str += "Move Forward"
        elif linear_x < 0:    command_str += "Move Backward"
        elif angular_z > 0:   command_str += "Turn Left"
        elif angular_z < 0:   command_str += "Turn Right"
        else:                 command_str += "Stop"
            
        self.get_logger().info(f'{command_str} | Linear X: {linear_x:.2f}, Angular Z: {angular_z:.2f}')

    def destroy_node(self):
        self.get_logger().info("Shutting down node.")
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    web_teleop_node = WebTeleopNode()
    try:
        rclpy.spin(web_teleop_node)
    except KeyboardInterrupt:
        pass
    finally:
        web_teleop_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()