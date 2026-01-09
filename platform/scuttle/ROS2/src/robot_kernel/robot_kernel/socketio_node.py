import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import socketio
import os
from std_msgs.msg import String
from geometry_msgs.msg import Twist


class socketio_node(Node):
    def __init__(self):
        super().__init__('socketio_node')
        self.image_sub = None
        self.remote_control_pub = self.create_publisher(
            String, 'robot_mode', 1)
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.br = CvBridge()

        self.io = socketio.Client(reconnection=True, reconnection_delay=0)
        self.serialNo = os.getenv("serialNo")
        self.robotKey = os.getenv("robotKey")
        self.serverUrl = os.getenv("serverUrl")

        self.last_status_emit = 0
        self.last_status_value = None

        self.io.connect(
            self.serverUrl,
            auth={'serialNo': self.serialNo, 'robotKey': self.robotKey},
            wait=False,
        )
        self.events()

    def events(self):
        @self.io.event
        def connect():
            print("Connected to server.")

        @self.io.event
        def disconnect():
            print("Disconnected from server.")

        def controlMode(data):
            print(data)
            mode = data.get("mode")
            if mode:
                msg = String()
                msg.data = mode
                self.remote_control_pub.publish(msg)

            if mode == 'manual':
                twist = data.get("twist")
                if twist:
                    msg = Twist()
                    msg.linear.x = float(twist.get('x'))
                    msg.angular.z = float(twist.get('z'))
                    self.timer_callback(msg)

        def on_server_signal(data):
            mode = data
            print("revieved Mode is: ", data)
            if mode:
                if mode == 'rec_frames':
                    if self.image_sub is not None:
                        self.destroy_subscription(self.image_sub)
                        self.image_sub = None
                    self.image_sub = self.create_subscription(
                        Image, "live_frame", self.listiner_callback, 1)

                elif mode == 'live_frame':
                    if self.image_sub is not None:
                        self.destroy_subscription(self.image_sub)
                        self.image_sub = None
                    self.image_sub = self.create_subscription(
                        Image, "video_frames", self.listiner_callback, 1)

        self.io.on("robot:controlMode", controlMode)
        self.io.on("robot:videoMode", on_server_signal)

    def timer_callback(self, twist: Twist):
        self.cmd_vel_publisher.publish(twist)

    def listiner_callback(self, data: Image):
        if True:
            try:
                recieved_frame = self.br.imgmsg_to_cv2(
                    data, desired_encoding="bgr8")
                _, buf = cv2.imencode('.jpg', recieved_frame)
                self.io.emit('robot:frame', buf.tobytes())
            except Exception:
                pass


def main():
    try:
        rclpy.init()
        node = socketio_node()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        sys.exit(0)


if __name__ == '__main__':
    main()
