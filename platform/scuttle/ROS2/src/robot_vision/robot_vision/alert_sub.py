import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import os
from datetime import datetime
from email.message import EmailMessage
import smtplib
import ssl
import os
import os
from dotenv import load_dotenv
load_dotenv() 

class alert_sub(Node):
    def __init__(self):
        super().__init__('alert_sub')
        self.image_sub = self.create_subscription(
            Image, "intruder_alert", self.listiner_callback, 1)
        self.br = CvBridge()
        self.EMAIL_SENDER = os.getenv("EMAIL_SENDER")
        self.EMAIL_PASSWORD = os.getenv("EMAIL_PASSWORD")
        self.EMAIL_RECEIVER = os.getenv("EMAIL_RECEIVER")

    def listiner_callback(self, data: Image):
        # self.get_logger().info("receiving video frames")
        recieved_frame = self.br.imgmsg_to_cv2(data, desired_encoding="bgr8")
        # cv2.imshow('no_name', recieved_frame)
        image_path = self.save_alert_image(recieved_frame)
        # cv2.waitKey(1)
        self.send_mail(image_path)
        

    def save_alert_image(self, frame):
        """Save intruder image and queue for notification"""
        alert_dir = os.path.join(
            os.path.dirname(__file__),
            '..',
            'alert_images'
        )
        os.makedirs(alert_dir, exist_ok=True)

        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        image_path = os.path.join(alert_dir, f"intruder_{timestamp}.jpg")

        cv2.imwrite(image_path, frame)
        return image_path    
    
    def send_mail(self,image_path):

      context = ssl.create_default_context()
      try:
          msg = EmailMessage()
          msg["Subject"] = "Intruder Alert!"
          msg["From"] = self.EMAIL_SENDER
          msg["To"] = self.EMAIL_RECEIVER
          msg.set_content("An unknown person was detected while you were away. Image attached.")

          with open(image_path, "rb") as f:
              img_data = f.read()
              msg.add_attachment(img_data, maintype='image', subtype='jpeg', filename="intruder.jpg")

          with smtplib.SMTP_SSL("smtp.gmail.com", 465, context=context) as smtp:
              smtp.login(self.EMAIL_SENDER, self.EMAIL_PASSWORD) # type: ignore
              smtp.send_message(msg)


          # Delete image after successful email
          os.remove(image_path)
          print(f"Deleted alert image: {image_path}")

      except Exception as e:
          print(f"Error sending email: {e}")

def main():

    try:
        rclpy.init()
        alerter = alert_sub()

        rclpy.spin(alerter)
        alerter.destroy_node()
        rclpy.shutdown()

    except KeyboardInterrupt:
        print('exit 0 ')


if __name__ == '__main__':
    main()
