# face_rec.py
from insightface.app import FaceAnalysis
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import time
from .util import resolve_project_path


class FaceRecognition(Node):
    def __init__(self):
        super().__init__('face_rec')


        self.sub = self.create_subscription(Image, "video_frames", self.listener_callback, 1)
        self.intruder_pub = self.create_publisher(Image, 'intruder_alert', 1)
        self.frame_pub = self.create_publisher(Image, 'live_frame', 1)


        self.DETECTION_INTERVAL = 5.0
        self.EMBEDDINGS_PATH = resolve_project_path('data/face_embeddings.npy')
        self.EMBEDDING_THRESHOLD = 0.5
        
        self.br = CvBridge()
        self.last_alert_time = 0.0
        self.last_intruder_frame = None
        self.last_live_sent = 0.0

        self.app = FaceAnalysis(name='antelopev2',root=resolve_project_path('insightface').as_posix(),  allowed_modules=['detection', 'recognition'])
        self.app.prepare(ctx_id=-1)   # set 0 if you have GPU
        self.reference_embeddings = np.load(self.EMBEDDINGS_PATH)



        

    def listener_callback(self, msg: Image):
        frame = self.br.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        # self.get_logger().info("Received frame")
        faces = self.app.get(frame)
        intruder_now = False


        if faces:
            for f in faces:
                label, score = self.recognize_face(f.embedding, self.reference_embeddings)
                bbox = f.bbox.astype(int)
                color = (0, 255, 0) if label != "Unknown" else (0, 0, 255)
                cv2.rectangle(frame, (bbox[0], bbox[1]), (bbox[2], bbox[3]), color, 2)
                cv2.putText(frame, f"{label} {score:.2f}", (bbox[0]+5, bbox[1]-10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)

                
                
                # Track any intruder
                if label == "Unknown":
                    intruder_now = True



        final_frame = self.br.cv2_to_imgmsg(frame, encoding="bgr8")
        final_frame.header = msg.header
        self.frame_pub.publish(final_frame)
        # self.get_logger().info("intruder frame")
        # Live preview (throttled)
        now = time.time()
 

        # Intruder event (rate-limited)
        if intruder_now and (now - self.last_alert_time > self.DETECTION_INTERVAL):
            self.last_alert_time = now
            self.last_intruder_frame = frame.copy()

            # Publish to ROS for local tools
            ros_img = self.br.cv2_to_imgmsg(self.last_intruder_frame, encoding="bgr8")
            self.intruder_pub.publish(ros_img)



        # Optional local debug window
        cv2.imshow('Face Recognition', frame)
        cv2.waitKey(1)
        

    def publisher_callback(self):
        if self.INTRUDER_DETECTED and self.last_intruder_frame is not None:
            img_msg = self.br.cv2_to_imgmsg(
                self.last_intruder_frame, encoding="bgr8")
            self.intruder_pub.publish(img_msg)
            self.INTRUDER_DETECTED = False
            self.last_intruder_frame = None

    def calculate_similarity(self, e1, e2):
        return float(np.dot(e1, e2) / (np.linalg.norm(e1) * np.linalg.norm(e2)))

    def recognize_face(self, face_embedding, reference_embeddings):
        sims = [self.calculate_similarity(face_embedding, emb) for emb in reference_embeddings]
        best_idx = int(np.argmax(sims))
        best_score = float(sims[best_idx])
        label = "Hatim" if best_score > self.EMBEDDING_THRESHOLD else "Unknown"
        return label, best_score



def main():
    rclpy.init()
    node = FaceRecognition()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        # cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
