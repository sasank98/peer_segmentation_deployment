import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import torch

class PalletDetectionNode(Node):
    def __init__(self):
        super().__init__('pallet_detection_python')
        self.subscription = self.create_subscription(
            Image,
            '/rgb/image_rect_color',
            self.listener_callback,
            10)
        # subscribes to the left camera image of zed2i,topic obtained from zed2i ROS2 documentation
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu") # Puts the model on the GPU if available
        self.publisher = self.create_publisher(Image, '/modified_image', 10) # publishes to /modified_image topic
        self.bridge = CvBridge()
        
        self.model = YOLO("/home/peer_ws/src/pallet_detection_python/pallet_detection_python/best.pt") # not a good idea, [COMPOSITION]

        self.class_dict = {0: 'ground', 1: 'pallet'}

    def listener_callback(self, msg):
        print('Received an image')
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        result_numpy = self.model(cv_image)[0].cpu().numpy() # convert output to cpu and numpy arrays for opencv processing
        # couldn't test this due to lack of example data, if this doesnt work, try to convert the results to cpu and numpy array
        # results = results.cpu().numpy()
        # given code is faster than converting to numpy array
        for i in range(result_numpy.boxes.cls.shape[0]):
            xyxy = result_numpy.boxes.xyxy[i]
            if result_numpy.boxes.cls[i] == 0:
                cv2.rectangle(cv_image, (int(xyxy[0]), int(xyxy[1])), (int(xyxy[2]), int(xyxy[3])), (0, 255, 0), 2)
                cv2.putText(cv_image, self.class_dict[int(result_numpy.boxes.cls[i])], (int(xyxy[0]), int(xyxy[1])), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            else:
                cv2.rectangle(cv_image, (int(xyxy[0]), int(xyxy[1])), (int(xyxy[2]), int(xyxy[3])), (0, 0, 255), 2)
                cv2.putText(cv_image, self.class_dict[int(result_numpy.boxes.cls[i])], (int(xyxy[0]), int(xyxy[1])), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                
        modified_image_msg = self.bridge.cv2_to_imgmsg(cv_image, 'bgr8')
        self.publisher.publish(modified_image_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PalletDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()