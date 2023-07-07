import rclpy
import cv2
from rclpy.node import Node 
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge 



class CamPublisher(Node):
    
    def __init__(self):
        super().__init__("cam_publisher")
        self.image_publisher = self.create_publisher(Image, "cam_image", 10)
        self.timer = self.create_timer(0.0000001, self.publish_image_callback)
        self.get_logger().info(" cam_publisher node started")
        self.image_capture = cv2.VideoCapture(4)
        self.bridge = CvBridge()
        
    def publish_image_callback(self):
        
        # if not self.image_capture.isOpened():
        #     raise IOError("Cannot open webcam")

        while True:
            Has_return, frame = self.image_capture.read()
            if Has_return == True:
                self.image_publisher.publish(self.bridge.cv2_to_imgmsg(frame))
                self.get_logger().info('Cam Video Publishing')
        
            
            
def main(args=None):
# this line is required to initialize the ros communication.
    rclpy.init(args= args)
    node = CamPublisher()
    rclpy.spin(node)
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()