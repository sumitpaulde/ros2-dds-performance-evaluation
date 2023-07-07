import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
import cv2 # OpenCV library
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images

 
class BinayFileSubscriber(Node):

  def __init__(self):
    super().__init__('binary_file_subscriber')
    self.subscription = self.create_subscription(Image, "binary_image", self.cam_image_listener_callback, 10)
    self.bridge = CvBridge()
    
  def cam_image_listener_callback(self, data):
    self.get_logger().info('Receiving Binary Image')
    current_frame = self.bridge.imgmsg_to_cv2(data, desired_encoding='8UC3')
    cv2.imshow("images", current_frame)
    cv2.waitKey(1)
    
def main(args=None):
    rclpy.init(args= args)
    node = BinayFileSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()
