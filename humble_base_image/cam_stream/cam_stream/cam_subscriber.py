import rclpy # Python library for ROS 2
import cv2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from hare_robot_interfaces.msg import Binaryfile 
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
 # OpenCV library
 
class CamSubscriber(Node):

  def __init__(self):
    super().__init__('cam_subscriber')
    self.subscription = self.create_subscription(Binaryfile, "cam_feed", self.cam_image_listener_callback, 10)
    self.bridge = CvBridge()
    self.get_logger().info('Subscriber node has been started')
    
  def cam_image_listener_callback(self, data):
    window_title = "Subscriber Feed"
    self.get_logger().info('Receiving cam stream')
    current_frame = self.bridge.imgmsg_to_cv2(data.im)
    cv2.imshow(window_title, current_frame)
    self.get_logger().info(str(current_frame))
    cv2.waitKey(5)

    
def main(args=None):
    rclpy.init(args= args)
    node = CamSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()