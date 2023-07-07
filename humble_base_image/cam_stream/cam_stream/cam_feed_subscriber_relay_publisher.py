import rclpy 
from rclpy.node import Node 
from hare_robot_interfaces.msg import Binaryfile 



 
class CamFeedSubscriberRelayPublisher(Node):

  def __init__(self):
    super().__init__('cam_feed_subscriber_relay_publisher')
    self.subscription = self.create_subscription(Binaryfile, "cam_feed",self.cam_image_listener_callback, 10)
    self.image_relay_publisher = self.create_publisher( Binaryfile, "cam_relay", 10)
    #self.timer = self.create_timer(0.0000001, self.relay_publisher_callback)
    self.get_logger().info("Cam Feed subscriber realy publisher started")
    
  def cam_image_listener_callback(self, data):
    self.get_logger().info('Receiving camera feed')
    self.realy_publisher_callback(data)
    

  def realy_publisher_callback(self, data):
    relay_msg = Binaryfile()
    relay_msg = data
    self.get_logger().info('Publishing relay camera file')
    self.image_relay_publisher.publish(relay_msg)

    
def main(args=None):
    rclpy.init(args= args)
    node = CamFeedSubscriberRelayPublisher()
    rclpy.spin(node)
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()