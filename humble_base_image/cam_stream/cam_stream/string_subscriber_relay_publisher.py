import rclpy 
from rclpy.node import Node 
from hare_robot_interfaces.msg import StringTimestamp
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy




 
class StringSubscriberRelayPublisher(Node):

  def __init__(self):
    super().__init__('string_subscriber_relay_publisher')
    #self.timer = self.create_timer(0.0000001, self.relay_publisher_callback)
    self.get_logger().info("Sample String subscriber realy publisher node started")
    self.subscription = self.create_subscription(StringTimestamp, "string_data", self.sample_string_listener_callback, 1)
    self.sample_string_relay_publisher = self.create_publisher( StringTimestamp, "string_data_relay", 1)

  def sample_string_listener_callback(self, data):
    self.get_logger().info('String data is receiving')
    self.realy_publisher_callback(data)
    

  def realy_publisher_callback(self, data):
    relay_msg =   StringTimestamp()
    relay_msg.input = data.input
    relay_msg.publish_time = data.publish_time
    self.get_logger().info('Publishing Sample String relay')
    self.sample_string_relay_publisher.publish(relay_msg)


    
def main(args=None):
    rclpy.init(args= args)
    node = StringSubscriberRelayPublisher()
    rclpy.spin(node)
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()