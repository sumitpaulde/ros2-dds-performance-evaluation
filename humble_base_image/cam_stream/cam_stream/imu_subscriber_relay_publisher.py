import rclpy 
from rclpy.node import Node 
from hare_robot_interfaces.msg import Imu
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy




 
class ImuSubscriberRelayPublisher(Node):

  def __init__(self):
    super().__init__('imu_subscriber_relay_publisher')
    self.subscription = self.create_subscription(Imu, "imu_data", self.imu_data_listener_callback, 1)
    self.sample_string_relay_publisher = self.create_publisher( Imu, "imu_data_relay", 1)
    
  def imu_data_listener_callback(self, data):
    self.get_logger().info('Imu data is receiving')
    self.realy_publisher_callback(data)
    

  def realy_publisher_callback(self, data):
    relay_msg = Imu()
    relay_msg.header = data.header
    relay_msg.orientation_covariance = data.orientation_covariance
    relay_msg.angular_velocity_covariance = data.angular_velocity_covariance
    relay_msg.publish_time = data.publish_time
    self.get_logger().info('Publishing imu data relay')
    self.sample_string_relay_publisher.publish(relay_msg)

    
def main(args=None):
    rclpy.init(args= args)
    node = ImuSubscriberRelayPublisher()
    rclpy.spin(node)
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()