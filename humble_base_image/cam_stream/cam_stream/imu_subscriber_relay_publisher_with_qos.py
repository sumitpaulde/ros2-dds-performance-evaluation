import rclpy 
from rclpy.node import Node 
from hare_robot_interfaces.msg import Imu
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy




 
class ImuSubscriberRelayPublisher(Node):

  def __init__(self):
    super().__init__('imu_subscriber_relay_publisher_with_qos')
    self.reliability_parameter = self.declare_parameter("reliability", "RELIABLE")
    self.reliability_input = self.get_parameter("reliability").value
    self.durability_parameter = self.declare_parameter("durability", "VOLATILE")
    self.durability_input = self.get_parameter("durability").value
    self.get_logger().info("Imu subscriber realy publisher node started")
    self.subscription = self.create_subscription(Imu, "imu_data", self.imu_data_listener_callback, qos_profile=self.get_qos_profile_settings(self.reliability_input, self.durability_input))
    self.sample_string_relay_publisher = self.create_publisher( Imu, "imu_data_relay", qos_profile=self.get_qos_profile_settings(self.reliability_input, self.durability_input))
    
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


  def get_qos_profile_settings(self, reliability_input, durability_input):
        reliability = ''
        durability = ''

        if reliability_input.upper() == "RELIABLE":
              reliability = QoSReliabilityPolicy.RELIABLE
        else :
              reliability = QoSReliabilityPolicy.BEST_EFFORT

        if durability_input.upper() == "VOLATILE":
             durability = QoSDurabilityPolicy.VOLATILE
        else:
             durability = QoSDurabilityPolicy.TRANSIENT_LOCAL               
              
         
        qos_profile = QoSProfile(reliability=reliability, durability=durability, history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1)
        return qos_profile


    
def main(args=None):
    rclpy.init(args= args)
    node = ImuSubscriberRelayPublisher()
    rclpy.spin(node)
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()