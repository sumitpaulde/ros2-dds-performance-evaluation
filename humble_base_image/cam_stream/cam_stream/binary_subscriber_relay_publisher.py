import rclpy 
from rclpy.node import Node 
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from hare_robot_interfaces.msg import Binaryfile 



 
class BinarySubscriberRelayPublisher(Node):

  def __init__(self):
    super().__init__('binary_subscriber_relay_publisher')
    # self.reliability_parameter = self.declare_parameter("reliability", "RELIABLE")
    # self.reliability_input = self.get_parameter("reliability").value
    # self.durability_parameter = self.declare_parameter("durability", "VOLATILE")
    # self.durability_input = self.get_parameter("durability").value
    # qos_profile_input =  self.get_qos_profile_settings(self.reliability_input, self.durability_input)  
    # self.subscription = self.create_subscription(Binaryfile, "binary_image", self.cam_image_listener_callback, qos_profile=qos_profile_input)
    # self.image_relay_publisher = self.create_publisher( Binaryfile, "binary_image_relay", qos_profile=qos_profile_input)
    self.subscription = self.create_subscription(Binaryfile, "binary_image",self.cam_image_listener_callback, 1)
    self.image_relay_publisher = self.create_publisher( Binaryfile, "binary_image_relay", 1)
    #self.timer = self.create_timer(0.0000001, self.relay_publisher_callback)
    self.get_logger().info("Binary subscriber realy publisher started")
    
  def cam_image_listener_callback(self, data):
    self.get_logger().info('Receiving binary file')
    self.realy_publisher_callback(data)
    

  def realy_publisher_callback(self, data):
    relay_msg = Binaryfile()
    relay_msg.im = data.im
    relay_msg.publish_time = data.publish_time
    self.get_logger().info('Publishing binary file')
    self.image_relay_publisher.publish(relay_msg)

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
    node = BinarySubscriberRelayPublisher()
    rclpy.spin(node)
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()