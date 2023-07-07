import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Imu # Image is the message type


 
class ImuSubscriber(Node):

  def __init__(self):
    super().__init__('imu_subscriber')
    self.subscription = self.create_subscription(Imu, "/camera/imu", self.imu_listener_callback, 10)
    self.get_logger().info('IMU Subscriber node has been started')
    
  def imu_listener_callback(self, data):
    self.get_logger().info('Receiving IMU data')
    self.get_logger().info((str(data.header.stamp.sec) +"."+ str(data.header.stamp.nanosec) ) )

    
def main(args=None):
    rclpy.init(args= args)
    node = ImuSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()