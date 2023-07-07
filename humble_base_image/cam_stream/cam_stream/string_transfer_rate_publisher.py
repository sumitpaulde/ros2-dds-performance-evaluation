import rclpy
from rclpy.node import Node 
from hare_robot_interfaces.msg import StringTimestamp
from std_msgs.msg import String
import datetime
import pandas as pd
import os




class StringTransferRatePublisher(Node):
    
    def __init__(self):
        super().__init__("string_transfer_rate_publisher")
        self.get_logger().info("Sample string publisher node started")
        self.string_input = self.declare_parameter("input")
        self.string_parameter_input = self.get_parameter("input").value
        self.timer_parameter = self.declare_parameter("hz")
        self.timer_parameter_input = self.get_parameter("hz").value
        self.domain_parameter = self.declare_parameter("domain", "different_domain")
        self.domain_parameter_input = self.get_parameter("domain").value
        self.timer = self.create_timer(self.hz_to_second(self.timer_parameter_input), self.publish_string_callback)
        self.data_frame_publisher = 0
        self.home_path = os.path.expanduser('~')
        self.string_publisher = self.create_publisher( StringTimestamp, "string_data", 1)
        self.subscription = self.create_subscription(StringTimestamp, "string_data_relay", self.string_data_listener_callback, 1)


    def publish_string_callback(self):
        self.data_frame_publisher += 1
        msg = StringTimestamp()
        input_string = String()
        input_string.data = self.string_parameter_input
        msg.input = input_string
        msg.publish_time = self.get_current_timestamp()
        self.string_publisher.publish(msg)
        if self.data_frame_publisher == 1000:
            raise SystemExit
        self.get_logger().info("Transfered String: " + input_string.data)

    def string_data_listener_callback(self, data):
        self.get_logger().info("String data receiving")
        if data.input == "x" or data.publish_time ==  1.111:
            raise SystemExit


    
    def get_current_timestamp(self):
        current_time = datetime.datetime.now()
        time_stamp = current_time.timestamp()
        return time_stamp

    
    def hz_to_second(self, hz_input):
        timer_input = float(1 / hz_input)
        return timer_input
    
    
            
            
def main(args=None):
    rclpy.init(args= args)
    node = StringTransferRatePublisher()
    rclpy.spin(node)
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()