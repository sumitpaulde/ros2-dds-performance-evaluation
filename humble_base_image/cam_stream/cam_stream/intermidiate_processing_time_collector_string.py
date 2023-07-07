import rclpy 
from rclpy.node import Node 
from hare_robot_interfaces.msg import StringTimestamp
from std_msgs.msg import String
import datetime
import pandas as pd
import os
import time


 
class IntermidiateProcessingTimeCollectorString(Node):

  def __init__(self):
    super().__init__('intermidiate_processing_time_collector_string')
    self.data_frame_counter = 0
    self.data_array = []
    self.hz_parameter = self.declare_parameter("hz")
    self.hz_parameter_input = self.get_parameter("hz").value
    self.rate = self.hz_parameter_input
    self.domain_parameter = self.declare_parameter("domain", "different_domain")
    self.domain_parameter_input = self.get_parameter("domain").value
    self.home_path = os.path.expanduser('~')
    self.subscription = self.create_subscription(StringTimestamp, "string_data", self.string_data_listener_callback, 1)
    self.string_relay_publisher = self.create_publisher( StringTimestamp, "string_data_relay", 1)
    self.csv_folder_location = self.home_path + os.sep +"experiment_results"+ os.sep + "interprocessing_time" + os.sep + "string_interprocess" + os.sep + self.domain_parameter_input + os.sep + str(self.hz_parameter_input) + "hz" 
    os.makedirs(self.csv_folder_location, exist_ok=True)
    
    
  def string_data_listener_callback(self, data):
    subscribe_time = float(self.get_current_timestamp())
    self.get_logger().info('Receiving String data')
    relay_msg =   StringTimestamp()
    relay_msg.input = data.input
    relay_msg.publish_time = data.publish_time
    self.string_relay_publisher.publish(relay_msg)
    publish_time = float(self.get_current_timestamp())
    self.get_logger().info("Relay String: " + str(self.hz_parameter_input) + " hz with domain " + str(self.domain_parameter_input) + " "+ str(string_length ) +"_char")
    string_data = String()
    string_data = relay_msg.input
    string_length = str(len((string_data.data)))
    file_name =  "interprocess_string_" + string_length  + "_char_" + str(self.rate)  +"_hz_latency_data.csv"
    file_path = self.csv_folder_location + os.sep  + file_name
    self.data_frame_counter +=1
    intermediate_time_temp  = datetime.datetime.fromtimestamp(float(publish_time))  - datetime.datetime.fromtimestamp(float(subscribe_time))  
    intermediate_time = intermediate_time_temp.total_seconds()
    self.data_array.append(['Frame_Number: '+ str(self.data_frame_counter),  subscribe_time, publish_time, intermediate_time ])
    if self.data_frame_counter == 50 and self.data_frame_counter < 55:
          string_data.data = "x"
          relay_msg.input = string_data
          relay_msg.publish_time = float(1.111)
          self.string_relay_publisher.publish(relay_msg)
          self.write_csv_from_dataframe(self.data_array, file_path )
          self.data_array.clear()
          self.get_logger().info("--------------------------------------------")
          self.get_logger().info("+++++++++++++++++++++++++++++++++++++++++++++")
          time.sleep(5)

          raise SystemExit
    self.get_logger().info("Relay Publishing the String data")
    

  def get_current_timestamp(self):
    current_time = datetime.datetime.now()
    time_stamp = current_time.timestamp()
    return time_stamp

  def write_csv_from_dataframe(self, data_array, file_path_input):
      columns_input=['Frame_Number','Subscribe_Time_Subscriber', 'Publish_Time_From_Subscriber', 'Interprocess_Time']
      
      if os.path.exists(file_path_input) == False :
          data_frame = pd.DataFrame(data_array, columns=columns_input)
      # data_frame.set_index('Frame_Number')
          data_frame.to_csv(file_path_input)
      else :
          data_frame = pd.DataFrame(data_array)
        # data_frame.set_index('Frame_Number')
          data_frame.to_csv(file_path_input, mode= 'a', header=False)

  
    
def main(args=None):
    rclpy.init(args= args)
    node = IntermidiateProcessingTimeCollectorString()
    rclpy.spin(node)
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()