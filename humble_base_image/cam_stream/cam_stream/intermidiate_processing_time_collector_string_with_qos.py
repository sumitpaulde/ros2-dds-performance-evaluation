import rclpy 
from rclpy.node import Node 
from hare_robot_interfaces.msg import StringTimestamp
import datetime
import pandas as pd
import os
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy


 
class IntermidiateProcessingTimeCollectorString(Node):

  def __init__(self):
    super().__init__('intermidiate_processing_time_collector_string')
    self.data_frame_counter = 0
    self.data_array = []
    self.string_size_parameter = self.declare_parameter("stringsize")
    self.string_size_parameter_input = self.get_parameter("stringsize").value
    self.string_filesize = self.string_size_parameter_input
    self.hz_parameter = self.declare_parameter("hz")
    self.hz_parameter_input = self.get_parameter("hz").value
    self.rate = self.hz_parameter_input
    self.home_path = os.path.expanduser('~')
    self.reliability_parameter = self.declare_parameter("reliability", "RELIABLE")
    self.reliability_input = self.get_parameter("reliability").value
    self.durability_parameter = self.declare_parameter("durability", "VOLATILE")
    self.durability_input = self.get_parameter("durability").value
    self.subscription = self.create_subscription(StringTimestamp, "string_data", self.string_data_listener_callback, qos_profile=self.get_qos_profile_settings(self.reliability_input, self.durability_input))
    self.image_relay_publisher = self.create_publisher( StringTimestamp, "string_data_relay1", qos_profile=self.get_qos_profile_settings(self.reliability_input, self.durability_input))
    
  def string_data_listener_callback(self, data):
    subscribe_time = float(self.get_current_timestamp())
    self.get_logger().info('Receiving String data')
    relay_msg =   StringTimestamp()
    relay_msg.input = data.input
    relay_msg.publish_time = data.publish_time
    self.image_relay_publisher.publish(relay_msg)
    publish_time = float(self.get_current_timestamp())
    self.get_logger().info('Publishing Sample String relay')
    file_name =  "interprocess_string_" + str(self.string_filesize)+ "_char_" + str(self.rate)  +"_hz_latency_data.csv"
    csv_folder_location = self.home_path + os.sep +"experiment_results"+ os.sep + "interprocessing_time" + os.sep + "string" + os.sep + str(self.hz_parameter_input) + "hz" 
    os.makedirs(csv_folder_location, exist_ok=True)
    file_path = csv_folder_location + os.sep  + file_name
    self.data_frame_counter +=1
    self.data_array.append(['Frame_Number: '+ str(self.data_frame_counter),  subscribe_time, publish_time ])
    if self.data_frame_counter % 100 == 0 :
          self.write_csv_from_dataframe(self.data_array, file_path )
          self.data_array.clear()
          self.get_logger().info("--------------------------------------------")
          self.get_logger().info("+++++++++++++++++++++++++++++++++++++++++++++")
    self.get_logger().info("Relay Publishing the String data")
    

  def get_current_timestamp(self):
    current_time = datetime.datetime.now()
    time_stamp = current_time.timestamp()
    return time_stamp

  def write_csv_from_dataframe(self, data_array, file_path_input):
      columns_input=['Frame_Number','Subscribe_Time_Subscriber', 'Publish_Time_From_Subscriber']
      
      if os.path.exists(file_path_input) == False :
          data_frame = pd.DataFrame(data_array, columns=columns_input)
      # data_frame.set_index('Frame_Number')
          data_frame.to_csv(file_path_input)
      else :
          data_frame = pd.DataFrame(data_array)
        # data_frame.set_index('Frame_Number')
          data_frame.to_csv(file_path_input, mode= 'a', header=False)

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
    node = IntermidiateProcessingTimeCollectorString()
    rclpy.spin(node)
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()