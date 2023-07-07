import rclpy 
from rclpy.node import Node 
from hare_robot_interfaces.msg import StringTimestamp
from std_msgs.msg import String
import datetime
import pandas as pd
import os
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy

 
class StringTransferRate(Node):

  def __init__(self):
    super().__init__('string_transfer_rate_with_qos')
    
    # self.sample_string_relay_publisher = self.create_publisher( StringTimestamp, "sample_string_relay", 10)
    self.get_logger().info("Sample string publisher node started")
    self.data_frame_counter = 0
    self.data_array = []
    self.hz_parameter = self.declare_parameter("hz", 10)
    self.hz_parameter_input = self.get_parameter("hz").value
    self.rate = self.hz_parameter_input
    self.home_path = os.path.expanduser('~')
    self.reliability_parameter = self.declare_parameter("reliability", "RELIABLE")
    self.reliability_input = self.get_parameter("reliability").value
    self.durability_parameter = self.declare_parameter("durability", "VOLATILE")
    self.durability_input = self.get_parameter("durability").value
    self.dds_implementation_parameter = self.declare_parameter("dds", "cyclone")
    self.dds_input = self.get_parameter("dds").value
    self.subscription = self.create_subscription(StringTimestamp, "string_data", self.string_msg_listener_callback, qos_profile=self.get_qos_profile_settings(self.reliability_input, self.durability_input))
    
    
  def string_msg_listener_callback(self, data):
    # relay_msg = StringTimestamp()
    subscribe_time = float(self.get_current_timestamp())
    self.get_logger().info('Subscribed to the String_data')
    downloaded_string = String()
    downloaded_string = data.input
    downloaded_time = float(self.get_current_timestamp())
    string_length = str(len((downloaded_string.data)))
    file_name = "string_"+ string_length + "_char_" + str(self.rate)  +"_transfer_rate_data.csv"
    csv_folder_location = self.home_path + os.sep +"experiment_results"+  os.sep + "transfer_rate"+ os.sep + str((self.dds_input).lower()) + os.sep + "string" + os.sep + \
         str(self.reliability_input[0]) + "_" + str(self.durability_input[0])+ os.sep + str(self.hz_parameter_input) + "hz" 
    os.makedirs(csv_folder_location, exist_ok=True)
    file_path = csv_folder_location + os.sep  + file_name
    self.data_frame_counter +=1
    downloading_time_for_one_frame = downloaded_time - subscribe_time
    self.data_array.append(['Frame_Number: '+ str(self.data_frame_counter), subscribe_time ,  downloaded_time, downloading_time_for_one_frame])
    if self.data_frame_counter % 100 == 0 :
          self.write_csv_from_dataframe(self.data_array, file_path )
          self.data_array.clear()
          self.get_logger().info("--------------------------------------------")
          self.get_logger().info("+++++++++++++++++++++++++++++++++++++++++++++")
    self.get_logger().info("Downloading the String file")
    


  def get_current_timestamp(self):
    current_time = datetime.datetime.now()
    time_stamp = current_time.timestamp()
    return time_stamp

  def write_csv_from_dataframe(self, data_array, file_path_input):
      columns_input=['Frame_Number','Subscribe_Time', 'String_Msg_Downloaded_Time', 'downloading time for one string frame']
      
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
    node = StringTransferRate()
    rclpy.spin(node)
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()