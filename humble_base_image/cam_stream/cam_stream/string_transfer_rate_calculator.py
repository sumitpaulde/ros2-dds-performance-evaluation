import rclpy 
from rclpy.node import Node 
from hare_robot_interfaces.msg import StringTimestamp
from std_msgs.msg import String
import datetime
import pandas as pd
import os


 
class StringTransferRateCalculator(Node):

  def __init__(self):
    super().__init__('string_transfer_rate_calculator')
    
    # self.sample_string_relay_publisher = self.create_publisher( StringTimestamp, "sample_string_relay", 10)
    self.get_logger().info("string_transfer_rate_calculator node started")
    self.data_frame_counter = 0
    self.data_array = []
    self.hz_parameter = self.declare_parameter("hz", 10)
    self.hz_parameter_input = self.get_parameter("hz").value
    self.rate = self.hz_parameter_input
    self.domain_parameter = self.declare_parameter("domain", "different_domain")
    self.domain_parameter_input = self.get_parameter("domain").value
    self.home_path = os.path.expanduser('~')
    self.csv_folder_location = self.home_path + os.sep +"experiment_results"+  os.sep + "transfer_rate" + os.sep + self.domain_parameter_input + os.sep + "string" + os.sep + str(self.hz_parameter_input) + "hz" 
    os.makedirs(self.csv_folder_location, exist_ok=True)
    self.subscription = self.create_subscription(StringTimestamp, "string_data", self.string_msg_listener_callback, 1)
    
    
  def string_msg_listener_callback(self, data):
    # relay_msg = StringTimestamp()
    subscribe_time = float(self.get_current_timestamp())
    self.get_logger().info('Subscribed to the String_data')
    downloaded_string = String()
    downloaded_string = data.input
    downloaded_time = float(self.get_current_timestamp())
    string_length = str(len((downloaded_string.data)))
    file_name = "string_"+ string_length + "_char_" + str(self.rate)  +"hz_transfer_rate_data.csv"
    file_path = self.csv_folder_location + os.sep  + file_name
    self.data_frame_counter +=1
    transfer_rate_of_string = datetime.datetime.fromtimestamp(float(downloaded_time))  - datetime.datetime.fromtimestamp(float(subscribe_time))  
    transfer_rate = transfer_rate_of_string.total_seconds()
    self.data_array.append(['Frame_Number: '+ str(self.data_frame_counter), subscribe_time ,  downloaded_time, transfer_rate ])
    self.get_logger().info("Downloaded String: " + downloaded_string.data)
    if self.data_frame_counter % 50 == 0 :
          self.write_csv_from_dataframe(self.data_array, file_path )
          self.data_array.clear()
    if string_length == "1":
          raise SystemExit
    
    


  def get_current_timestamp(self):
    current_time = datetime.datetime.now()
    time_stamp = current_time.timestamp()
    return time_stamp

  def write_csv_from_dataframe(self, data_array, file_path_input):
      columns_input=['Frame_Number','Subscribe_Time', 'String_Msg_Downloaded_Time', 'Single_frame_download_time']
      
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
    node = StringTransferRateCalculator()
    rclpy.spin(node)
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()