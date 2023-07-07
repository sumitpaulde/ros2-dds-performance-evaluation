import rclpy 
from rclpy.node import Node 
from hare_robot_interfaces.msg import Imu 
import datetime
import pandas as pd
import os



 
class IMUTransferRateCalculator(Node):

  def __init__(self):
    super().__init__('imu_transfer_rate_calculator')
    self.get_logger().info('intermidiate_processing_time_collector_imu node has been started')
    self.data_frame_counter = 0
    self.data_array = []
    self.hz_parameter = self.declare_parameter("hz")
    self.hz_parameter_input = self.get_parameter("hz").value
    self.rate = self.hz_parameter_input
    self.home_path = os.path.expanduser('~')
    file_name =  "interprocess_imu_" + str(self.rate)  +"_hz_latency_data.csv"
    csv_folder_location = self.home_path + os.sep +"experiment_results"+ os.sep + "interprocessing_time" + os.sep + "imu" + os.sep + str(self.hz_parameter_input) + "hz" 
    os.makedirs(csv_folder_location, exist_ok=True)
    self.file_path = csv_folder_location + os.sep  + file_name
    self.subscription = self.create_subscription(Imu, "imu_data", self.imu_data_listener_callback, 1)
    self.image_relay_publisher = self.create_publisher( Imu, "imu_data_relay1", 1)
    
  def imu_data_listener_callback(self, data):
    subscribe_time = float(self.get_current_timestamp())
    self.get_logger().info('Receiving Imu data')
    relay_msg = Imu()
    relay_msg.header = data.header
    relay_msg.orientation_covariance = data.orientation_covariance
    relay_msg.angular_velocity_covariance = data.angular_velocity_covariance
    relay_msg.publish_time = data.publish_time
    download_time_stamp = float(self.get_current_timestamp())
    self.image_relay_publisher.publish(relay_msg)
    
    self.data_frame_counter +=1
    intermediate_time_temp  = datetime.datetime.fromtimestamp(float(download_time_stamp))  - datetime.datetime.fromtimestamp(float(subscribe_time))  
    download_time = intermediate_time_temp.total_seconds()
    self.data_array.append(['Frame_Number: '+ str(self.data_frame_counter),  subscribe_time, download_time_stamp, download_time ])

    if relay_msg.header.frame_id == str(000):
          self.write_csv_from_dataframe(self.data_array, self.file_path )
          self.data_array.clear()
          self.get_logger().info("Imu data Interprocessing finished for " + str(self.rate) + "hz")
          raise SystemExit

  
    

  def get_current_timestamp(self):
    current_time = datetime.datetime.now()
    time_stamp = current_time.timestamp()
    return time_stamp

  def write_csv_from_dataframe(self, data_array, file_path_input):
      columns_input=['Frame_Number','Subscribe_Time_Subscriber', 'Publish_Time_From_Subscriber', 'Single_frame_download_time']
      
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
    node = IMUTransferRateCalculator()
    rclpy.spin(node)
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()