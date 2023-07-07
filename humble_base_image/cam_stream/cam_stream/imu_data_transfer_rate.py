import rclpy 
from rclpy.node import Node 
from hare_robot_interfaces.msg import Imu
from std_msgs.msg import String
import datetime
import pandas as pd
import os
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy


 
class IMUTransferRate(Node):

  def __init__(self):
    super().__init__('imu_data_transfer_rate')
    self.get_logger().info("Imu Transfer rate checker node started")
    self.data_frame_counter = 0
    self.data_array = []
    self.hz_parameter = self.declare_parameter("hz", 10)
    self.hz_parameter_input = self.get_parameter("hz").value
    self.rate = self.hz_parameter_input
    self.home_path = os.path.expanduser('~')
    file_name = "imu_data_"+ str(self.rate)  +"_hz_transfer_rate_data.csv"
    csv_folder_location = self.home_path + os.sep +"experiment_results"+  os.sep + "transfer_rate"+ os.sep + "imu"
    os.makedirs(csv_folder_location, exist_ok=True)
    self.file_path = csv_folder_location + os.sep  + file_name

    self.subscription = self.create_subscription(Imu, "imu_data", self.imu_data_listener_callback, qos_profile=self.get_qos_profile_settings(self.reliability_input, self.durability_input))
    
  def imu_data_listener_callback(self, data):
    self.get_logger().info('Subscribed to the imu_data')
    subscribe_time = float(self.get_current_timestamp())
    relay_msg = Imu()
    relay_msg.header = data.header
    relay_msg.orientation_covariance = data.orientation_covariance
    relay_msg.angular_velocity_covariance = data.angular_velocity_covariance
    relay_msg.publish_time = data.publish_time
    downloaded_time = float(self.get_current_timestamp())

    self.data_frame_counter +=1
    transfer_rate_of_imu = datetime.fromtimestamp(float(downloaded_time))  - datetime.fromtimestamp(float(subscribe_time))  
    transfer_rate = transfer_rate_of_imu.total_seconds()
    self.data_array.append(['Frame_Number: '+ str(self.data_frame_counter), subscribe_time ,  downloaded_time, transfer_rate])
    if self.data_frame_counter % 100 == 0 :
          self.write_csv_from_dataframe(self.data_array, self.file_path )
          self.data_array.clear()
          self.get_logger().info("--------------------------------------------")
          self.get_logger().info("+++++++++++++++++++++++++++++++++++++++++++++")
    self.get_logger().info("Downloading the IMU file")
    


  def get_current_timestamp(self):
    current_time = datetime.datetime.now()
    time_stamp = current_time.timestamp()
    return time_stamp

  def write_csv_from_dataframe(self, data_array, file_path_input):
      columns_input=['Frame_Number','Subscribe_Time', 'IMU_Msg_Downloaded_Time', 'Single_frame_download_time']
      
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
    node = IMUTransferRate()
    rclpy.spin(node)
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()