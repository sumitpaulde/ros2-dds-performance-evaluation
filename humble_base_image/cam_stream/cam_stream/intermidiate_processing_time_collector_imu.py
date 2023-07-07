import rclpy 
from rclpy.node import Node 
from hare_robot_interfaces.msg import Imu 
import datetime
import pandas as pd
import os
import time



 
class IntermidiateProcessingTimeCollectorImu(Node):

  def __init__(self):
    super().__init__('intermidiate_processing_time_collector_imu')
    self.get_logger().info('intermidiate_processing_time_collector_imu node has been started')
    self.data_frame_counter = 0
    self.data_array = []
    self.hz_parameter = self.declare_parameter("hz")
    self.hz_parameter_input = self.get_parameter("hz").value
    self.domain_parameter = self.declare_parameter("domain", "different_domain")
    self.domain_parameter_input = self.get_parameter("domain").value
    self.rate = self.hz_parameter_input
    self.home_path = os.path.expanduser('~')
    file_name =  "interprocess_imu_" + str(self.rate)  +"_hz_latency_data.csv"
    csv_folder_location = self.home_path + os.sep +"experiment_results"+ os.sep + "interprocessing_time" + os.sep + "imu" + os.sep + self.domain_parameter_input + os.sep + str(self.hz_parameter_input) + "hz" 
    os.makedirs(csv_folder_location, exist_ok=True)
    self.file_path = csv_folder_location + os.sep  + file_name

    self.subscription = self.create_subscription(Imu, "imu_data", self.imu_data_listener_callback, 1)
    self.imu_relay_publisher = self.create_publisher( Imu, "imu_data_relay", 1)
    
  def imu_data_listener_callback(self, data):
    subscribe_time = float(self.get_current_timestamp())
    self.get_logger().info('Receiving Imu data')
    relay_msg = Imu()
    relay_msg.header = data.header
    relay_msg.orientation_covariance = data.orientation_covariance
    relay_msg.angular_velocity_covariance = data.angular_velocity_covariance
    relay_msg.publish_time = data.publish_time
    self.imu_relay_publisher.publish(relay_msg)
    publish_time = float(self.get_current_timestamp())
    self.data_frame_counter +=1
    intermediate_time_temp  = datetime.datetime.fromtimestamp(float(publish_time))  - datetime.datetime.fromtimestamp(float(subscribe_time))  
    intermediate_time = intermediate_time_temp.total_seconds()
    self.data_array.append(['Frame_Number: '+ str(self.data_frame_counter),  subscribe_time, publish_time, intermediate_time ])

    if self.data_frame_counter == 100 and self.data_frame_counter <105 :
          frame_id = str(000)
          relay_msg = Imu()
          relay_msg.header.frame_id = frame_id
          relay_msg.publish_time = float(1.1111)
          self.imu_relay_publisher.publish(relay_msg)
          self.write_csv_from_dataframe(self.data_array, self.file_path )
          self.data_array.clear()
          self.get_logger().info("Imu data Interprocessing finished for " + str(self.rate) + "hz")
          self.get_logger().info("--------------------------------------------")
          self.get_logger().info("+++++++++++++++++++++++++++++++++++++++++++++")
          time.sleep(5)
          raise SystemExit

  
    

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
    node = IntermidiateProcessingTimeCollectorImu()
    rclpy.spin(node)
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()