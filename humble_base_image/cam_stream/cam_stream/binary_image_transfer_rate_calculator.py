import rclpy 
import cv2 
from rclpy.node import Node 
from hare_robot_interfaces.msg import Binaryfile
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge 
import datetime
import pandas as pd
import os
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy

 
class BinaryImageTransferRateCalculator(Node):

  def __init__(self):
    super().__init__('binary_image_transfer_rate_calculator')
    self.get_logger().info('binary_image_transfer_rate calculating node has started')
    #self.timer = self.create_timer(0.0000001, self.relay_publisher_callback)
    # self.bridge = CvBridge()
    self.data_frame_counter = 0
    self.data_array = []
    self.image_size_parameter = self.declare_parameter("imagesize", "33kb")
    self.image_size_parameter_input = self.get_parameter("imagesize").value
    self.image_filesize = self.image_size_parameter_input
    self.hz_parameter = self.declare_parameter("hz", 1)
    self.hz_parameter_input = self.get_parameter("hz").value
    self.rate = self.hz_parameter_input
    self.domain_parameter = self.declare_parameter("domain", "different_domain")
    self.domain_parameter_input = self.get_parameter("domain").value
    self.subscription = self.create_subscription(Binaryfile, "binary_image", self.cam_image_listener_callback, 1)
    self.image_relay_publisher = self.create_publisher( Binaryfile, "binary_image_relay1", 1)
    self.home_path = os.path.expanduser('~')
    file_name = "binary_image_"+ str(self.image_filesize) + "_" + str(self.rate)  +"_hz_transfer_rate_data.csv"
    csv_folder_location = self.home_path + os.sep +"experiment_results"+ os.sep + "transfer_rate" + os.sep + "binary_transfer" + os.sep + self.domain_parameter_input + os.sep + str(self.hz_parameter_input) + "hz"
    os.makedirs(csv_folder_location, exist_ok=True)
    self.file_path = csv_folder_location + os.sep  + file_name
    self.get_logger().info("csv_file_path= "+ self.file_path)
    
  def cam_image_listener_callback(self, data):
    relay_msg = Image()
    subscribe_time = float(self.get_current_timestamp())
    self.get_logger().info('Subscribed to the binary_image')
    relay_msg = data
    downloaded_time = float(self.get_current_timestamp())
    if relay_msg.publish_time == 0.00 :
        raise SystemExit 
    self.data_frame_counter +=1
    transfer_rate_of_file = datetime.datetime.fromtimestamp(float(downloaded_time))  - datetime.datetime.fromtimestamp(float(subscribe_time))  
    transfer_rate = transfer_rate_of_file.total_seconds()

    self.data_array.append(['Frame_Number: '+ str(self.data_frame_counter), subscribe_time ,  downloaded_time, transfer_rate])
    transfer_rate_of_file = downloaded_time - subscribe_time
    if self.data_frame_counter % 30 == 0 :
          self.write_csv_from_dataframe(self.data_array, self.file_path )
          self.data_array.clear()
          self.get_logger().info("--------------------------------------------")
          self.get_logger().info("+++++++++++++++++++++++++++++++++++++++++++++")
    self.get_logger().info('I frame has been transfered in : ' + str(transfer_rate_of_file) + " sec")
    
    cv2.waitKey(1)


  def get_current_timestamp(self):
    current_time = datetime.datetime.now()
    time_stamp = current_time.timestamp()
    return time_stamp

  def write_csv_from_dataframe(self, data_array, file_path_input):
      columns_input=['Frame_Number','Subscribe_Time', 'Binary_Image_Downloaded_Time', 'Single_frame_download_time']
      
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
    node = BinaryImageTransferRateCalculator()
    rclpy.spin(node)
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()