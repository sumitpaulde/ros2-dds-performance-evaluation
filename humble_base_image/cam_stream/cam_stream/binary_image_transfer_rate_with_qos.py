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

 
class BinaryImageTransferRate(Node):

  def __init__(self):
    super().__init__('binary_image_transfer_rate_with_qos')
    self.get_logger().info('binary_image_transfer_rate calculating node has started')
    #self.timer = self.create_timer(0.0000001, self.relay_publisher_callback)
    # self.bridge = CvBridge()
    self.data_frame_counter = 0
    self.data_array = []
    self.image_size_parameter = self.declare_parameter("imagesize")
    self.image_size_parameter_input = self.get_parameter("imagesize").value
    self.image_filesize = self.image_size_parameter_input
    self.hz_parameter = self.declare_parameter("hz")
    self.hz_parameter_input = self.get_parameter("hz").value
    self.rate = self.hz_parameter_input
    self.reliability_parameter = self.declare_parameter("reliability", "RELIABLE")
    self.reliability_input = self.get_parameter("reliability").value
    self.durability_parameter = self.declare_parameter("durability", "VOLATILE")
    self.durability_input = self.get_parameter("durability").value
    self.dds_implementation_parameter = self.declare_parameter("dds", "cyclone")
    self.dds_input = self.get_parameter("dds").value
    self.subscription = self.create_subscription(Binaryfile, "binary_image", self.cam_image_listener_callback, qos_profile=self.get_qos_profile_settings(self.reliability_input, self.durability_input))
    self.image_relay_publisher = self.create_publisher( Binaryfile, "binary_image_relay1", qos_profile=self.get_qos_profile_settings(self.reliability_input, self.durability_input))
    self.home_path = os.path.expanduser('~')
    
  def cam_image_listener_callback(self, data):
    relay_msg = Image()
    subscribe_time = float(self.get_current_timestamp())
    self.get_logger().info('Subscribed to the binary_image')
    relay_msg = data.im
    downloaded_time = float(self.get_current_timestamp())
    file_name = "binary_image_"+ str(self.image_filesize) + "_" + str(self.rate)  +"_hz_transfer_rate_data.csv"
    csv_folder_location = self.home_path + os.sep +"experiment_results"+ os.sep + "transfer_rate"+ os.sep + str((self.dds_input).lower()) + os.sep + "binary" + os.sep + \
         str(self.reliability_input[0]) + "_" + str(self.durability_input[0])+ os.sep + str(self.hz_parameter_input) + "hz" 
    self.get_logger().info("")
    os.makedirs(csv_folder_location, exist_ok=True)
    file_path = csv_folder_location + os.sep  + file_name
    self.get_logger().info("csv_file_path= "+ file_path)
    
    self.data_frame_counter +=1
    transfer_rate_of_file = downloaded_time - subscribe_time
    self.data_array.append(['Frame_Number: '+ str(self.data_frame_counter), subscribe_time ,  downloaded_time, transfer_rate_of_file])
    transfer_rate_of_file = downloaded_time - subscribe_time
    if self.data_frame_counter % 100 == 0 :
          self.write_csv_from_dataframe(self.data_array, file_path )
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
    node = BinaryImageTransferRate()
    rclpy.spin(node)
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()