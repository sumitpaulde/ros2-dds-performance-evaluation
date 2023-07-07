import rclpy 
import cv2 
from rclpy.node import Node 
from hare_robot_interfaces.msg import Binaryfile 
from cv_bridge import CvBridge 
import datetime
import pandas as pd
import os

 
class IntermidiateProcessingTimeCollectorSub(Node):

  def __init__(self):
    super().__init__('intermidiate_processing_time_collector_binary')
    #self.timer = self.create_timer(0.0000001, self.relay_publisher_callback)
    self.bridge = CvBridge()
    self.data_frame_counter = 0
    self.data_array = []
    self.image_size_parameter = self.declare_parameter("imagesize","33kb")
    self.image_size_parameter_input = self.get_parameter("imagesize").value
    self.image_filesize = self.image_size_parameter_input
    self.hz_parameter = self.declare_parameter("hz",1)
    self.hz_parameter_input = self.get_parameter("hz").value
    self.rate = self.hz_parameter_input
    self.home_path = os.path.expanduser('~')
    self.subscription = self.create_subscription(Binaryfile, "binary_image", self.cam_image_listener_callback, 1)
    self.image_relay_publisher = self.create_publisher( Binaryfile, "binary_image_relay", 1)
    file_name = "interprocess_"+ str(self.image_filesize) + "_" + str(self.rate)  +"_latency_data.csv"
    csv_folder_location = self.home_path + os.sep +"experiment_results"+ os.sep + "interprocessing_time" + os.sep + "binary" + os.sep + str(self.hz_parameter_input) + "hz" 
    os.makedirs(csv_folder_location, exist_ok=True)
    self.file_path = csv_folder_location + os.sep  + file_name
    
  def cam_image_listener_callback(self, data):
    subscribe_time = float(self.get_current_timestamp())
    relay_msg = Binaryfile()
    self.get_logger().info('Receiving cam stream')
    relay_msg.im = data.im
    relay_msg.publish_time = data.publish_time
    self.image_relay_publisher.publish(relay_msg)
    publish_time = float(self.get_current_timestamp())
    self.data_frame_counter +=1
    intermediate_time_temp  = datetime.datetime.fromtimestamp(float(publish_time))  - datetime.datetime.fromtimestamp(float(subscribe_time))  
    intermediate_time = intermediate_time_temp.total_seconds()
    self.data_array.append(['Frame_Number: '+ str(self.data_frame_counter), subscribe_time ,  publish_time, intermediate_time])
    if self.data_frame_counter == 120 and self.data_frame_counter < 125:
          relay_msg.subscribe_time = float(1.1111)
          self.image_relay_publisher.publish(relay_msg)
          self.write_csv_from_dataframe(self.data_array, self.file_path )
          self.data_array.clear()
          self.get_logger().info("--------------------------------------------")
          self.get_logger().info("+++++++++++++++++++++++++++++++++++++++++++++")
    if relay_msg.publish_time == 0.00:
          raise SystemExit 
    self.get_logger().info('Binary Files interprocess calculating for ' + str(self.image_filesize ) +' with rate: ' + str(self.rate ) +"hz")
    self.get_logger().info("Relay Publishing the Binary File")
    
    cv2.waitKey(1)


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
    node = IntermidiateProcessingTimeCollectorSub()
    rclpy.spin(node)
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()