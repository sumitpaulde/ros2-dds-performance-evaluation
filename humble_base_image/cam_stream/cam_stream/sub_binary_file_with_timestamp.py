import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from hare_robot_interfaces.msg import Binaryfile 
import cv2 # OpenCV library
import datetime
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import pandas as pd
import os
 
class BinaryFileSubscriberWithTimeStamp(Node):

  def __init__(self):
    super().__init__('sub_binary_file_with_timestamp')
    self.subscription = self.create_subscription(Binaryfile, "cam_image_relay", self.cam_image_listener_callback, 10)
    self.bridge = CvBridge()
    self.data_frame_counter = 0
    self.data_array = []
    self.csv_file_path= "/home/spaul/ros2_ws/latency_data.csv"
    
  def cam_image_listener_callback(self, data):
    self.get_logger().info('Receiving Binary Image')
    current_frame = self.bridge.imgmsg_to_cv2(data.im, desired_encoding='8UC3')
    data.subscribe_time = float(self.get_current_timestamp()) 
    self.data_frame_counter +=1
    self.data_array.append(['Frame_Number: '+ str(self.data_frame_counter), data.publish_time, data.subscribe_time, current_frame.nbytes])
    if self.data_frame_counter % 100 == 0 :
        self.write_csv_from_dataframe(self.data_array)
        self.data_array.clear()
        self.get_logger().info("--------------------------------------------")
        self.get_logger().info("+++++++++++++++++++++++++++++++++++++++++++++")
    if self.data_frame_counter % 1000 == 0 :
        self.calculate_latency()
    #self.get_logger().info('Binary Files are Published at timestamp: ' + str(data.publish_time) +' Subscribed at: ' + str(data.subscribe_time))
    cv2.imshow("images", current_frame)
    current_frame.nbytes
    self.get_logger().info('Binary File size: ' + str(current_frame.nbytes))
    cv2.waitKey(1)

  def get_current_timestamp(self):
    current_time = datetime.datetime.now()
    time_stamp = current_time.timestamp()
    return time_stamp

  def write_csv_from_dataframe(self, data_array):
    columns_input=['Frame_Number','Publish_Time', 'Subscribe_Time', "Frame_Size_In_Byte"]
    if os.path.exists(self.csv_file_path) == False :
      data_frame = pd.DataFrame(data_array, columns=columns_input)
      # data_frame.set_index('Frame_Number')
      data_frame.to_csv(self.csv_file_path)
    else :
      data_frame = pd.DataFrame(data_array)
      # data_frame.set_index('Frame_Number')
      data_frame.to_csv(self.csv_file_path, mode= 'a', header=False)

  def calculate_latency(self):
    df = pd.read_csv(self.csv_file_path)
    new_col = (df["Subscribe_Time"]-df["Publish_Time"]).tolist()
    df.insert(loc=5, column='Single File latency', value=new_col)
    total_time= df['Subscribe_Time'][999] - df['Subscribe_Time'][0] 

    #df.drop('Unnamed: 0', inplace=True, axis=2)
    df.loc['Throughput'] = pd.Series(df['Frame_Size_In_Byte'].sum() / (total_time*1000 ), index=['Frame_Size_In_Byte'])
    df.loc['Latency Average'] = pd.Series(df['Single File latency'].sum() / (1000*2), index=['Single File latency'])

    df.to_csv(self.csv_file_path)
    rclpy.shutdown()

    
def main(args=None):
    rclpy.init(args= args)
    node = BinaryFileSubscriberWithTimeStamp()
    rclpy.spin(node)
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()