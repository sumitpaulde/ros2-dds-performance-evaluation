import rclpy 
import cv2 
from rclpy.node import Node 
from hare_robot_interfaces.msg import Binaryfile 
from cv_bridge import CvBridge 
import datetime
import pandas as pd
import os

 
class CamSubscriberRelayPublisher(Node):

  def __init__(self):
    super().__init__('cam_subscriber_relay_publisher')
    self.subscription = self.create_subscription(Binaryfile, "binary_image", self.cam_image_listener_callback, 10)
    self.image_relay_publisher = self.create_publisher( Binaryfile, "binary_image_relay", 10)
    #self.timer = self.create_timer(0.0000001, self.relay_publisher_callback)
    self.bridge = CvBridge()
    self.data_frame_counter = 0
    self.data_array = []
    
  def cam_image_listener_callback(self, data):
    relay_msg = Binaryfile()
    relay_msg.subscribe_time = float(self.get_current_timestamp())
    window_title = "Subscriber Feed"
    self.get_logger().info('Receiving cam stream')
    current_frame = self.bridge.imgmsg_to_cv2(data.im)
    #cv2.imshow(window_title, current_frame)
    relay_msg.im = data.im
    relay_msg.publish_time = float(self.get_current_timestamp())
    self.image_relay_publisher.publish(relay_msg)
    self.data_frame_counter +=1
    self.data_array.append(['Frame_Number: '+ str(self.data_frame_counter), data.publish_time, relay_msg.subscribe_time ,  relay_msg.publish_time])
    if self.data_frame_counter % 100 == 0 :
          self.write_csv_from_dataframe(self.data_array)
          self.data_array.clear()
          self.get_logger().info("--------------------------------------------")
          self.get_logger().info("+++++++++++++++++++++++++++++++++++++++++++++")
    self.get_logger().info('Binary Files are Subscribed by realy_subscriber at timestamp: ' + str(relay_msg.subscribe_time) +' Published by relay_subsciber at: ' + str(relay_msg.publish_time))
    self.get_logger().info("Relay Publishing the Binary File")
    
    cv2.waitKey(1)


  def get_current_timestamp(self):
    current_time = datetime.datetime.now()
    time_stamp = current_time.timestamp()
    return time_stamp

  def write_csv_from_dataframe(self, data_array):
      columns_input=['Frame_Number','Publish_Time_Publisher','Subscribe_Time', 'Publish_Time_From_Subscriber']
      if os.path.exists("/home/spaul/interprocess_latency_data.csv") == False :
          data_frame = pd.DataFrame(data_array, columns=columns_input)
      # data_frame.set_index('Frame_Number')
          data_frame.to_csv("/home/spaul/interprocess_latency_data.csv")
      else :
          data_frame = pd.DataFrame(data_array)
        # data_frame.set_index('Frame_Number')
          data_frame.to_csv("/home/spaul/interprocess_latency_data.csv", mode= 'a', header=False)
    
def main(args=None):
    rclpy.init(args= args)
    node = CamSubscriberRelayPublisher()
    rclpy.spin(node)
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()