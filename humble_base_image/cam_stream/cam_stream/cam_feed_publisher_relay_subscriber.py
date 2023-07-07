import rclpy
import cv2
from rclpy.node import Node 
from hare_robot_interfaces.msg import Binaryfile 
from cv_bridge import CvBridge 
import datetime
import os
import pandas as pd
import time

class CamFeedPublisherRelaySubscriber(Node):
    
    def __init__(self):
        super().__init__("cam_feed_publisher_relay_subscriber")
        self.get_logger().info(" cam_publisher node started")
        self.home_path = os.path.expanduser('~')
        self.image_capture = cv2.VideoCapture(1)
        
        self.bridge = CvBridge()
        self.data_frame_receiver = 0
        self.data_frame_publisher = 0
        self.timer_parameter = self.declare_parameter("hz", 10)
        self.timer_parameter_input = self.get_parameter("hz").value
        self.timer = self.create_timer(self.hz_to_second(self.timer_parameter_input), self.publish_image_callback)
        self.domain_parameter = self.declare_parameter("domain", "different_domain")
        self.domain_parameter_input = self.get_parameter("domain").value
        self.csv_folder_location = self.home_path + os.sep +"experiment_results" + os.sep + "cam_feed_latency" + os.sep + self.domain_parameter_input +os.sep + str(self.timer_parameter_input) + "hz"
        folder_name = "jetbot_cam_feed"
        file_name = folder_name + "_" + str(self.timer_parameter_input) + "hz_latency_data.csv" 
        os.makedirs(self.csv_folder_location, exist_ok=True)
        self.csv_file_path = self.csv_folder_location + os.sep  + file_name
        self.data_array = []
        self.image_publisher = self.create_publisher(Binaryfile, "cam_feed", 10)
        self.subscription = self.create_subscription( Binaryfile, "cam_relay", self.cam_image_relay_listener_callback, 10)


    def publish_image_callback(self):
        msg = Binaryfile()
    
        Has_return, frame = self.image_capture.read()
        if Has_return == True:
                msg.im = self.bridge.cv2_to_imgmsg(frame)
                msg.publish_time = self.get_current_timestamp()
                msg.subscribe_time = 0.0
                self.image_publisher.publish(msg)
                time.sleep(1)
                self.data_frame_publisher += 1
        self.get_logger().info('Camera Stream is Publishing ' + " with rate " + str(self.timer_parameter_input) + "hz for domain: "+ str(self.domain_parameter_input) + " received frame: " + str(self.data_frame_receiver))
        

    def get_current_timestamp(self):
        current_time = datetime.datetime.now()
        time_stamp = current_time.timestamp()
        return time_stamp

    def hz_to_second(self, hz_input):
        timer_input = float(1 / hz_input)
        return timer_input   

    def cam_image_relay_listener_callback(self, data):
        data.subscribe_time = float(self.get_current_timestamp())
        self.get_logger().info('listener started')
        self.data_frame_receiver +=1
        self.data_array.append(['Frame_Number: '+ str(self.data_frame_receiver), data.publish_time, data.subscribe_time ])
        if self.data_frame_receiver % 101 == 0 :
            self.write_csv_from_dataframe(self.data_array, self.csv_file_path)
            self.data_array.clear()
            raise SystemExit 
            # self.get_logger().info("--------------------------------------------")
            # self.get_logger().info("+++++++++++++++++++++++++++++++++++++++++++++")
        self.get_logger().info('Binary Files are Published at timestamp: ' + str(data.publish_time) +' Subscribed at: ' + str(data.subscribe_time))
     
    def write_csv_from_dataframe(self, data_array, csv_file_path):
        columns_input=['Frame_Number','Publish_Time', 'Subscribe_Time']
        if os.path.exists(csv_file_path) == False :
            data_frame = pd.DataFrame(data_array, columns=columns_input)
      # data_frame.set_index('Frame_Number')
            data_frame.to_csv(csv_file_path)
        else :
            data_frame = pd.DataFrame(data_array)
        # data_frame.set_index('Frame_Number')
            data_frame.to_csv(csv_file_path,mode='a', header=False)
            
def main(args=None):
# this line is required to initialize the ros communication.
    rclpy.init(args= args)
    node = CamFeedPublisherRelaySubscriber()
    rclpy.spin(node)
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()