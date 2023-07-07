import rclpy
import cv2
from rclpy.node import Node 
from hare_robot_interfaces.msg import Binaryfile 
from cv_bridge import CvBridge 
import datetime
import pandas as pd
import os



class CamPublisherRelaySubscriberSterio(Node):
    
    def __init__(self):
        super().__init__("cam_publisher_relay_subscriber_sterio")
        self.image_publisher = self.create_publisher( Binaryfile, "cam_image", 10)
       # self.subscription = self.create_subscription(  Binaryfile, "cam_image_relay", self.cam_image_relay_listener_callback, 10)
        self.timer = self.create_timer(0.0000001, self.publish_image_callback)
        self.get_logger().info(" cam_publisher node started")
        self.image_capture = cv2.VideoCapture(self.gstreamer_pipeline(), cv2.CAP_GSTREAMER)
        self.bridge = CvBridge()
        self.data_frame_counter = 0
        self.data_array = []
        
    def publish_image_callback(self):
        window_title = "Publisher Camera"
        if not self.image_capture.isOpened():
            raise IOError("Cannot open webcam")

        while True:
            Has_return, frame = self.image_capture.read()
            if Has_return == True:
                msg = Binaryfile()
                msg.im = self.bridge.cv2_to_imgmsg(frame)
                msg.publish_time = self.get_current_timestamp()
                msg.subscribe_time = 0.0
                self.image_publisher.publish(msg)
                cv2.imshow(window_title, frame)
                self.get_logger().info('Binary Files Are Publishing at timestamp: ' + str(msg.publish_time))

    def get_current_timestamp(self):
        current_time = datetime.datetime.now()
        time_stamp = current_time.timestamp()
        return time_stamp

    def cam_image_relay_listener_callback(self, data):
        data.subscribe_time = float(self.get_current_timestamp()) 
        self.data_frame_counter +=1
        self.data_array.append(['Frame_Number: '+ str(self.data_frame_counter), data.publish_time, data.subscribe_time])
        if self.data_frame_counter % 100 == 0 :
            self.write_csv_from_dataframe(self.data_array)
            self.data_array.clear()
            self.get_logger().info("--------------------------------------------")
            self.get_logger().info("+++++++++++++++++++++++++++++++++++++++++++++")
        self.get_logger().info('Binary Files are Published at timestamp: ' + str(data.publish_time) +' Subscribed at: ' + str(data.subscribe_time))

    def write_csv_from_dataframe(self, data_array):
        columns_input=['Frame_Number','Publish_Time', 'Subscribe_Time']
        if os.path.exists("/home/latency_data.csv") == False :
            data_frame = pd.DataFrame(data_array, columns=columns_input)
      # data_frame.set_index('Frame_Number')
            data_frame.to_csv("/home/latency_data.csv")
        else :
            data_frame = pd.DataFrame(data_array)
        # data_frame.set_index('Frame_Number')
            data_frame.to_csv("/home/latency_data.csv", mode= 'a', header=False)

    def gstreamer_pipeline(self):
        return ("nvarguscamerasrc sensor-id=1 ! video/x-raw(memory:NVMM),width=3280, height=2464,framerate=21/1, format=NV12 ! nvvidconv flip-method=2 ! video/x-raw,format=BGRx, width=816, height=616, pixel-aspect-ratio=1/1 ! videoconvert ! video/x-raw,format=BGR ! appsink drop=1")



            
            
def main(args=None):
# this line is required to initialize the ros communication.
    rclpy.init(args= args)
    node = CamPublisherRelaySubscriberSterio()
    rclpy.spin(node)
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()