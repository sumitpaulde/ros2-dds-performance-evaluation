import rclpy
import cv2
import datetime;
from rclpy.node import Node 
from hare_robot_interfaces.msg import Binaryfile 
from cv_bridge import CvBridge 
import numpy as np
import os


class IntermidiateProcessingTimeCollectorPublisher(Node):
    
    def __init__(self):
        super().__init__("intermidiate_processing_time_collector_pub")
        self.image_publisher = self.create_publisher(Binaryfile, "binary_image", 1)
        self.get_logger().info("intermidiate_processing_time_collector publisher node started")
        self.extention = '.png'
        self.file_path_parameter = self.declare_parameter("filepath")
        self.file_path_parameter_input = self.get_parameter("filepath").value
        self.timer_parameter = self.declare_parameter("hz")
        self.timer_parameter_input = self.get_parameter("hz").value
        self.timer = self.create_timer(self.hz_to_second(self.timer_parameter_input), self.publish_image_callback)
        # '/home/sumit/raw_data_binary/2011_09_26/2011_09_26_drive_0001_sync/'
        self.filepath =  self.file_path_parameter_input
        self.image_capture = [self.load_binary_file_from_local(self.filepath)]
        self.images= cv2.cvtColor(np.array(self.image_capture),cv2.COLOR_GRAY2RGB)
        self.bridge = CvBridge()
        
        
        
    def publish_image_callback(self):
        msg = Binaryfile()
        msg.im = self.bridge.cv2_to_imgmsg(self.images)
        msg.publish_time = self.get_current_timestamp()
        msg.subscribe_time = 0.0
        self.image_publisher.publish(msg)
        self.get_logger().info('Binary Files Are Publishing at timestamp: ' + str(msg.publish_time))
        
    def load_binary_file_from_local(self, filepath):
        os.chdir(filepath)
        n=0
        for subdir, dirs, files in os.walk("."):
                    if files:
                        for name in dirs:
                            print("dir- " + subdir + os.sep + name)
                        for file in files:
                                n+=1 
                                print("file- "+ str(n) + subdir + os.sep + file)
                                filepath = subdir + os.sep + file
                                if filepath.endswith(self.extention):
                                    return cv2.imread(filepath)
        
    def get_current_timestamp(self):
        current_time = datetime.datetime.now()
        time_stamp = current_time.timestamp()
        return time_stamp

    def hz_to_second(self, hz_input):
        timer_input = float(1 / hz_input)
        return timer_input

            
            
def main(args=None):
    rclpy.init(args= args)
    node = IntermidiateProcessingTimeCollectorPublisher()
    rclpy.spin(node)
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()
