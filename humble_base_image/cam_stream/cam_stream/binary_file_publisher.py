import rclpy
import cv2
import numpy as np
import os
from rclpy.node import Node 
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge 


class BinaryFilePublisher(Node):
    
    def __init__(self):
        super().__init__("binary_file_publisher")
        self.image_publisher = self.create_publisher(Image, "cam_image", 10)
        self.timer = self.create_timer(0.1, self.publish_image_callback)
        self.get_logger().info(" cam_publisher node started")
        self.extention = '.png'
        self.file_path_parameter = self.declare_parameter("filepath")
        self.file_path_parameter_input = self.get_parameter("filepath").value
        # '/home/sumit/raw_data_binary/2011_09_26/2011_09_26_drive_0001_sync/'
        self.filepath =  self.file_path_parameter_input
        self.image_capture = [self.load_binary_file_from_local(self.filepath)]
        self.images= cv2.cvtColor(np.array(self.image_capture),cv2.COLOR_GRAY2RGB)
        self.bridge = CvBridge()
        
        
        
    def publish_image_callback(self):
        self.image_publisher.publish(self.bridge.cv2_to_imgmsg(self.images))
        #self.get_logger().info('Binary Files Are Publishing')
        
    def load_binary_file_from_local(self, filepath):
        os.chdir(filepath)
        n=0
        for root, subdirs, files in os.walk("."):
                    if files:
                        for dir in subdirs:
                            self.get_logger().info("dir- " + dir)
                            self.get_logger().info( str(subdirs))
                        for file in files:
                            n+=1 
                            filepath = root + os.sep + file
                            self.get_logger().info("fileno: " + str(n) +" filepath: " + str(filepath))
                            if filepath.endswith(self.extention):
                                return cv2.imread(filepath)
        
            
            
def main(args=None):
    rclpy.init(args= args)
    node = BinaryFilePublisher()
    rclpy.spin(node)
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()
