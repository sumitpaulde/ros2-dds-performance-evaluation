import rclpy
import cv2
from rclpy.node import Node 
from hare_robot_interfaces.msg import Binaryfile 
from cv_bridge import CvBridge 
import datetime
import pandas as pd
import numpy as np
import os



class BinaryPublisherRelaySubscriber(Node):
    
    def __init__(self):
        super().__init__("binary_transfer_rate_publisher")
        self.get_logger().info("Binay Image Transfer Publisher node started")
        self.extention = '.png'
        self.home_path = os.path.expanduser('~')
        self.intial_path = self.home_path + "/images/images_33kb"
        self.file_path_parameter = self.declare_parameter("filepath", self.intial_path)
        self.file_path_parameter_input = self.get_parameter("filepath").value
        self.timer_parameter = self.declare_parameter("hz", 10)
        self.timer_parameter_input = self.get_parameter("hz").value
        self.domain_parameter = self.declare_parameter("domain", "different_domain")
        self.domain_parameter_input = self.get_parameter("domain").value
        self.timer = self.create_timer(self.hz_to_second(self.timer_parameter_input), self.publish_image_callback)
        self.filepath =  self.file_path_parameter_input
        self.image_capture = [self.load_binary_file_from_local(self.filepath)]
        self.images= np.array(self.image_capture)
        self.bridge = CvBridge()
        self.data_frame_receiver = 0
        self.data_frame_publisher = 0
        self.data_array = []
        self.image_publisher = self.create_publisher( Binaryfile, "binary_image", 1)
        self.subscription = self.create_subscription(Binaryfile, "binary_image_relay",self.cam_image_listener_callback, 1)


    def publish_image_callback(self):
        msg = Binaryfile()
        msg.im = self.bridge.cv2_to_imgmsg(self.images)
        msg.publish_time = self.get_current_timestamp()
        self.image_publisher.publish(msg)
        self.data_frame_publisher += 1
        if self.data_frame_publisher > 50:
             raise SystemExit      
        self.get_logger().info('Binary Files Are Publishing ' + str(self.file_path_parameter_input ) + " with rate " + str(self.timer_parameter_input) + "hz for domain: "+ str(self.domain_parameter_input) + " received frame: " + str(self.data_frame_receiver))

    def load_binary_file_from_local(self, filepath):
        os.chdir(filepath)
        n=0
        for subdir, dirs, files in os.walk("."):
                    if files:
                        for name in dirs:
                            print("dir- " + subdir + os.sep + name)
                        for file in files:
                                n+=1 
                                # print("file- "+ str(n) + subdir + os.sep + file)
                                filepath = subdir + os.sep + file
                                if filepath.endswith(self.extention):
                                    return cv2.imread(filepath)
                                

    def cam_image_listener_callback(self, data):
        self.get_logger().info('Receiving binary file')
        if data.subscribe_time == 1.1111 :
            msg = Binaryfile()
            msg.publish_time = float(0.00)
            self.image_publisher.publish(msg)
            raise SystemExit 
    
    def get_current_timestamp(self):
        current_time = datetime.datetime.now()
        time_stamp = current_time.timestamp()
        return time_stamp


    def hz_to_second(self, hz_input):
        timer_input = float(1 / hz_input)
        return timer_input
    
           
            
def main(args=None):
# this line is required to initialize the ros communication.
    rclpy.init(args= args)
    node = BinaryPublisherRelaySubscriber()
    rclpy.spin(node)
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()