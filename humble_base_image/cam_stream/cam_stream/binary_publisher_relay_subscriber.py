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
        super().__init__("binary_publisher_relay_subscriber")
        self.get_logger().info("Binay Publisher node started")
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
        self.subscription = self.create_subscription( Binaryfile, "binary_image_relay", self.cam_image_relay_listener_callback, 1)
        self.image_publisher = self.create_publisher( Binaryfile, "binary_image", 1)
        self.csv_folder_location = self.home_path + os.sep +"experiment_results" + os.sep + "binary_latency" + os.sep + self.domain_parameter_input +os.sep + str(self.timer_parameter_input) + "hz"
        folder_name = os.path.basename(os.path.normpath(self.filepath))
        file_name = folder_name + "_" + str(self.timer_parameter_input) + "hz_latency_data.csv" 
        os.makedirs(self.csv_folder_location, exist_ok=True)
        self.csv_file_path = self.csv_folder_location + os.sep  + file_name
        self.get_logger().info("csv_file_path= "+ self.csv_file_path)


    def publish_image_callback(self):
        msg = Binaryfile()
        msg.im = self.bridge.cv2_to_imgmsg(self.images)
        msg.publish_time = self.get_current_timestamp()
        msg.subscribe_time = 0.0
        self.image_publisher.publish(msg)
        self.data_frame_publisher += 1
        self.get_logger().info('Binary Files Are Publishing ' + str(self.file_path_parameter_input ) + " with rate " + str(self.timer_parameter_input) + "hz for domain: "+ str(self.domain_parameter_input))

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
    
    def get_current_timestamp(self):
        current_time = datetime.datetime.now()
        time_stamp = current_time.timestamp()
        return time_stamp


    def cam_image_relay_listener_callback(self, data):
        data.subscribe_time = float(self.get_current_timestamp()) 
        self.data_frame_receiver +=1
        self.data_array.append(['Frame_Number: '+ str(self.data_frame_receiver), data.publish_time, data.subscribe_time])
        if self.data_frame_receiver % 50 == 0 :
            self.write_csv_from_dataframe(self.data_array, self.csv_file_path)
            self.data_array.clear()
        if self.data_frame_publisher % 151 == 0 :
            self.write_counter_to_csv(self.data_frame_publisher,  self.data_frame_receiver, self.csv_file_path)
            raise SystemExit 
            # self.get_logger().info("--------------------------------------------")
            # self.get_logger().info("+++++++++++++++++++++++++++++++++++++++++++++")
        self.get_logger().info('Binary Files are Published at timestamp: ' + str(data.publish_time) +' Subscribed at: ' + str(data.subscribe_time))

# for this method we need a way as soon as it changes the directory to stop the
# process and start the new image size sending as well saving the csv file according to 
# the name of the directory.
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

    def write_counter_to_csv(self, publisher_counter, received_counter, csv_file_path):
        data_frame = pd.read_csv(self.csv_file_path)
        data_frame['Published_Number_of_Frame'] = publisher_counter
        data_frame['Received_Number_Of_Frame'] = received_counter
        data_frame.to_csv(csv_file_path, index=False, header=True)

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