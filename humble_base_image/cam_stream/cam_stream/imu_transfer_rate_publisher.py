import rclpy
from rclpy.node import Node
from hare_robot_interfaces.msg import Imu
from builtin_interfaces.msg import Time
import os
import pandas as pd
import datetime
import time

class ImuTransferRatePublisher(Node):
    
    def __init__(self):
        super().__init__("imu_transfer_rate_publisher")
        self.data_frame_receiver = 0
        self.data_frame_publisher = 1
        self.data_array = []
        self.file_path_parameter = self.declare_parameter("filepath")
        self.file_path_parameter_input = self.get_parameter("filepath").value
        self.filepath =  self.file_path_parameter_input
        self.timer_parameter = self.declare_parameter("hz", 1)
        self.timer_parameter_input = self.get_parameter("hz").value
        self.timer = self.create_timer(self.hz_to_second(self.timer_parameter_input), self.publish_imu_callback)
        self.home_path = os.path.expanduser('~')
        self.domain_parameter = self.declare_parameter("domain", "different_domain")
        self.domain_parameter_input = self.get_parameter("domain").value
        self.get_logger().info('imu_publisher for transfer file has been started')
        self.imu_publisher = self.create_publisher( Imu, "imu_data", 1)
        self.subscription = self.create_subscription(Imu, "imu_data_relay", self.imu_data_listener_callback, 1)
        self.imu_list = self.read_imu_file_from_local(self.filepath)



    def publish_imu_callback(self):
        
        imu_msg = Imu()
        imu_msg.orientation_covariance = [float(self.imu_list[self.data_frame_receiver][1]), float(self.imu_list[self.data_frame_receiver][2]) , float(self.imu_list[self.data_frame_receiver][3]) ]
        imu_msg.angular_velocity_covariance = [float(self.imu_list[self.data_frame_receiver][4]), float(self.imu_list[self.data_frame_receiver][5]), float(self.imu_list[self.data_frame_receiver][6]) ]
        frame_id = str(self.data_frame_publisher)
        imu_msg.header.frame_id = frame_id
        time_stamp_imu = self.imu_list[self.data_frame_receiver][0]
        time_stamp_list = time_stamp_imu.split(".")
        time_stamp = Time()
        time_stamp.sec = int(time_stamp_list[0])
        time_stamp.nanosec = int(time_stamp_list[1])
        imu_msg.header.stamp = time_stamp
        self.data_frame_publisher += 1

        if self.data_frame_publisher == 1000:
            imu_msg.publish_time = float(self.get_current_timestamp())
            self.imu_publisher.publish(imu_msg)
            raise SystemExit
        
        if self.data_frame_publisher <1000:
            imu_msg.publish_time = float(self.get_current_timestamp())
            self.imu_publisher.publish(imu_msg)

        else:
            self.data_frame_publisher = 0

    def imu_data_listener_callback(self, data):
        self.get_logger().info('Imu data is receiving')
        
        if data.header.frame_id == str(000) or data.publish_time == 1.1111:
            self.get_logger().info("--------------------------------------------")
            self.get_logger().info("+++++++++++++++++++++++++++++++++++++++++++++")
            time.sleep(5)
            raise SystemExit



    def read_imu_file_from_local(self, filepath):
        imu_np_list = []
        with open(filepath) as f:
            for line in f:
                imu_list = line.split()
                imu_np_list.append(imu_list)
        return imu_np_list



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
    node = ImuTransferRatePublisher()
    rclpy.spin(node)
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()