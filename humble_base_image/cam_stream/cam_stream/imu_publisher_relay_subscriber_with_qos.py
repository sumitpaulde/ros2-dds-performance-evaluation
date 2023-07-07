import rclpy
from rclpy.node import Node
from hare_robot_interfaces.msg import Imu
from builtin_interfaces.msg import Time
import os
import pandas as pd
import datetime
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy

class ImuPublisherRelaySubscriber(Node):
    
    def __init__(self):
        super().__init__("imu_publisher_relay_subscriber_with_qos")
        self.data_frame_counter = 0
        self.data_array = []
        self.file_path_parameter = self.declare_parameter("filepath")
        self.file_path_parameter_input = self.get_parameter("filepath").value
        self.filepath =  self.file_path_parameter_input
        self.timer_parameter = self.declare_parameter("hz")
        self.timer_parameter_input = self.get_parameter("hz").value
        self.timer = self.create_timer(self.hz_to_second(self.timer_parameter_input), self.publish_imu_callback)
        self.home_path = os.path.expanduser('~')
        self.reliability_parameter = self.declare_parameter("reliability", "RELIABLE")
        self.reliability_input = self.get_parameter("reliability").value
        self.durability_parameter = self.declare_parameter("durability", "VOLATILE")
        self.durability_input = self.get_parameter("durability").value
        self.dds_implementation_parameter = self.declare_parameter("dds", "cyclone")
        self.dds_input = self.get_parameter("dds").value
        self.imu_list = self.read_imu_file_from_local(self.filepath)
        self.get_logger().info('imu_publisher_relay_subscriber has started')
        self.subscription = self.create_subscription(Imu, "imu_data_relay", self.imu_data_listener_callback, qos_profile=self.get_qos_profile_settings(self.reliability_input, self.durability_input))
        self.imu_publisher = self.create_publisher( Imu, "imu_data", qos_profile=self.get_qos_profile_settings(self.reliability_input, self.durability_input))

    def publish_imu_callback(self):
        
        imu_msg = Imu()
        imu_msg.orientation_covariance = [float(self.imu_list[self.data_frame_counter][1]), float(self.imu_list[self.data_frame_counter][2]) , float(self.imu_list[self.data_frame_counter][3]) ]
        imu_msg.angular_velocity_covariance = [float(self.imu_list[self.data_frame_counter][4]), float(self.imu_list[self.data_frame_counter][5]), float(self.imu_list[self.data_frame_counter][6]) ]
        frame_id = str(self.data_frame_counter)
        imu_msg.header.frame_id = frame_id
        time_stamp_imu = self.imu_list[self.data_frame_counter][0]
        time_stamp_list = time_stamp_imu.split(".")
        time_stamp = Time()
        time_stamp.sec = int(time_stamp_list[0])
        time_stamp.nanosec = int(time_stamp_list[1])
        imu_msg.header.stamp = time_stamp
        self.data_frame_counter +=1
        if self.data_frame_counter <= 10550:
            imu_msg.publish_time = float(self.get_current_timestamp())
            self.imu_publisher.publish(imu_msg)
        else:
            self.data_frame_counter = 0

        
    
    def imu_data_listener_callback(self, data):
        relay_msg = Imu()
        subscribe_time = float(self.get_current_timestamp())
        self.get_logger().info('Subscribed to the imu_data')
        frame_id = data.header.frame_id
        publish_time = data.publish_time
        file_name = "imu_data_" + str(self.timer_parameter_input)  +"_hz_latency_data.csv"
        csv_folder_location = self.home_path + os.sep +"experiment_results"+ os.sep + str((self.dds_input).lower()) + os.sep + "imu" + os.sep + \
         str(self.reliability_input[0]) + "_" + str(self.durability_input[0])
        os.makedirs(csv_folder_location, exist_ok=True)
        file_path = csv_folder_location + os.sep  + file_name
        self.data_array.append(['Frame_Number: '+ str(frame_id), publish_time, subscribe_time])
        if self.data_frame_counter % 100 == 0 :
          self.write_csv_from_dataframe(self.data_array, file_path )
          self.data_array.clear()
          self.get_logger().info("--------------------------------------------")
          self.get_logger().info("+++++++++++++++++++++++++++++++++++++++++++++")
        

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

    def write_csv_from_dataframe(self, data_array, file_path_input):
      columns_input=['Frame_Number','Publish_Timestamp_Publisher', 'Subscribe_Timestamp_Publisher']
      
      if os.path.exists(file_path_input) == False :
          data_frame = pd.DataFrame(data_array, columns=columns_input)
      # data_frame.set_index('Frame_Number')
          data_frame.to_csv(file_path_input)
      else :
          data_frame = pd.DataFrame(data_array)
        # data_frame.set_index('Frame_Number')
          data_frame.to_csv(file_path_input, mode= 'a', header=False)
    
    def hz_to_second(self, hz_input):
        timer_input = float(1 / hz_input)
        return timer_input
    
    def get_qos_profile_settings(self, reliability_input, durability_input):
        reliability = ''
        durability = ''

        if reliability_input.upper() == "RELIABLE":
              reliability = QoSReliabilityPolicy.RELIABLE
        else :
              reliability = QoSReliabilityPolicy.BEST_EFFORT

        if durability_input.upper() == "VOLATILE":
             durability = QoSDurabilityPolicy.VOLATILE
        else:
             durability = QoSDurabilityPolicy.TRANSIENT_LOCAL               
              
         
        qos_profile = QoSProfile(reliability=reliability, durability=durability, history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1)
        return qos_profile
            

def main(args=None):
# this line is required to initialize the ros communication.
    rclpy.init(args= args)
    node = ImuPublisherRelaySubscriber()
    rclpy.spin(node)
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()