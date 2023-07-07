import rclpy
from rclpy.node import Node 
from hare_robot_interfaces.msg import StringTimestamp
from std_msgs.msg import String
import datetime
import pandas as pd
import os
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy



class StringPublisherRelaySubscriber(Node):
    
    def __init__(self):
        super().__init__("string_publisher_relay_subscriber_with_qos")
        self.get_logger().info("Sample string publisher node started")
        self.string_input = self.declare_parameter("input")
        self.string_parameter_input = self.get_parameter("input").value
        self.timer_parameter = self.declare_parameter("hz")
        self.timer_parameter_input = self.get_parameter("hz").value
        self.reliability_parameter = self.declare_parameter("reliability", "RELIABLE")
        self.reliability_input = self.get_parameter("reliability").value
        self.durability_parameter = self.declare_parameter("durability", "VOLATILE")
        self.durability_input = self.get_parameter("durability").value
        self.dds_implementation_parameter = self.declare_parameter("dds", "cyclone")
        self.dds_input = self.get_parameter("dds").value
        self.timer = self.create_timer(self.hz_to_second(self.timer_parameter_input), self.publish_string_callback)
        self.data_frame_counter = 0
        self.home_path = os.path.expanduser('~')
        self.data_array = []
        self.subscription = self.create_subscription( StringTimestamp, "string_data_relay", self.string_relay_listener_callback, qos_profile=self.get_qos_profile_settings(self.reliability_input, self.durability_input))
        self.string_publisher = self.create_publisher( StringTimestamp, "string_data", qos_profile=self.get_qos_profile_settings(self.reliability_input, self.durability_input))


    def publish_string_callback(self):
        msg = StringTimestamp()
        input_string = String()
        input_string.data = self.string_parameter_input
        msg.input = input_string
        msg.publish_time = self.get_current_timestamp()
        self.string_publisher.publish(msg)
        self.get_logger().info('Sample String is Publishing at timestamp: ' + str(msg.publish_time))

    
    def get_current_timestamp(self):
        current_time = datetime.datetime.now()
        time_stamp = current_time.timestamp()
        return time_stamp


    def string_relay_listener_callback(self, data):
        subscribe_time = float(self.get_current_timestamp()) 
        self.data_frame_counter +=1
        self.data_array.append(['String_MSG_Number: '+ str(self.data_frame_counter), data.publish_time, subscribe_time])
        file_name = "string_" + str(len(self.string_parameter_input )) + "_char_"+ str(self.timer_parameter_input) + "hz_latency_data.csv"
        csv_folder_location = self.home_path + os.sep +"experiment_results"+ os.sep + str((self.dds_input).lower()) + os.sep + "string" + os.sep + \
         str(self.reliability_input[0]) + "_" + str(self.durability_input[0])+ os.sep + str(self.timer_parameter_input) + "hz" 
        os.makedirs(csv_folder_location, exist_ok=True)
        csv_file_path = csv_folder_location + os.sep  + file_name
        if self.data_frame_counter % 100 == 0 :
            self.write_csv_from_dataframe(self.data_array, csv_file_path)
            self.data_array.clear()
            self.get_logger().info("--------------------------------------------")
            self.get_logger().info("+++++++++++++++++++++++++++++++++++++++++++++")
        self.get_logger().info('Strings are Published at timestamp: ' + str(data.publish_time) +' Subscribed at: ' + str(subscribe_time))


    def write_csv_from_dataframe(self, data_array, csv_file_path):
        columns_input=['Frame_Number','Publish_Time', 'Subscribe_Time']
        if os.path.exists(csv_file_path) == False :
            data_frame = pd.DataFrame(data_array, columns=columns_input)
      # data_frame.set_index('Frame_Number')
            data_frame.to_csv(csv_file_path)
        else :
            data_frame = pd.DataFrame(data_array)
        # data_frame.set_index('Frame_Number')
            data_frame.to_csv(csv_file_path, mode= 'a', header=False)
    
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
    rclpy.init(args= args)
    node = StringPublisherRelaySubscriber()
    rclpy.spin(node)
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()