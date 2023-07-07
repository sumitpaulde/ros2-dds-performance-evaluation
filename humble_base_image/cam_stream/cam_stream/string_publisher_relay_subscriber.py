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
        super().__init__("string_publisher_relay_subscriber")
        self.get_logger().info("Sample string publisher node started")
        self.string_input = self.declare_parameter("input")
        self.string_parameter_input = self.get_parameter("input").value
        self.timer_parameter = self.declare_parameter("hz")
        self.timer_parameter_input = self.get_parameter("hz").value
        self.domain_parameter = self.declare_parameter("domain", "different_domain")
        self.domain_parameter_input = self.get_parameter("domain").value
        self.timer = self.create_timer(self.hz_to_second(self.timer_parameter_input), self.publish_string_callback)
        self.data_frame_receiver = 0
        self.data_frame_publisher = 0
        self.home_path = os.path.expanduser('~')
        self.data_array = []
        self.subscription = self.create_subscription( StringTimestamp, "string_data_relay", self.string_relay_listener_callback, 1)
        self.string_publisher = self.create_publisher( StringTimestamp, "string_data", 1)
        file_name = "string_" + str(len(self.string_parameter_input )) + "_char_"+ str(self.timer_parameter_input) + "hz_latency_data.csv"
        self.csv_folder_location = self.home_path + os.sep +"experiment_results" + os.sep + "string_latency" + os.sep + self.domain_parameter_input + os.sep + str(self.timer_parameter_input) + "hz"
        os.makedirs(self.csv_folder_location, exist_ok=True)
        self.csv_file_path = self.csv_folder_location + os.sep  + file_name
        self.get_logger().info("csv_file_path= "+ self.csv_file_path)


    def publish_string_callback(self):
        msg = StringTimestamp()
        input_string = String()
        input_string.data = self.string_parameter_input
        msg.input = input_string
        msg.publish_time = self.get_current_timestamp()
        self.string_publisher.publish(msg)
        self.data_frame_publisher += 1
        self.get_logger().info('Sample String is Publishing at timestamp: ' + str(msg.publish_time))

    
    def get_current_timestamp(self):
        current_time = datetime.datetime.now()
        time_stamp = current_time.timestamp()
        return time_stamp


    def string_relay_listener_callback(self, data):
        subscribe_time = float(self.get_current_timestamp()) 
        self.data_frame_receiver +=1
        self.data_array.append(['String_MSG_Number: '+ str(self.data_frame_receiver), data.publish_time, subscribe_time])
        if self.data_frame_receiver % 50 == 0 :
            self.write_csv_from_dataframe(self.data_array, self.csv_file_path)
            self.data_array.clear()
        if self.data_frame_publisher % 151 == 0 :
            self.write_counter_to_csv(self.data_frame_publisher,  self.data_frame_receiver, self.csv_file_path)
            raise SystemExit 
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

    def write_counter_to_csv(self, publisher_counter, received_counter, csv_file_path):
        data_frame = pd.read_csv(self.csv_file_path)
        data_frame['Published_Number_of_Frame'] = publisher_counter
        data_frame['Received_Number_Of_Frame'] = received_counter
        data_frame.to_csv(csv_file_path, index=False, header=True)
    
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