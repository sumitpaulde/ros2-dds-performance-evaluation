# ROS2 DDS performance evaluation framework while facilitating Cooperative Driving in Autonomous Vehicle

## Use Case
<p align="justify">
  For our research, we have focused on the cooperative perception of the autonomous vehicle use case.The cooperative perception paradigm depends on exchanging the sensor, camera, or radio device data via wireless communication to extend the sensing capabilities over the line-of-sight and field-of-view for automotive vehicles. According to the received data, the vehicle can make some driving decisions like, sudden obstacles, overtaking, avoiding hidden barriers, etc. reference paper-<a href="https://ieeexplore.ieee.org/document/6232187">Multiple vehicle driving control for traffic flow efficiency</a>. In the paper <a href="https://ieeexplore.ieee.org/document/9041020" title="A Self-Driving Car Architecture in ROS2">A Self-Driving Car Architecture in ROS2</a>, researchers have presented the autonomous driving software architecture based on Robot Operating System 2 and evaluated the usability of a ROS2 node functioning as an autonomous vehicle during multiple automated driving scenarios. For this purpose, an automated vehicle is equipped with different types of sensor systems like IMU, lidar, V2X, radar, and GNSS. Many ROS2 topics are required to exchange tons of sensor data with various components of an automated vehicle node and other automated vehicle nodes. As DDS is the middleware of the ROS2 system, the performance of the DDS is a crucial factor while exchanging the continuous stream of data. With domain abstraction, ROS2 nodes can create logical partitions according to the data or subscriber types, making the dynamic discovery within a domain easier and faster. Unfortunately, there is a limitation on the number of participants for a single domain, and the number of DDS topics is not unlimited. Multiple domain data exchanges are required to implement the cooperative perception paradigm of autonomous vehicles. How the data exchange between automated vehicles (ROS2 nodes) behaves while the nodes are participants in different domains is still unknown. To depict the data exchange among automated vehicles with various sensor data types, we have used wirelessly connected different physical ROS2 nodes for our experiments. We assume the ROS2 nodes entail no latency while reading and publishing the various types of sensor data to different ros2 topics.
</p>


![cooperative_driving](https://github.com/sumitpaulde/ros2-dds-performance-evaluation/assets/62351460/d3b830e1-5097-4ec3-9a5e-121a4783decc)

## Installation

### Prerequisite
To install the evaluation framework on Ubuntu or Windows machine, please install the prerequisite libraries or softwares.

1. git.
2. docker.
3. docker-compose.

### Download the DDS-implementations custom installers
To run the experiments with ROS2-Humble we had to make some changes for the eProsima and RTI Connext DDS installers. eProsima DDS still now does not have a new release for Humble compatibility, and RTI Connext does not provide the rmw installer for the ARM architecture. So for both of these DDS implementations, we have made some changes to the installer files and kept those in the [shared folder](https://tubcloud.tu-berlin.de/s/jC93LXrt9ScKF7r).

### Build
1. Like any other GitHub repo, you can clone the repo.
1. Make sure to download the previously mentioned installer.
1. For any DDS implementation build, you have to build the humble_base_image first.
1. <code>cd humble_base_image</code>
1. on the terminal, run <code>docker-compose build</code>
1. Copy DDS-Installers
    * eProsima: For the eProsima humble build, you have to copy the eProsima_Fast-DDS-v2.7.1.tar and src.tar file in the [eprosima_humble](https://tubcloud.tu-berlin.de/s/jC93LXrt9ScKF7r) directory and un-tar both files.
    * RTI Connext Laptop: For the RTI Connext laptop, build copy the rti_connext_dds-6.1.1-lm-x64Linux4gcc7.3.0.run file from [rti_laptop](https://tubcloud.tu-berlin.de/s/jC93LXrt9ScKF7r) folder and keep it in rti_connext_laptop directory.
    *  RTI Connext ARM: Download the rti_connext_dds_6.1.1_arm.tar.gz file from the [rti_arm folder](https://tubcloud.tu-berlin.de/s/jC93LXrt9ScKF7r) and keep it in the rti_connext_arm directory without un-tar.
1. Now, to build any of the docker images of the DDS implementations, use cd to the respective directory and run <code>docker-compose build</code>

## Running Experiments
To run the experiments, you have to first run the docker images of the DDS implementations in the previous step. To run the docker images, please use the bellow mentioned commands.

#### Cyclone DDS Publisher
      docker run -dit -p 7447:7447 --net=host --privileged  <docker_image_cyclone>
#### Cyclone DDS Subscriber
      docker run -dit -p 7448:7448 --net=host --privileged  <docker_image_cyclone>
#### eProsima and RTI Connext
      docker run -d --net=host -ti  --privileged <docker_image_dds_implementation>

#### Running RTT(Round Trip Time) experiment for Binary File type
We have added <code> experiment_dds_performance_script_binary.sh, experiment_dds_performance_script_string.sh, experiment_dds_performance_script_imu.sh </code> scripts for running the experiments for different file types.

* **Run Docker Container:** Start the docker containers on both **Publisher** and **Subscriber** physical machines.
* use  <code> docker exec -it <container_id> /bin/bash </code> to enter the running container on two terminals on each machine.
* **Start Bridging Service:** As we will have experiments for both the same domain and different domain communication, for different domain communication, we will need respective bridging services to start before the experiment. Run the bridging service on one terminal of each machine.
    1. **cyclone dds publisher**: <code> ~/ros2_ws/src/zenoh-plugin-dds/target/release/zenoh-bridge-dds -d 5 -e tcp/<IP_address_of_subscriber>:7448</code> <br>
       **cyclone dds subscriber**: <code> ~/ros2_ws/src/zenoh-plugin-dds/target/release/zenoh-bridge-dds -d 10 -e tcp/<IP_address_of_publisher>:7447</code> <br>
    1. **eProsima dds**: <code>source /root/is_workspace/install/setup.bash</code> <br>
        <code>integration-service /root/is_workspace/binary_data.yaml</code> <br>
    1. **rti_connext_laptop**: <code>cd /root/rti_workspace/y/rti_connext_dds-6.1.1 && bin/rtiroutingservice -cfgFile /root/rti_workspace/y/rti_connext_dds-6.1.1/camera_stream_bridge.xml -cfgName domain5to10</code> <br>
       **rti_connext_arm**: <code> cd ~/rti_workspace/rti_connext_dds_6.1.1_arm/ && bin/rtiroutingservice -cfgFile ~/rti_workspace/camera_stream_bridge.xml -cfgName domain5to10</code> <br>

* **Subscriber Node:** On the subscriber node, start the relay binary publisher ros2 node by the command <br>
<code>RMW_IMPLEMENTATION=<rmw_implementation_value> ROS_DOMAIN_ID=10 ros2 run cam_stream binary_subscriber_relay_publisher</code> <br>
the rmw_implementation_value can be <code> rmw_cyclonedds_cpp, rmw_fastrtps_cpp, rmw_connextdds  </code> according to the DDS implementation.
* **Publisher Node:** Now you can start the script for the Binary file experiment by running <code>bash ./experiment_dds_performance_script_binary.sh</code>
* As soon as the script starts, it will ask you to choose a rmw for the experiment, please choose by typing 1, 2 or 3.<br>
  ![14](https://github.com/sumitpaulde/ros2-dds-performance-evaluation/assets/62351460/21dd1a40-63f7-4e93-8b41-421b35548888)
* To confirm that you have already started the bridging service, the script will remind you with a question, if you already have the bridging service on, please press "y". <br>
  ![12](https://github.com/sumitpaulde/ros2-dds-performance-evaluation/assets/62351460/7965212b-0270-46fc-a494-0a9a9edc7935)

**Note:** To run the experiments for other data types, you have to change the bridging service inputs for the ePrsoima DDS and RTI Connext DDS. <br>
For string data type, for example: <br>
  On the Publisher node <code>bash ./experiment_dds_performance_script_string.sh</code> <br>
  **eProsima DDS:** <code>integration-service /root/is_workspace/**string_data.yaml**</code> <br>
  **RTI Connext DDS:** <code>  bin/rtiroutingservice -cfgFile ~/rti_workspace/**string_stream_bridge.xml** -cfgName domain5to10</code> <br>

#### Running intermediate file processing time experiment for Binary File type
* To run the intermediate file processing time experiment, you have to follow the previously mentioned steps, run docker container, and start bridging service. <br>
* **Publisher Node:** <code>bash ./transfer_rate_publisher.sh</code> <br>
 Please choose the rmw and the file type for the publisher.
* **Subscriber Node** <code>bash ./intermidiate_process_time_calculator.sh</code>
  Please choose the rmw and the file type for the subscriber.

Similar way, you can run the experiments for the string and IMU file type and also with the transfer rate calculator experiments.

##### Draw the experiment output figure 
To draw the diagram from the result of the experiments, we have added some scripts in the diagram_scripts folder. <br>
![cyclone_pi3_wireless_low](https://github.com/sumitpaulde/ros2-dds-performance-evaluation/assets/62351460/e6e6dd8f-21ee-4f5d-886c-26f74a103dee)
To find the behavior of the same domain vs. different domain for the binary files, we have used the following command <br>
<code>python3 sd_vs_dd_same_dds.py /home/spaul/experiment_results/pi3/cyclone/wireless/interprocess/binary/different_domain/ /home/spaul/experiment_results/pi3/cyclone/wireless/interprocess/binary/same_domain/ /home/spaul/experiment_results/pi3/cyclone/wireless/binary_latency/different_domain /home/spaul/experiment_results/pi3/cyclone/wireless/binary_latency/same_domain cyclone_wireless_pi3 2hz 4hz 8hz</code> <br>

Here for Raspberry Pi3 wirelessly connected nodes
* Arg 1: interprocess time experiment result file location for different domain wirelessly connected nodes.
* Arg 2: interprocess time experiment result file location for same domain wirelessly connected nodes.
* Arg 3: RTT experiment result file location different domain wirelessly connected nodes.
* Arg 4: RTT experiment result file location for same domain wirelessly connected nodes.
* Arg 5: dds, machine, and connectivity type, for example, cyclone_wireless_pi3.
* Arg 6,7,8: Required frequencies to show in the plot as 2Hz, 4Hz, 8Hz.

  

















<code></code>





