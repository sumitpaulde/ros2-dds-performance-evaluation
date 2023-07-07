## ROS2 DDS performance evaluation facilitating Cooperative Driving in Autonomous Vehicle

<p align="justify">
For our research, we have focused on the cooperative perception of the autonomous vehicle use case.The cooperative perception paradigm depends on exchanging the sensor, camera, or radio device data via wireless communication to extend the sensing capabilities over the line-of-sight and field-of-view for automotive vehicles. According to the received data, the vehicle can make some driving decisions like, sudden obstacles, overtaking, avoiding hidden barriers, etc. [[9]](). In the [A Self-Driving Car Architecture in ROS2](https://ieeexplore.ieee.org/document/9041020) paper, researchers have presented the autonomous driving software architecture based on Robot Operating System 2 and evaluated the usability of a ROS2 node functioning as an autonomous vehicle during multiple automated driving scenarios. For this purpose, an automated vehicle is equipped with different types of sensor systems like IMU, lidar, V2X, radar, and GNSS. Many ROS2 topics are required to exchange tons of sensor data with various components of an automated vehicle node and other automated vehicle nodes. As DDS is the middleware of the ROS2 system, the performance of the DDS is a crucial factor while exchanging the continuous stream of data. With domain abstraction, ROS2 nodes can create logical partitions according to the data or subscriber types, making the dynamic discovery within a domain easier and faster. Unfortunately, there is a limitation on the number of participants for a single domain, and the number of DDS topics is not unlimited. Multiple domain data exchanges are required to implement the cooperative perception paradigm of autonomous vehicles. How the data exchange between automated vehicles (ROS2 nodes) behaves while the nodes are participants in different domains is still unknown. To depict the data exchange among automated vehicles with various sensor data types, we have used wirelessly connected different physical ROS2 nodes for our experiments. We assume the ROS2 nodes entail no latency while reading and publishing the various
types of sensor data to different ros2 topics.
</p>


#### Trying the forgotten writing style
![cooperative_driving](https://github.com/sumitpaulde/ros2-dds-performance-evaluation/assets/62351460/d3b830e1-5097-4ec3-9a5e-121a4783decc)
