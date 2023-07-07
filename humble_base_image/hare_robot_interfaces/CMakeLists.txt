cmake_minimum_required(VERSION 3.5)
project(hare_robot_interfaces)


# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
#find_package(example_interfaces REQUIRED)


rosidl_generate_interfaces(${PROJECT_NAME}
"msg/HardwareStatus.msg"
"msg/LedStateArray.msg"
"msg/Turtle.msg"
"msg/TurtleArray.msg"
"msg/Header.msg"
"msg/Image.msg"
"msg/Binaryfile.msg"
"msg/StringTimestamp.msg"
"msg/Imu.msg"
"srv/SetLed.srv"
"srv/CatchTurtle.srv"
DEPENDENCIES builtin_interfaces std_msgs sensor_msgs
# example_interfaces
)
# add the builtin interfaces on the above line if we are adding some previous 
# built in interfaces.

ament_package()
