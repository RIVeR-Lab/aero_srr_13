cmake_minimum_required(VERSION 2.8.3)
project(oryxsrr_msgs)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs message_generation tf oryx_msgs geometry_msgs pcl pcl_ros sensor_msgs)

add_message_files(DIRECTORY msg FILES OccupancyGrid.msg)
add_service_files(DIRECTORY srv FILES Command.srv)

generate_messages(DEPENDENCIES std_msgs oryx_msgs geometry_msgs sensor_msgs)

catkin_package(CATKIN_DEPENDS roscpp std_msgs message_runtime tf oryx_msgs geometry_msgs pcl pcl_ros sensor_msgs)