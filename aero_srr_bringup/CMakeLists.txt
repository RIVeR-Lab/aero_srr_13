cmake_minimum_required(VERSION 2.8.3)
project(aero_srr_bringup)

find_package(catkin REQUIRED)

catkin_package(CATKIN_DEPENDS aero_drive_controller aero_path_planning)
