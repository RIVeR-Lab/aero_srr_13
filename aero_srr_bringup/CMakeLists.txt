cmake_minimum_required(VERSION 2.8.3)
project(aero_srr_bringup)

find_package(catkin REQUIRED)

catkin_package(CATKIN_DEPENDS oryx_drive_controller oryx_path_planning)
