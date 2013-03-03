#Build the odometry node
rosbuild_add_executable(aero_odometry aero_odometry.cpp)
target_link_libraries(aero_odometry ${PROJECT_NAME}Filters)