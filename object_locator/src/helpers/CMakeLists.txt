#Build the detection manager library
rosbuild_add_library(${PROJECT_NAME}_detman DetectionManager.cpp)
#rosbuild_link_boost(${PROJECT_NAME}_detman thread)