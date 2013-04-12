#Build the detection manager library
rosbuild_add_library(Detman DetectionManager.cpp)
#rosbuild_link_boost(Detman thread)

add_subdirectory(vision_pipeline)