rosbuild_add_executable(aero_laser_filter aero_laser_filter.cpp)

rosbuild_add_library(ConversionStage   ConversionStage.cpp)
rosbuild_add_library(LocalFilterStage  LocalFilterStage.cpp)
rosbuild_add_library(GlobalFilterStage GlobalFilterStage.cpp)