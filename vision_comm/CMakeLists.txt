cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

include_directories(${PROJECT_SOURCE_DIR}/include)
find_package(OpenCV REQUIRED)

#common commands for building c++ executables and libraries
rosbuild_add_library(vision_comm src/IOImages.cpp src/RosBridge.cpp src/RosStereoBridge.cpp src/MonoCam.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(testMonoCam example/monocam.cpp)
rosbuild_add_executable(testRosCam example/rosbridge.cpp)
rosbuild_add_executable(pubcam example/pubcam.cpp)
#rosbuild_add_executable(testStereoCam example/stereocam.cpp)
rosbuild_add_executable(testRosStereoCam example/rosstereobridge.cpp)
#target_link_libraries(testMonoCam vision_comm)
target_link_libraries(testRosCam vision_comm)
target_link_libraries(pubcam vision_comm)
#target_link_libraries(testStereoCam vision_comm)
target_link_libraries(testRosStereoCam vision_comm)
