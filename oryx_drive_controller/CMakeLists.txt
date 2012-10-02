cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_find_ros_package(actionlib_msgs)
include(${actionlib_msgs_PACKAGE_PATH}/cmake/actionbuild.cmake)
genaction()
rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

#Get the current date
MACRO (TODAY RESULT) 
    IF (WIN32) 
        EXECUTE_PROCESS(COMMAND "cmd" " /C date /T" OUTPUT_VARIABLE ${RESULT}) 
        string(REGEX REPLACE "(..)/(..)/..(..).*" "\\1\\2\\3" ${RESULT} ${${RESULT}}) 
    ELSEIF(UNIX) 
        EXECUTE_PROCESS(COMMAND "date" "+%d/%m/%Y" OUTPUT_VARIABLE ${RESULT}) 
        string(REGEX REPLACE "(..)/(..)/..(..).*" "\\1\\2\\3" ${RESULT} ${${RESULT}}) 
    ELSE (WIN32) 
        MESSAGE(SEND_ERROR "date not implemented") 
        SET(${RESULT} 000000) 
    ENDIF (WIN32) 
ENDMACRO (TODAY) 

TODAY(RESULT) 
message(STATUS "Compilation date = ${RESULT}") 

# The version number.
set (oryx_drive_controller_VERSION_MAJOR 0)
set (oryx_drive_controller_VERSION_MINOR 0)
set (oryx_drive_controller_VERSION_BUILD ${RESULT})

message(STATUS "Building Oryx Drive Controller Version ${oryx_drive_controller_VERSION_MAJOR}.${oryx_drive_controller_VERSION_MINOR}.${oryx_drive_controller_VERSION_BUILD}")

# configure a header file to pass some of the CMake settings
# to the source code to use as build-time configuration parameters
configure_file (
  "${PROJECT_SOURCE_DIR}/src/OryxDriveControllerConfig.h.in"
  "${PROJECT_BINARY_DIR}/OryxDriveControllerConfig.h"
  )

#Link all source folders so that Headers will be properly found
include_directories("${PROJECT_BINARY_DIR}")
include_directories("${PROJECT_SOURCE_DIR}/src/nodes")
include_directories("${PROJECT_SOURCE_DIR}/src/velocity_command")
include_directories("${PROJECT_SOURCE_DIR}/src/translate_command")

#Add the Boost libraries
rosbuild_add_boost_directories()

#Add non-node library files to be build
rosbuild_add_library(${PROJECT_NAME} src/velocity_command/VelocityControlServer.cpp)
rosbuild_add_library(${PROJECT_NAME} src/translate_command/TranslateControlServer.cpp)

#Build Nodes
rosbuild_add_executable(oryx_drive_controller src/nodes/oryx_drive_controller.cpp)
rosbuild_link_boost(oryx_drive_controller thread)