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
rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

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
set (oryx_path_planner_VERSION_MAJOR 0)
set (oryx_path_planner_VERSION_MINOR 0)
set (oryx_path_planner_VERSION_BUILD ${RESULT})

#the verbosity level
#define a verbosity option
set (VERBOSITY OFF)

#set the appropriate flag in the config header
if(VERBOSITY)
	set (oryx_path_planner_VERBOSITY true)
else(VERBOSITY)
	set (oryx_path_planner_VERBOSITY false)
endif(VERBOSITY)

# configure a header file to pass some of the CMake settings
# to the source code to use as build-time configuration parameters
configure_file (
  "${PROJECT_SOURCE_DIR}/include/OryxPathPlannerConfig.h.in"
  "${PROJECT_BINARY_DIR}/OryxPathPlannerConfig.h"
  )

#Link all source folders so that Headers will be properly found
include_directories("${PROJECT_BINARY_DIR}")
include_directories("${PROJECT_SOURCE_DIR}/src")


#include boost directories
rosbuild_add_boost_directories()

#build library for non-node sources
rosbuild_add_library(${PROJECT_NAME} 
		      src/helpers/Tentacles.cpp
		      src/helpers/OccupancyGrid.cpp
		      src/helpers/RRTCarrot.cpp
		      src/helpers/global_planner.cpp
		      src/helpers/AeroPathPlanningUtilities.cpp)



#build nodes
#rosbuild_add_executable(aero_path_planning_test src/nodes/orxy_path_planning_test.cpp src/helpers/Tentacles.cpp src/helpers/OccupancyGrid.cpp)
rosbuild_add_executable(aero_local_planner src/nodes/oryx_local_planner.cpp)
target_link_libraries(aero_local_planner ${PROJECT_NAME})
rosbuild_add_executable(occupancy_generator src/nodes/occupancy_generator.cpp)
target_link_libraries(occupancy_generator ${PROJECT_NAME})
rosbuild_add_executable(oryx_global_planner src/nodes/oryx_global_planner.cpp)
target_link_libraries(oryx_global_planner ${PROJECT_NAME})
rosbuild_add_executable(aero_global_planner src/nodes/aero_global_planner.cpp)
target_link_libraries(aero_global_planner ${PROJECT_NAME})

#build tests
#rosbuild_add_gtest(test/CarrotPathFinderTest test/CarrotPathFinderTest.cpp)
#target_link_libraries(test/CarrotPathFinderTest lib${PROJECT_NAME})

rosbuild_add_gtest(test/AeroPathPlanningTests test/AeroPathPlanningTests.cpp test/CastLineTest.cpp test/RRTCarrotTest.cpp)
target_link_libraries(test/AeroPathPlanningTests ${PROJECT_NAME})
