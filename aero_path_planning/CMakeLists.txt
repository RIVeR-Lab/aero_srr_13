cmake_minimum_required(VERSION 2.8.3)
project(aero_path_planning)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs message_generation oryx_msgs tf aero_drive_controller actionlib_msgs actionlib geometry_msgs pcl pcl_ros aero_srr_msgs sensor_msgs)
find_package(Boost REQUIRED COMPONENTS)

#generate msgs
add_message_files(DIRECTORY msg FILES OccupancyGridMsg.msg)
generate_messages(DEPENDENCIES std_msgs oryx_msgs actionlib_msgs geometry_msgs sensor_msgs)


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
  "${PROJECT_SOURCE_DIR}/src/OryxPathPlannerConfig.h.in"
  "${PROJECT_BINARY_DIR}/OryxPathPlannerConfig.h"
  )

catkin_package(CATKIN_DEPENDS roscpp std_msgs message_runtime oryx_msgs tf aero_drive_controller actionlib_msgs actionlib geometry_msgs pcl pcl_ros aero_srr_msgs sensor_msgs)

#Link all source folders so that Headers will be properly found
include_directories("${PROJECT_BINARY_DIR}")
include_directories("${PROJECT_SOURCE_DIR}/src")
include_directories("${PROJECT_SOURCE_DIR}/include")
include_directories("${PROJECT_SOURCE_DIR}/src/nodes")
include_directories("${PROJECT_SOURCE_DIR}/src/helpers")
include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

set(CMAKE_CXX_FLAGS "-O1")        ## Optimize

#build nodes
add_executable(oryx_path_planning_test src/nodes/orxy_path_planning_test.cpp src/helpers/Tentacles.cpp src/helpers/OccupancyGrid.cpp)
target_link_libraries(oryx_path_planning_test ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(oryx_path_planning_test ${PROJECT_NAME}_gencpp)

add_executable(oryx_local_planner src/nodes/oryx_local_planner.cpp src/helpers/Tentacles.cpp src/helpers/OccupancyGrid.cpp)
target_link_libraries(oryx_local_planner ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(oryx_local_planner ${PROJECT_NAME}_gencpp)

add_executable(occupancy_generator src/nodes/occupancy_generator.cpp src/helpers/OccupancyGrid.cpp)
target_link_libraries(occupancy_generator ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(occupancy_generator ${PROJECT_NAME}_gencpp)

add_executable(oryx_global_planner src/nodes/oryx_global_planner.cpp src/helpers/OccupancyGrid.cpp)
target_link_libraries(oryx_global_planner ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(oryx_global_planner ${PROJECT_NAME}_gencpp)
