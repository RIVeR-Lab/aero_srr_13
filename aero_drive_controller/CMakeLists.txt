cmake_minimum_required(VERSION 2.8.3)
project(aero_drive_controller)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs message_generation oryx_msgs actionlib_msgs actionlib geometry_msgs)
find_package(Boost REQUIRED COMPONENTS thread)


#generate actions and msgs
add_message_files(DIRECTORY msg FILES VelocityArc.msg VelocityTranslate.msg)
add_action_files(DIRECTORY action FILES TranslateCommand.action VelocityCommand.action)
generate_messages(DEPENDENCIES std_msgs oryx_msgs actionlib_msgs)


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

catkin_package(CATKIN_DEPENDS roscpp message_runtime oryx_msgs actionlib_msgs actionlib geometry_msgs)


#Link all source folders so that Headers will be properly found
include_directories("${PROJECT_BINARY_DIR}")
include_directories("${PROJECT_SOURCE_DIR}/include")
include_directories("${PROJECT_SOURCE_DIR}/src/nodes")
include_directories("${PROJECT_SOURCE_DIR}/src/velocity_command")
include_directories("${PROJECT_SOURCE_DIR}/src/translate_command")
include_directories("${PROJECT_SOURCE_DIR}/src/controllers")
include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

#Add non-node library files to be build
add_library(${PROJECT_NAME}_lib
  src/velocity_command/VelocityControlServer.cpp
  src/translate_command/TranslateControlServer.cpp
  src/controllers/ArcDriveController.cpp
)
target_link_libraries(${PROJECT_NAME}_lib ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(${PROJECT_NAME}_lib oryx_drive_controller_gencpp)

#Build Nodes
add_executable(oryx_drive_controller src/nodes/oryx_drive_controller.cpp)
target_link_libraries(oryx_drive_controller ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PROJECT_NAME}_lib)
add_dependencies(oryx_drive_controller oryx_drive_controller_gencpp)

add_executable(oryx_drive_controller_test src/nodes/oryx_drive_controller_action_tests.cpp)
target_link_libraries(oryx_drive_controller_test ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PROJECT_NAME}_lib)
add_dependencies(oryx_drive_controller_test oryx_drive_controller_gencpp)