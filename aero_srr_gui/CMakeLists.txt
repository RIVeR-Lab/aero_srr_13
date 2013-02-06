##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
project(aero_srr_gui)

##############################################################################
# Ros Initialisation
##############################################################################


find_package(catkin REQUIRED COMPONENTS roscpp rospy oryx_msgs)
find_package(Qt4 REQUIRED QtCore QtGui)

##############################################################################
# Qt Environment
##############################################################################

INCLUDE(${QT_USE_FILE})
ADD_DEFINITIONS(${QT_DEFINITIONS})

##############################################################################
# Sections
##############################################################################

file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/oryx_srr_gui/*.hpp)

QT4_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT4_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT4_WRAP_CPP(QT_MOC_HPP ${QT_MOC})

##############################################################################
# Sources
##############################################################################

file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)

##############################################################################
# Configure catkin package
##############################################################################

catkin_package(CATKIN_DEPENDS roscpp rospy oryx_msgs)

##############################################################################
# Binaries
##############################################################################

include_directories(include ${QT_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
include_directories(${PROJECT_BINARY_DIR}) #needed to find generated ui files...
add_executable(oryx_srr_gui ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
target_link_libraries(oryx_srr_gui ${QT_LIBRARIES} ${catkin_LIBRARIES})
