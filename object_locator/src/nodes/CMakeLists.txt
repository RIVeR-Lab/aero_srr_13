rosbuild_add_executable(lower_detector_node lower_detector_node.cpp)
add_executable(objDet objectDetection2.cpp)
add_executable(cap frameSave.cpp)
rosbuild_add_executable(camera_tf_tree_node camera_tf_tree_node.cpp)
add_executable(gpu gputest.cpp)
add_executable(blob blobDetector.cpp)
add_executable(blob2 blobBox.cpp)
#add_executable(surf Surf.cpp)
add_executable(hough Hough.cpp)



target_link_libraries(lower_detector_node ${OpenCV_LIBS} Detman)
target_link_libraries(objDet ${OpenCV_LIBS})
target_link_libraries(cap ${OpenCV_LIBS})
target_link_libraries(gpu ${OpenCV_LIBS})
target_link_libraries(blob ${OpenCV_LIBS})
target_link_libraries(blob2 ${OpenCV_LIBS})
target_link_libraries(hough ${OpenCV_LIBS})

target_link_libraries (lower_detector_node ${PCL_LIBRARIES})

find_package(PCL 1.6 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#add_executable (pcl pcl.cpp)
#target_link_libraries (pcl ${PCL_LIBRARIES})


