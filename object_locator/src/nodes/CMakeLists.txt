rosbuild_add_executable(lower_detector_node lower_detector_node.cpp)
add_executable(objDet objectDetection2.cpp)
add_executable(cap frameSave.cpp)
rosbuild_add_executable(camera_tf_tree_node camera_tf_tree_node.cpp)
add_executable(gpu gputest.cpp)


target_link_libraries(lower_detector_node ${OpenCV_LIBS} Detman)
target_link_libraries(objDet ${OpenCV_LIBS})
target_link_libraries(cap ${OpenCV_LIBS})
target_link_libraries(gpu ${OpenCV_LIBS})
