rosbuild_add_executable(ros2cv ros2cv.cpp)
add_executable(objDet objectDetection2.cpp)
add_executable(cap frameSave.cpp)
rosbuild_add_executable(test_tf_tree test_tf_tree.cpp)


target_link_libraries(ros2cv ${OpenCV_LIBS} ${PROJECT_NAME}_detman)
target_link_libraries(objDet ${OpenCV_LIBS})
target_link_libraries(cap ${OpenCV_LIBS})

