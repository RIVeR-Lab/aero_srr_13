rosbuild_add_executable(ros2cv ros2cv.cpp)
target_link_libraries(ros2cv ${OpenCV_LIBS})