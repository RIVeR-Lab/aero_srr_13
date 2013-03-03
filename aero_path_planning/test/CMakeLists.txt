#build tests
#rosbuild_add_gtest(test/CarrotPathFinderTest test/CarrotPathFinderTest.cpp)
#target_link_libraries(test/CarrotPathFinderTest lib${PROJECT_NAME})

rosbuild_add_gtest(test/AeroPathPlanningTests AeroPathPlanningTests.cpp CastLineTest.cpp RRTCarrotTest.cpp)
target_link_libraries(test/AeroPathPlanningTests ${PROJECT_NAME}Core ${PROJECT_NAME}OccupancyGrid ${PROJECT_NAME}Planners)