rosbuild_add_library(AStarCarrot AStarCarrot.cpp)
rosbuild_add_library(RRTCarrot   RRTCarrot.cpp)
target_link_libraries(AStarCarrot OccupancyGrid AeroUtilities)
target_link_libraries(RRTCarrot   OccupancyGrid AeroUtilities)