#build library for non-node sources
rosbuild_add_library(${PROJECT_NAME}Core
		      GlobalPlanner.cpp
		      LocalPlanner.cpp
		      AeroPathPlanningUtilities.cpp)
rosbuild_add_library(${PROJECT_NAME}OccupancyGrid
		      OccupancyGrid.cpp
		      Tentacles.cpp)      
rosbuild_add_library(${PROJECT_NAME}Planners 
		      RRTCarrot.cpp AStarCarrot.cpp)
		      
target_link_libraries(${PROJECT_NAME}Core ${PROJECT_NAME}OccupancyGrid ${PROJECT_NAME}Planners )
