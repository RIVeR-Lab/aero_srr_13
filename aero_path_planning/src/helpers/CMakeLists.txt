#build library for non-node sources
rosbuild_add_library(${PROJECT_NAME}Core
		      global_planner.cpp
		      AeroPathPlanningUtilities.cpp)
rosbuild_add_library(${PROJECT_NAME}OccupancyGrid
		      OccupancyGrid.cpp
		      Tentacles.cpp)
rosbuild_add_library(${PROJECT_NAME}Planners 
		      RRTCarrot.cpp)