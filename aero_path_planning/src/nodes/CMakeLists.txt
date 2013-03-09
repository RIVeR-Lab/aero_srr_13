#build nodes
#rosbuild_add_executable(aero_path_planning_test src/nodes/orxy_path_planning_test.cpp src/helpers/Tentacles.cpp src/helpers/OccupancyGrid.cpp)

#Build the Local Planner Node
rosbuild_add_executable(aero_local_planner oryx_local_planner.cpp)
target_link_libraries(aero_local_planner ${PROJECT_NAME}Core ${PROJECT_NAME}OccupancyGrid ${PROJECT_NAME}FitnessQueue)

#Build the Occupancy Generator test node
rosbuild_add_executable(occupancy_generator occupancy_generator.cpp)
target_link_libraries(occupancy_generator ${PROJECT_NAME}Core ${PROJECT_NAME}OccupancyGrid)

#Build the Oryx Global Panner node
rosbuild_add_executable(oryx_global_planner oryx_global_planner.cpp)
target_link_libraries(oryx_global_planner ${PROJECT_NAME}Core ${PROJECT_NAME}OccupancyGrid ${PROJECT_NAME}Planners)

#Build the Aero global planner node
rosbuild_add_executable(aero_global_planner aero_global_planner.cpp)
target_link_libraries(aero_global_planner ${PROJECT_NAME}Core ${PROJECT_NAME}OccupancyGrid ${PROJECT_NAME}Planners)