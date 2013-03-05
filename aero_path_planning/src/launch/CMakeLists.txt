#Set all parameters here
#Parameters should have 3 set()'s: set(X_ARG arg_literal), set(X_CDEF DEF), set(X_VAL, value)

#Parameter for the topic for exchanging OccupancyGrids between the global and local planner
set(OCCUPANCY_TOPIC_ARG aero/occupancy_topic)
set(OCCUPANCY_TOPIC_CDF OCCUPANCY_TOPIC)
set(OCCUPANCY_TOPIC_VAL "\"aero/occupancy_topic\"")

#Parameter defining what platform we're on
set(PLATFORM_ARG aero/platform)
set(PLATFORM_CDF PLATFORM)
set(PLATFORM_VAL "\"0\"")

#Parameter defining the velocity command topic
set(VELOCITY_CMD_TOPIC_ARG aero/velocity_command_topic)
set(VELOCITY_CMD_TOPIC_CDF VEL_CMD_TOPIC)
set(VELOCITY_CMD_TOPIC_VAL "\"aero/velocity_command_topic\"")

#Parameters for defining tentacle selection huristics
set(GOAL_WEIGHT_ARG aero/goal_weight)
set(GOAL_WEIGHT_CDF GOAL_WEIGHT)
set(GOAL_WEIGHT_VAL "\"2.0\"")
set(TRAV_WEIGHT_ARG aero/traversed_weight)
set(TRAV_WEIGHT_CDF TRAV_WEIGHT)
set(TRAV_WEIGHT_VAL "\"0.1\"")
set(DIFF_WEIGHT_ARG aero/difficult_weight)
set(DIFF_WEIGHT_CDF DIFF_WEIGHT)
set(DIFF_WEIGHT_VAL "\"0.25\"")

#Build the parameter header
configure_file (
  "AeroPathPlanningParameters.h.in"
  "${PROJECT_BINARY_DIR}/AeroPathPlanningParameters.h"
  )

#Add the launch file subdirs
add_subdirectory(aero_global_planner)
add_subdirectory(aero_local_planner)
add_subdirectory(aero_path_planning)
