#Set all parameters here
#Parameters should have 3 set()'s: set(X_ARG arg_literal), set(X_CDEF DEF), set(X_VAL, value)

#Namespace params
set(R_NS aero)
set(O_NC occupancy)
set(T_NS tentacles)
set(S_NS speed_set)

#Parameter for the topic for exchanging OccupancyGrids between the global and local planner
set(OCCUPANCY_TOPIC_ARG ${R_NS}/occupancy_topic)
set(OCCUPANCY_TOPIC_CDF OCCUPANCY_TOPIC)
set(OCCUPANCY_TOPIC_VAL "\"${R_NS}/occupancy_topic\"")

#Parameter defining the velocity command topic
set(VELOCITY_CMD_TOPIC_ARG ${R_NS}/velocity_command_topic)
set(VELOCITY_CMD_TOPIC_CDF VEL_CMD_TOPIC)
set(VELOCITY_CMD_TOPIC_VAL "\"${R_NS}/velocity_command_topic\"")

#Paramter defining the odometry topic
set(ODOMETRY_TOPIC_ARG ${R_NS}/odometry)
set(ODOMETRY_TOPIC_CDF ODOMETRY_TOPIC)
set(ODOMETRY_TOPIC_VAL "\"${R_NS}/odometry\"")

#Parameter defining what platform we're on
set(PLATFORM_ARG ${R_NS}/platform)
set(PLATFORM_CDF PLATFORM)
set(PLATFORM_VAL "\"0\"")

#Parameters for defining tentacle selection huristics
set(GOAL_WEIGHT_ARG ${R_NS}/goal_weight)
set(GOAL_WEIGHT_CDF GOAL_WEIGHT)
set(GOAL_WEIGHT_VAL "\"2.0\"")
set(TRAV_WEIGHT_ARG ${R_NS}/traversed_weight)
set(TRAV_WEIGHT_CDF TRAV_WEIGHT)
set(TRAV_WEIGHT_VAL "\"0.1\"")
set(DIFF_WEIGHT_ARG ${R_NS}/difficult_weight)
set(DIFF_WEIGHT_CDF DIFF_WEIGHT)
set(DIFF_WEIGHT_VAL "\"0.25\"")

#Parameters for speed_sets on param server
set(S_MIN_SPEED_ARG ${S_NS}/min_speed)
set(S_MIN_SPEED_CDF S_MIN_SPEED)
set(S_MIN_SPEED_VAL "\"0.01\"")

set(S_MAX_SPEED_ARG ${S_NS}/max_speed)
set(S_MAX_SPEED_CDF S_MAX_SPEED)
set(S_MAX_SPEED_VAL "\"1.5\"")

set(S_NUMBER_ARG ${S_NS}/number)
set(S_NUMBER_CDF S_NUMBER)
set(S_NUMBER_VAL "\"15\"")

#Build the parameter header
configure_file (
  "AeroPathPlanningParameters.h.in"
  "${PROJECT_BINARY_DIR}/AeroPathPlanningParameters.h"
  )

#Add the launch file subdirs
add_subdirectory(aero_global_planner)
add_subdirectory(aero_local_planner)
add_subdirectory(aero_path_planning)
