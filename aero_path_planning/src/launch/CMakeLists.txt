#Set all parameters here
#Parameters should have 3 set()'s: set(X_ARG arg_literal), set(X_CDEF DEF), set(X_VAL, value)

#Namespace params
set(R_NS aero)
set(O_NS occupancy)
set(L_NS local)
set(G_NS global)
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

set(UNKN_WEIGHT_ARG ${R_NS}/unknown_weight)
set(UNKN_WEIGHT_CDF UNKN_WEIGHT)
set(UNKN_WEIGHT_VAL "\"0.1\"")

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

#Parameters for local occupancy grid information
set(L_OCC_UPDTRT_ARG "${R_NS}/${O_NS}/${L_NS}/update_rate")
set(L_OCC_UPDTRT_CDF L_OCC_UPDTRT)
set(L_OCC_UPDTRT_VAL "\"0.2\"")

set(L_OCC_XDIM_ARG "${R_NS}/${O_NS}/${L_NS}/x_dim")
set(L_OCC_XDIM_CDF L_OCC_XDIM)
set(L_OCC_XDIM_VAL "\"200\"")

set(L_OCC_YDIM_ARG "${R_NS}/${O_NS}/${L_NS}/y_dim")
set(L_OCC_YDIM_CDF L_OCC_YDIM)
set(L_OCC_YDIM_VAL "\"200\"")

set(L_OCC_ZDIM_ARG "${R_NS}/${O_NS}/${L_NS}/z_dim")
set(L_OCC_ZDIM_CDF L_OCC_ZDIM)
set(L_OCC_ZDIM_VAL "\"0\"")

set(L_OCC_XORG_ARG "${R_NS}/${O_NS}/${L_NS}/x_org")
set(L_OCC_XORG_CDF L_OCC_XORG)
set(L_OCC_XORG_VAL "\"0\"")

set(L_OCC_YORG_ARG "${R_NS}/${O_NS}/${L_NS}/y_org")
set(L_OCC_YORG_CDF L_OCC_YORG)
set(L_OCC_YORG_VAL "\"100\"")

set(L_OCC_ZORG_ARG "${R_NS}/${O_NS}/${L_NS}/z_org")
set(L_OCC_ZORG_CDF L_OCC_ZORG)
set(L_OCC_ZORG_VAL "\"0\"")

set(L_OCC_RES_ARG "${R_NS}/${O_NS}/${L_NS}/res")
set(L_OCC_RES_CDF L_OCC_RES)
set(L_OCC_RES_VAL "\"0.1\"")

#Parameters for tentacle generation
set(T_EXPFACT_ARG "${R_NS}/${T_NS}/exp_fact")
set(T_EXPFACT_CDF T_EXPFACT)
set(T_EXPFACT_VAL "\"1.15\"")

set(T_NUMBER_ARG "${R_NS}/${T_NS}/number")
set(T_NUMBER_CDF T_NUMBER)
set(T_NUMBER_VAL "\"81\"")


#Build the parameter header
configure_file (
  "AeroPathPlanningParameters.h.in"
  "${PROJECT_BINARY_DIR}/AeroPathPlanningParameters.h"
)

#Add the launch file subdirs
add_subdirectory(aero_global_planner)
add_subdirectory(aero_local_planner)
add_subdirectory(aero_path_planning)
