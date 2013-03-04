#Set all parameters here

set(OCCUPANCY_TOPIC_DEF OCCUPANCY_TOPIC)
set(OCCUPANCY_TOPIC_LIT "\"aero/occupancy_topic\"")

#Build the parameter header
configure_file (
  "AeroPathPlanningParameters.h.in"
  "${PROJECT_BINARY_DIR}/AeroPathPlanningParameters.h"
  )

add_subdirectory(aero_global_planner)
add_subdirectory(aero_local_planner)
add_subdirectory(aero_path_planning)
