#Build the parameter header
configure_file (
  "AeroPathPlanningParameters.h.in"
  "${PROJECT_BINARY_DIR}/AeroPathPlanningParameters.h"
  )

add_subdirectory(aero_global_planner)
add_subdirectory(aero_local_planner)
add_subdirectory(aero_path_planning)
