FILE(REMOVE_RECURSE
  "../srv_gen"
  "../src/rgbdslam/srv"
  "CMakeFiles/ROSBUILD_gensrv_cpp"
  "../srv_gen/cpp/include/rgbdslam/rgbdslam_ros_ui_b.h"
  "../srv_gen/cpp/include/rgbdslam/rgbdslam_ros_ui_f.h"
  "../srv_gen/cpp/include/rgbdslam/rgbdslam_ros_ui.h"
  "../srv_gen/cpp/include/rgbdslam/rgbdslam_ros_ui_s.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
