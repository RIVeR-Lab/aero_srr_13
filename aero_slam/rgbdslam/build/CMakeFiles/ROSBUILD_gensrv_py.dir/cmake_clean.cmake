FILE(REMOVE_RECURSE
  "../srv_gen"
  "../src/rgbdslam/srv"
  "CMakeFiles/ROSBUILD_gensrv_py"
  "../src/rgbdslam/srv/__init__.py"
  "../src/rgbdslam/srv/_rgbdslam_ros_ui_f.py"
  "../src/rgbdslam/srv/_rgbdslam_ros_ui_s.py"
  "../src/rgbdslam/srv/_rgbdslam_ros_ui.py"
  "../src/rgbdslam/srv/_rgbdslam_ros_ui_b.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
