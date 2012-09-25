FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/action_test/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genaction_msgs"
  "../msg/TestAction.msg"
  "../msg/TestGoal.msg"
  "../msg/TestActionGoal.msg"
  "../msg/TestResult.msg"
  "../msg/TestActionResult.msg"
  "../msg/TestFeedback.msg"
  "../msg/TestActionFeedback.msg"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genaction_msgs.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
