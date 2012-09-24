FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/action_test/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "../msg_gen/cpp/include/action_test/TestAction.h"
  "../msg_gen/cpp/include/action_test/TestGoal.h"
  "../msg_gen/cpp/include/action_test/TestActionGoal.h"
  "../msg_gen/cpp/include/action_test/TestResult.h"
  "../msg_gen/cpp/include/action_test/TestActionResult.h"
  "../msg_gen/cpp/include/action_test/TestFeedback.h"
  "../msg_gen/cpp/include/action_test/TestActionFeedback.h"
  "../msg_gen/cpp/include/action_test/FibonacciResult.h"
  "../msg_gen/cpp/include/action_test/FibonacciActionGoal.h"
  "../msg_gen/cpp/include/action_test/FibonacciGoal.h"
  "../msg_gen/cpp/include/action_test/FibonacciActionResult.h"
  "../msg_gen/cpp/include/action_test/FibonacciActionFeedback.h"
  "../msg_gen/cpp/include/action_test/FibonacciFeedback.h"
  "../msg_gen/cpp/include/action_test/FibonacciAction.h"
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
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
