FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/action_test/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/action_test/msg/__init__.py"
  "../src/action_test/msg/_TestAction.py"
  "../src/action_test/msg/_TestGoal.py"
  "../src/action_test/msg/_TestActionGoal.py"
  "../src/action_test/msg/_TestResult.py"
  "../src/action_test/msg/_TestActionResult.py"
  "../src/action_test/msg/_TestFeedback.py"
  "../src/action_test/msg/_TestActionFeedback.py"
  "../src/action_test/msg/_FibonacciResult.py"
  "../src/action_test/msg/_FibonacciActionGoal.py"
  "../src/action_test/msg/_FibonacciGoal.py"
  "../src/action_test/msg/_FibonacciActionResult.py"
  "../src/action_test/msg/_FibonacciActionFeedback.py"
  "../src/action_test/msg/_FibonacciFeedback.py"
  "../src/action_test/msg/_FibonacciAction.py"
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
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
