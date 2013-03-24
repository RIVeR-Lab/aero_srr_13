#!/usr/bin/env sh
# generated from catkin/cmake/templates/env.sh.in

if [ $# -eq 0 ] ; then
  /bin/echo "Entering environment at '/home/bpwiselybabu/groovy_workspace/aero_srr/prosilica_stereo/build/devel', type 'exit' to leave"
  . "/home/bpwiselybabu/groovy_workspace/aero_srr/prosilica_stereo/build/devel/setup.sh"
  "$SHELL" -i
  /bin/echo "Exiting environment at '/home/bpwiselybabu/groovy_workspace/aero_srr/prosilica_stereo/build/devel'"
else
  . "/home/bpwiselybabu/groovy_workspace/aero_srr/prosilica_stereo/build/devel/setup.sh"
  exec "$@"
fi
