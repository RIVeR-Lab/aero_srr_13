
#Set all parameters here
#Parameters should have 3 set()'s: set(X_ARG arg_literal), set(X_CDEF DEF), set(X_VAL, value)

#Namespace params
set(ROBOT_NS aero)
set(SB_NS stereo_bottom)
set(ST_NS stereo_top)
set(L_NS left)
set(R_NS right)


#Parameter for the topic for subscribing to the left camera
set(BOTTOM_LEFT_IMAGE_TOPIC_ARG ${ROBOT_NS}/${SB_NS}/${L_NS}/image_raw)
set(BOTTOM_LEFT_IMAGE_TOPIC_CDF BOTTOM_LEFT_IMAGE_TOPIC)
set(BOTTOM_LEFT_IMAGE_TOPIC_VAL "\"${ROBOT_NS}/${SB_NS}/${L_NS}/image_raw\"")

set(BOTTOM_LEFT_INFO_TOPIC_ARG ${ROBOT_NS}/${SB_NS}/${L_NS}/camera_info)
set(BOTTOM_LEFT_INFO_TOPIC_CDF BOTTOM_LEFT_INFO_TOPIC)
set(BOTTOM_LEFT_INFO_TOPIC_VAL "\"${ROBOT_NS}/${SB_NS}/${L_NS}/camera_info\"")

#Parameter for the topic for subscribing to the right camera
set(BOTTOM_RIGHT_IMAGE_TOPIC_ARG ${ROBOT_NS}/${SB_NS}/${R_NS}/image_raw)
set(BOTTOM_RIGHT_IMAGE_TOPIC_CDF BOTTOM_RIGHT_IMAGE_TOPIC)
set(BOTTOM_RIGHT_IMAGE_TOPIC_VAL "\"${ROBOT_NS}/${SB_NS}/${R_NS}/image_raw\"")

set(BOTTOM_RIGHT_INFO_TOPIC_ARG ${ROBOT_NS}/${SB_NS}/${R_NS}/camera_info)
set(BOTTOM_RIGHT_INFO_TOPIC_CDF BOTTOM_RIGHT_INFO_TOPIC)
set(BOTTOM_RIGHT_INFO_TOPIC_VAL "\"${ROBOT_NS}/${SB_NS}/${R_NS}/camera_info\"")

#Build the parameter header
configure_file (
  "ObjectLocatorParams.h.in"
  "${PROJECT_BINARY_DIR}/ObjectLocatorParams.h"
)


add_subdirectory(nodes)
add_subdirectory(helpers)
add_subdirectory(launch)