; Auto-generated. Do not edit!


(cl:in-package EposManager-msg)


;//! \htmlinclude GroupEPOSControl.msg.html

(cl:defclass <GroupEPOSControl> (roslisp-msg-protocol:ros-message)
  ((motor_group
    :reader motor_group
    :initarg :motor_group
    :type (cl:vector EposManager-msg:EPOSControl)
   :initform (cl:make-array 0 :element-type 'EposManager-msg:EPOSControl :initial-element (cl:make-instance 'EposManager-msg:EPOSControl))))
)

(cl:defclass GroupEPOSControl (<GroupEPOSControl>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GroupEPOSControl>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GroupEPOSControl)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name EposManager-msg:<GroupEPOSControl> is deprecated: use EposManager-msg:GroupEPOSControl instead.")))

(cl:ensure-generic-function 'motor_group-val :lambda-list '(m))
(cl:defmethod motor_group-val ((m <GroupEPOSControl>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader EposManager-msg:motor_group-val is deprecated.  Use EposManager-msg:motor_group instead.")
  (motor_group m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GroupEPOSControl>) ostream)
  "Serializes a message object of type '<GroupEPOSControl>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'motor_group))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'motor_group))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GroupEPOSControl>) istream)
  "Deserializes a message object of type '<GroupEPOSControl>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'motor_group) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'motor_group)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'EposManager-msg:EPOSControl))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GroupEPOSControl>)))
  "Returns string type for a message object of type '<GroupEPOSControl>"
  "EposManager/GroupEPOSControl")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GroupEPOSControl)))
  "Returns string type for a message object of type 'GroupEPOSControl"
  "EposManager/GroupEPOSControl")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GroupEPOSControl>)))
  "Returns md5sum for a message object of type '<GroupEPOSControl>"
  "58778ccf9bb7db6f7ccd5eaa3d10b6d8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GroupEPOSControl)))
  "Returns md5sum for a message object of type 'GroupEPOSControl"
  "58778ccf9bb7db6f7ccd5eaa3d10b6d8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GroupEPOSControl>)))
  "Returns full string definition for message of type '<GroupEPOSControl>"
  (cl:format cl:nil "# This represents a control structure for a Maxon motor attached to an EPOS 2 controller.  ~%# The node_id is the node_id set on the epos controller itself~%# The control_mode corresponds with the desired mode of control~%# The setpoint corresponds with the desired value for the selected mode~%~%EPOSControl[] motor_group~%~%================================================================================~%MSG: EposManager/EPOSControl~%# This represents a control structure for a Maxon motor attached to an EPOS 2 controller.  ~%# The node_id is the node_id set on the epos controller itself~%# The control_mode corresponds with the desired mode of control~%# The setpoint corresponds with the desired value for the selected mode~%~%# Control Mode Options~%uint8 VELOCITY =1~%uint8 ABSOLUTE_POSITION = 2~%uint8 ABSOLUTE_POSITION_IMMEDIATE = 3~%uint8 RELATIVE_POSITION = 4~%uint8 RELATIVE_POSITION_IMMEDIATE = 5~%~%~%uint16 node_id~%uint8 control_mode~%int32 setpoint~%~%~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GroupEPOSControl)))
  "Returns full string definition for message of type 'GroupEPOSControl"
  (cl:format cl:nil "# This represents a control structure for a Maxon motor attached to an EPOS 2 controller.  ~%# The node_id is the node_id set on the epos controller itself~%# The control_mode corresponds with the desired mode of control~%# The setpoint corresponds with the desired value for the selected mode~%~%EPOSControl[] motor_group~%~%================================================================================~%MSG: EposManager/EPOSControl~%# This represents a control structure for a Maxon motor attached to an EPOS 2 controller.  ~%# The node_id is the node_id set on the epos controller itself~%# The control_mode corresponds with the desired mode of control~%# The setpoint corresponds with the desired value for the selected mode~%~%# Control Mode Options~%uint8 VELOCITY =1~%uint8 ABSOLUTE_POSITION = 2~%uint8 ABSOLUTE_POSITION_IMMEDIATE = 3~%uint8 RELATIVE_POSITION = 4~%uint8 RELATIVE_POSITION_IMMEDIATE = 5~%~%~%uint16 node_id~%uint8 control_mode~%int32 setpoint~%~%~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GroupEPOSControl>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'motor_group) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GroupEPOSControl>))
  "Converts a ROS message object to a list"
  (cl:list 'GroupEPOSControl
    (cl:cons ':motor_group (motor_group msg))
))
