; Auto-generated. Do not edit!


(cl:in-package EposManager-msg)


;//! \htmlinclude GroupMotorInfo.msg.html

(cl:defclass <GroupMotorInfo> (roslisp-msg-protocol:ros-message)
  ((motor_group
    :reader motor_group
    :initarg :motor_group
    :type (cl:vector EposManager-msg:MotorInfo)
   :initform (cl:make-array 0 :element-type 'EposManager-msg:MotorInfo :initial-element (cl:make-instance 'EposManager-msg:MotorInfo))))
)

(cl:defclass GroupMotorInfo (<GroupMotorInfo>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GroupMotorInfo>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GroupMotorInfo)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name EposManager-msg:<GroupMotorInfo> is deprecated: use EposManager-msg:GroupMotorInfo instead.")))

(cl:ensure-generic-function 'motor_group-val :lambda-list '(m))
(cl:defmethod motor_group-val ((m <GroupMotorInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader EposManager-msg:motor_group-val is deprecated.  Use EposManager-msg:motor_group instead.")
  (motor_group m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GroupMotorInfo>) ostream)
  "Serializes a message object of type '<GroupMotorInfo>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'motor_group))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'motor_group))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GroupMotorInfo>) istream)
  "Deserializes a message object of type '<GroupMotorInfo>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'motor_group) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'motor_group)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'EposManager-msg:MotorInfo))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GroupMotorInfo>)))
  "Returns string type for a message object of type '<GroupMotorInfo>"
  "EposManager/GroupMotorInfo")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GroupMotorInfo)))
  "Returns string type for a message object of type 'GroupMotorInfo"
  "EposManager/GroupMotorInfo")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GroupMotorInfo>)))
  "Returns md5sum for a message object of type '<GroupMotorInfo>"
  "df4d06ffe0935e1bb3f70b2d4bb7a274")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GroupMotorInfo)))
  "Returns md5sum for a message object of type 'GroupMotorInfo"
  "df4d06ffe0935e1bb3f70b2d4bb7a274")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GroupMotorInfo>)))
  "Returns full string definition for message of type '<GroupMotorInfo>"
  (cl:format cl:nil "# This represents the motor information of a group of motors~%~%MotorInfo[] motor_group~%~%================================================================================~%MSG: EposManager/MotorInfo~%# This represents the motor information of a specific motor~%# node_id is the node_id set on the EPOS Controller itself~%# motor_name is the name of the motor, as assigned by the user~%# enabled indicated whether or not the motor is enabled or disabled~%# motor_mode is the mode the motor is in (velocity, position, current)~%# motor_velocity is the velocity of the motor in rpm~%# motor_position is the position of the motor in encoder counts~%# motor_current is the current on the motor in milliamps~%~%uint16 node_id~%string motor_name~%uint16 state~%string faults~%int32 motor_velocity~%int32 motor_position~%int32 motor_current~%time stamp~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GroupMotorInfo)))
  "Returns full string definition for message of type 'GroupMotorInfo"
  (cl:format cl:nil "# This represents the motor information of a group of motors~%~%MotorInfo[] motor_group~%~%================================================================================~%MSG: EposManager/MotorInfo~%# This represents the motor information of a specific motor~%# node_id is the node_id set on the EPOS Controller itself~%# motor_name is the name of the motor, as assigned by the user~%# enabled indicated whether or not the motor is enabled or disabled~%# motor_mode is the mode the motor is in (velocity, position, current)~%# motor_velocity is the velocity of the motor in rpm~%# motor_position is the position of the motor in encoder counts~%# motor_current is the current on the motor in milliamps~%~%uint16 node_id~%string motor_name~%uint16 state~%string faults~%int32 motor_velocity~%int32 motor_position~%int32 motor_current~%time stamp~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GroupMotorInfo>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'motor_group) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GroupMotorInfo>))
  "Converts a ROS message object to a list"
  (cl:list 'GroupMotorInfo
    (cl:cons ':motor_group (motor_group msg))
))
