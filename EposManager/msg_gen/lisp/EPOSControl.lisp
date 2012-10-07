; Auto-generated. Do not edit!


(cl:in-package EposManager-msg)


;//! \htmlinclude EPOSControl.msg.html

(cl:defclass <EPOSControl> (roslisp-msg-protocol:ros-message)
  ((node_id
    :reader node_id
    :initarg :node_id
    :type cl:fixnum
    :initform 0)
   (control_mode
    :reader control_mode
    :initarg :control_mode
    :type cl:fixnum
    :initform 0)
   (setpoint
    :reader setpoint
    :initarg :setpoint
    :type cl:integer
    :initform 0))
)

(cl:defclass EPOSControl (<EPOSControl>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <EPOSControl>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'EPOSControl)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name EposManager-msg:<EPOSControl> is deprecated: use EposManager-msg:EPOSControl instead.")))

(cl:ensure-generic-function 'node_id-val :lambda-list '(m))
(cl:defmethod node_id-val ((m <EPOSControl>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader EposManager-msg:node_id-val is deprecated.  Use EposManager-msg:node_id instead.")
  (node_id m))

(cl:ensure-generic-function 'control_mode-val :lambda-list '(m))
(cl:defmethod control_mode-val ((m <EPOSControl>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader EposManager-msg:control_mode-val is deprecated.  Use EposManager-msg:control_mode instead.")
  (control_mode m))

(cl:ensure-generic-function 'setpoint-val :lambda-list '(m))
(cl:defmethod setpoint-val ((m <EPOSControl>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader EposManager-msg:setpoint-val is deprecated.  Use EposManager-msg:setpoint instead.")
  (setpoint m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<EPOSControl>)))
    "Constants for message type '<EPOSControl>"
  '((:VELOCITY . 1)
    (:ABSOLUTE_POSITION . 2)
    (:ABSOLUTE_POSITION_IMMEDIATE . 3)
    (:RELATIVE_POSITION . 4)
    (:RELATIVE_POSITION_IMMEDIATE . 5))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'EPOSControl)))
    "Constants for message type 'EPOSControl"
  '((:VELOCITY . 1)
    (:ABSOLUTE_POSITION . 2)
    (:ABSOLUTE_POSITION_IMMEDIATE . 3)
    (:RELATIVE_POSITION . 4)
    (:RELATIVE_POSITION_IMMEDIATE . 5))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <EPOSControl>) ostream)
  "Serializes a message object of type '<EPOSControl>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'node_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'node_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'control_mode)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'setpoint)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <EPOSControl>) istream)
  "Deserializes a message object of type '<EPOSControl>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'node_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'node_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'control_mode)) (cl:read-byte istream))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'setpoint) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<EPOSControl>)))
  "Returns string type for a message object of type '<EPOSControl>"
  "EposManager/EPOSControl")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'EPOSControl)))
  "Returns string type for a message object of type 'EPOSControl"
  "EposManager/EPOSControl")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<EPOSControl>)))
  "Returns md5sum for a message object of type '<EPOSControl>"
  "a188b630eb63b57363acb91954210f81")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'EPOSControl)))
  "Returns md5sum for a message object of type 'EPOSControl"
  "a188b630eb63b57363acb91954210f81")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<EPOSControl>)))
  "Returns full string definition for message of type '<EPOSControl>"
  (cl:format cl:nil "# This represents a control structure for a Maxon motor attached to an EPOS 2 controller.  ~%# The node_id is the node_id set on the epos controller itself~%# The control_mode corresponds with the desired mode of control~%# The setpoint corresponds with the desired value for the selected mode~%~%# Control Mode Options~%uint8 VELOCITY =1~%uint8 ABSOLUTE_POSITION = 2~%uint8 ABSOLUTE_POSITION_IMMEDIATE = 3~%uint8 RELATIVE_POSITION = 4~%uint8 RELATIVE_POSITION_IMMEDIATE = 5~%~%~%uint16 node_id~%uint8 control_mode~%int32 setpoint~%~%~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'EPOSControl)))
  "Returns full string definition for message of type 'EPOSControl"
  (cl:format cl:nil "# This represents a control structure for a Maxon motor attached to an EPOS 2 controller.  ~%# The node_id is the node_id set on the epos controller itself~%# The control_mode corresponds with the desired mode of control~%# The setpoint corresponds with the desired value for the selected mode~%~%# Control Mode Options~%uint8 VELOCITY =1~%uint8 ABSOLUTE_POSITION = 2~%uint8 ABSOLUTE_POSITION_IMMEDIATE = 3~%uint8 RELATIVE_POSITION = 4~%uint8 RELATIVE_POSITION_IMMEDIATE = 5~%~%~%uint16 node_id~%uint8 control_mode~%int32 setpoint~%~%~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <EPOSControl>))
  (cl:+ 0
     2
     1
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <EPOSControl>))
  "Converts a ROS message object to a list"
  (cl:list 'EPOSControl
    (cl:cons ':node_id (node_id msg))
    (cl:cons ':control_mode (control_mode msg))
    (cl:cons ':setpoint (setpoint msg))
))
