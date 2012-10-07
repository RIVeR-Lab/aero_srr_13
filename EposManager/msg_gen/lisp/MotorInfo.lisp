; Auto-generated. Do not edit!


(cl:in-package EposManager-msg)


;//! \htmlinclude MotorInfo.msg.html

(cl:defclass <MotorInfo> (roslisp-msg-protocol:ros-message)
  ((node_id
    :reader node_id
    :initarg :node_id
    :type cl:fixnum
    :initform 0)
   (motor_name
    :reader motor_name
    :initarg :motor_name
    :type cl:string
    :initform "")
   (state
    :reader state
    :initarg :state
    :type cl:fixnum
    :initform 0)
   (faults
    :reader faults
    :initarg :faults
    :type cl:string
    :initform "")
   (motor_velocity
    :reader motor_velocity
    :initarg :motor_velocity
    :type cl:integer
    :initform 0)
   (motor_position
    :reader motor_position
    :initarg :motor_position
    :type cl:integer
    :initform 0)
   (motor_current
    :reader motor_current
    :initarg :motor_current
    :type cl:integer
    :initform 0)
   (stamp
    :reader stamp
    :initarg :stamp
    :type cl:real
    :initform 0))
)

(cl:defclass MotorInfo (<MotorInfo>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MotorInfo>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MotorInfo)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name EposManager-msg:<MotorInfo> is deprecated: use EposManager-msg:MotorInfo instead.")))

(cl:ensure-generic-function 'node_id-val :lambda-list '(m))
(cl:defmethod node_id-val ((m <MotorInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader EposManager-msg:node_id-val is deprecated.  Use EposManager-msg:node_id instead.")
  (node_id m))

(cl:ensure-generic-function 'motor_name-val :lambda-list '(m))
(cl:defmethod motor_name-val ((m <MotorInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader EposManager-msg:motor_name-val is deprecated.  Use EposManager-msg:motor_name instead.")
  (motor_name m))

(cl:ensure-generic-function 'state-val :lambda-list '(m))
(cl:defmethod state-val ((m <MotorInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader EposManager-msg:state-val is deprecated.  Use EposManager-msg:state instead.")
  (state m))

(cl:ensure-generic-function 'faults-val :lambda-list '(m))
(cl:defmethod faults-val ((m <MotorInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader EposManager-msg:faults-val is deprecated.  Use EposManager-msg:faults instead.")
  (faults m))

(cl:ensure-generic-function 'motor_velocity-val :lambda-list '(m))
(cl:defmethod motor_velocity-val ((m <MotorInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader EposManager-msg:motor_velocity-val is deprecated.  Use EposManager-msg:motor_velocity instead.")
  (motor_velocity m))

(cl:ensure-generic-function 'motor_position-val :lambda-list '(m))
(cl:defmethod motor_position-val ((m <MotorInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader EposManager-msg:motor_position-val is deprecated.  Use EposManager-msg:motor_position instead.")
  (motor_position m))

(cl:ensure-generic-function 'motor_current-val :lambda-list '(m))
(cl:defmethod motor_current-val ((m <MotorInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader EposManager-msg:motor_current-val is deprecated.  Use EposManager-msg:motor_current instead.")
  (motor_current m))

(cl:ensure-generic-function 'stamp-val :lambda-list '(m))
(cl:defmethod stamp-val ((m <MotorInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader EposManager-msg:stamp-val is deprecated.  Use EposManager-msg:stamp instead.")
  (stamp m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MotorInfo>) ostream)
  "Serializes a message object of type '<MotorInfo>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'node_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'node_id)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'motor_name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'motor_name))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'state)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'state)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'faults))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'faults))
  (cl:let* ((signed (cl:slot-value msg 'motor_velocity)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'motor_position)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'motor_current)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((__sec (cl:floor (cl:slot-value msg 'stamp)))
        (__nsec (cl:round (cl:* 1e9 (cl:- (cl:slot-value msg 'stamp) (cl:floor (cl:slot-value msg 'stamp)))))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 0) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __nsec) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MotorInfo>) istream)
  "Deserializes a message object of type '<MotorInfo>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'node_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'node_id)) (cl:read-byte istream))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'motor_name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'motor_name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'state)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'state)) (cl:read-byte istream))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'faults) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'faults) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'motor_velocity) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'motor_position) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'motor_current) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((__sec 0) (__nsec 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 0) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __nsec) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'stamp) (cl:+ (cl:coerce __sec 'cl:double-float) (cl:/ __nsec 1e9))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MotorInfo>)))
  "Returns string type for a message object of type '<MotorInfo>"
  "EposManager/MotorInfo")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MotorInfo)))
  "Returns string type for a message object of type 'MotorInfo"
  "EposManager/MotorInfo")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MotorInfo>)))
  "Returns md5sum for a message object of type '<MotorInfo>"
  "f0af010a942d4953f1738bc4cb3bf83c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MotorInfo)))
  "Returns md5sum for a message object of type 'MotorInfo"
  "f0af010a942d4953f1738bc4cb3bf83c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MotorInfo>)))
  "Returns full string definition for message of type '<MotorInfo>"
  (cl:format cl:nil "# This represents the motor information of a specific motor~%# node_id is the node_id set on the EPOS Controller itself~%# motor_name is the name of the motor, as assigned by the user~%# enabled indicated whether or not the motor is enabled or disabled~%# motor_mode is the mode the motor is in (velocity, position, current)~%# motor_velocity is the velocity of the motor in rpm~%# motor_position is the position of the motor in encoder counts~%# motor_current is the current on the motor in milliamps~%~%uint16 node_id~%string motor_name~%uint16 state~%string faults~%int32 motor_velocity~%int32 motor_position~%int32 motor_current~%time stamp~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MotorInfo)))
  "Returns full string definition for message of type 'MotorInfo"
  (cl:format cl:nil "# This represents the motor information of a specific motor~%# node_id is the node_id set on the EPOS Controller itself~%# motor_name is the name of the motor, as assigned by the user~%# enabled indicated whether or not the motor is enabled or disabled~%# motor_mode is the mode the motor is in (velocity, position, current)~%# motor_velocity is the velocity of the motor in rpm~%# motor_position is the position of the motor in encoder counts~%# motor_current is the current on the motor in milliamps~%~%uint16 node_id~%string motor_name~%uint16 state~%string faults~%int32 motor_velocity~%int32 motor_position~%int32 motor_current~%time stamp~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MotorInfo>))
  (cl:+ 0
     2
     4 (cl:length (cl:slot-value msg 'motor_name))
     2
     4 (cl:length (cl:slot-value msg 'faults))
     4
     4
     4
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MotorInfo>))
  "Converts a ROS message object to a list"
  (cl:list 'MotorInfo
    (cl:cons ':node_id (node_id msg))
    (cl:cons ':motor_name (motor_name msg))
    (cl:cons ':state (state msg))
    (cl:cons ':faults (faults msg))
    (cl:cons ':motor_velocity (motor_velocity msg))
    (cl:cons ':motor_position (motor_position msg))
    (cl:cons ':motor_current (motor_current msg))
    (cl:cons ':stamp (stamp msg))
))
