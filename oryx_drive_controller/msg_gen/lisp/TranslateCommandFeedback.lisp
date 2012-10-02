; Auto-generated. Do not edit!


(cl:in-package oryx_drive_controller-msg)


;//! \htmlinclude TranslateCommandFeedback.msg.html

(cl:defclass <TranslateCommandFeedback> (roslisp-msg-protocol:ros-message)
  ((x_velocity
    :reader x_velocity
    :initarg :x_velocity
    :type cl:float
    :initform 0.0)
   (y_velocity
    :reader y_velocity
    :initarg :y_velocity
    :type cl:float
    :initform 0.0))
)

(cl:defclass TranslateCommandFeedback (<TranslateCommandFeedback>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TranslateCommandFeedback>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TranslateCommandFeedback)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name oryx_drive_controller-msg:<TranslateCommandFeedback> is deprecated: use oryx_drive_controller-msg:TranslateCommandFeedback instead.")))

(cl:ensure-generic-function 'x_velocity-val :lambda-list '(m))
(cl:defmethod x_velocity-val ((m <TranslateCommandFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader oryx_drive_controller-msg:x_velocity-val is deprecated.  Use oryx_drive_controller-msg:x_velocity instead.")
  (x_velocity m))

(cl:ensure-generic-function 'y_velocity-val :lambda-list '(m))
(cl:defmethod y_velocity-val ((m <TranslateCommandFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader oryx_drive_controller-msg:y_velocity-val is deprecated.  Use oryx_drive_controller-msg:y_velocity instead.")
  (y_velocity m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TranslateCommandFeedback>) ostream)
  "Serializes a message object of type '<TranslateCommandFeedback>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'x_velocity))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'y_velocity))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TranslateCommandFeedback>) istream)
  "Deserializes a message object of type '<TranslateCommandFeedback>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x_velocity) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y_velocity) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TranslateCommandFeedback>)))
  "Returns string type for a message object of type '<TranslateCommandFeedback>"
  "oryx_drive_controller/TranslateCommandFeedback")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TranslateCommandFeedback)))
  "Returns string type for a message object of type 'TranslateCommandFeedback"
  "oryx_drive_controller/TranslateCommandFeedback")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TranslateCommandFeedback>)))
  "Returns md5sum for a message object of type '<TranslateCommandFeedback>"
  "7ee4a9a1d2944426c31584cd7f0b5821")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TranslateCommandFeedback)))
  "Returns md5sum for a message object of type 'TranslateCommandFeedback"
  "7ee4a9a1d2944426c31584cd7f0b5821")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TranslateCommandFeedback>)))
  "Returns full string definition for message of type '<TranslateCommandFeedback>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%#feedback~%float64 x_velocity~%float64 y_velocity~%#float64 omega~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TranslateCommandFeedback)))
  "Returns full string definition for message of type 'TranslateCommandFeedback"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%#feedback~%float64 x_velocity~%float64 y_velocity~%#float64 omega~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TranslateCommandFeedback>))
  (cl:+ 0
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TranslateCommandFeedback>))
  "Converts a ROS message object to a list"
  (cl:list 'TranslateCommandFeedback
    (cl:cons ':x_velocity (x_velocity msg))
    (cl:cons ':y_velocity (y_velocity msg))
))
