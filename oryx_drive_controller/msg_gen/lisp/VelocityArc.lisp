; Auto-generated. Do not edit!


(cl:in-package oryx_drive_controller-msg)


;//! \htmlinclude VelocityArc.msg.html

(cl:defclass <VelocityArc> (roslisp-msg-protocol:ros-message)
  ((velocity
    :reader velocity
    :initarg :velocity
    :type cl:float
    :initform 0.0)
   (radius
    :reader radius
    :initarg :radius
    :type cl:float
    :initform 0.0))
)

(cl:defclass VelocityArc (<VelocityArc>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <VelocityArc>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'VelocityArc)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name oryx_drive_controller-msg:<VelocityArc> is deprecated: use oryx_drive_controller-msg:VelocityArc instead.")))

(cl:ensure-generic-function 'velocity-val :lambda-list '(m))
(cl:defmethod velocity-val ((m <VelocityArc>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader oryx_drive_controller-msg:velocity-val is deprecated.  Use oryx_drive_controller-msg:velocity instead.")
  (velocity m))

(cl:ensure-generic-function 'radius-val :lambda-list '(m))
(cl:defmethod radius-val ((m <VelocityArc>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader oryx_drive_controller-msg:radius-val is deprecated.  Use oryx_drive_controller-msg:radius instead.")
  (radius m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <VelocityArc>) ostream)
  "Serializes a message object of type '<VelocityArc>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'velocity))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'radius))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <VelocityArc>) istream)
  "Deserializes a message object of type '<VelocityArc>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'velocity) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'radius) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<VelocityArc>)))
  "Returns string type for a message object of type '<VelocityArc>"
  "oryx_drive_controller/VelocityArc")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'VelocityArc)))
  "Returns string type for a message object of type 'VelocityArc"
  "oryx_drive_controller/VelocityArc")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<VelocityArc>)))
  "Returns md5sum for a message object of type '<VelocityArc>"
  "a4769d0b94d27fd4f9e32c14097aeebb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'VelocityArc)))
  "Returns md5sum for a message object of type 'VelocityArc"
  "a4769d0b94d27fd4f9e32c14097aeebb")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<VelocityArc>)))
  "Returns full string definition for message of type '<VelocityArc>"
  (cl:format cl:nil "#Service used to send velocity-radius information to the ArcDriveController~%~%#Tangential Velocity To Maintain~%float64 velocity~%~%#Radius of arc to create~%float64 radius~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'VelocityArc)))
  "Returns full string definition for message of type 'VelocityArc"
  (cl:format cl:nil "#Service used to send velocity-radius information to the ArcDriveController~%~%#Tangential Velocity To Maintain~%float64 velocity~%~%#Radius of arc to create~%float64 radius~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <VelocityArc>))
  (cl:+ 0
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <VelocityArc>))
  "Converts a ROS message object to a list"
  (cl:list 'VelocityArc
    (cl:cons ':velocity (velocity msg))
    (cl:cons ':radius (radius msg))
))
