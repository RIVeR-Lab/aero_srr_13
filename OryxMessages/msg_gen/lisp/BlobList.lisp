; Auto-generated. Do not edit!


(cl:in-package OryxMessages-msg)


;//! \htmlinclude BlobList.msg.html

(cl:defclass <BlobList> (roslisp-msg-protocol:ros-message)
  ((blobs
    :reader blobs
    :initarg :blobs
    :type (cl:vector OryxMessages-msg:Blob)
   :initform (cl:make-array 0 :element-type 'OryxMessages-msg:Blob :initial-element (cl:make-instance 'OryxMessages-msg:Blob)))
   (blobCount
    :reader blobCount
    :initarg :blobCount
    :type cl:integer
    :initform 0)
   (color
    :reader color
    :initarg :color
    :type cl:fixnum
    :initform 0))
)

(cl:defclass BlobList (<BlobList>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <BlobList>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'BlobList)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name OryxMessages-msg:<BlobList> is deprecated: use OryxMessages-msg:BlobList instead.")))

(cl:ensure-generic-function 'blobs-val :lambda-list '(m))
(cl:defmethod blobs-val ((m <BlobList>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader OryxMessages-msg:blobs-val is deprecated.  Use OryxMessages-msg:blobs instead.")
  (blobs m))

(cl:ensure-generic-function 'blobCount-val :lambda-list '(m))
(cl:defmethod blobCount-val ((m <BlobList>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader OryxMessages-msg:blobCount-val is deprecated.  Use OryxMessages-msg:blobCount instead.")
  (blobCount m))

(cl:ensure-generic-function 'color-val :lambda-list '(m))
(cl:defmethod color-val ((m <BlobList>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader OryxMessages-msg:color-val is deprecated.  Use OryxMessages-msg:color instead.")
  (color m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <BlobList>) ostream)
  "Serializes a message object of type '<BlobList>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'blobs))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'blobs))
  (cl:let* ((signed (cl:slot-value msg 'blobCount)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'color)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <BlobList>) istream)
  "Deserializes a message object of type '<BlobList>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'blobs) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'blobs)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'OryxMessages-msg:Blob))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'blobCount) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'color) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<BlobList>)))
  "Returns string type for a message object of type '<BlobList>"
  "OryxMessages/BlobList")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'BlobList)))
  "Returns string type for a message object of type 'BlobList"
  "OryxMessages/BlobList")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<BlobList>)))
  "Returns md5sum for a message object of type '<BlobList>"
  "76c461d0b54f222fd763ee65e508f7e8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'BlobList)))
  "Returns md5sum for a message object of type 'BlobList"
  "76c461d0b54f222fd763ee65e508f7e8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<BlobList>)))
  "Returns full string definition for message of type '<BlobList>"
  (cl:format cl:nil "Blob[] blobs~%int32 blobCount~%int8 color~%~%================================================================================~%MSG: OryxMessages/Blob~%int32 x~%int32 y~%int32 size~%int32 radius~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'BlobList)))
  "Returns full string definition for message of type 'BlobList"
  (cl:format cl:nil "Blob[] blobs~%int32 blobCount~%int8 color~%~%================================================================================~%MSG: OryxMessages/Blob~%int32 x~%int32 y~%int32 size~%int32 radius~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <BlobList>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'blobs) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <BlobList>))
  "Converts a ROS message object to a list"
  (cl:list 'BlobList
    (cl:cons ':blobs (blobs msg))
    (cl:cons ':blobCount (blobCount msg))
    (cl:cons ':color (color msg))
))
