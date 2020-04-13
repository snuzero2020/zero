; Auto-generated. Do not edit!


(cl:in-package localization-msg)


;//! \htmlinclude Keyop.msg.html

(cl:defclass <Keyop> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (key
    :reader key
    :initarg :key
    :type cl:integer
    :initform 0))
)

(cl:defclass Keyop (<Keyop>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Keyop>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Keyop)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name localization-msg:<Keyop> is deprecated: use localization-msg:Keyop instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Keyop>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader localization-msg:header-val is deprecated.  Use localization-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'key-val :lambda-list '(m))
(cl:defmethod key-val ((m <Keyop>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader localization-msg:key-val is deprecated.  Use localization-msg:key instead.")
  (key m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Keyop>) ostream)
  "Serializes a message object of type '<Keyop>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let* ((signed (cl:slot-value msg 'key)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Keyop>) istream)
  "Deserializes a message object of type '<Keyop>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'key) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Keyop>)))
  "Returns string type for a message object of type '<Keyop>"
  "localization/Keyop")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Keyop)))
  "Returns string type for a message object of type 'Keyop"
  "localization/Keyop")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Keyop>)))
  "Returns md5sum for a message object of type '<Keyop>"
  "b541b7ffb470da46407f12a133fc3e51")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Keyop)))
  "Returns md5sum for a message object of type 'Keyop"
  "b541b7ffb470da46407f12a133fc3e51")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Keyop>)))
  "Returns full string definition for message of type '<Keyop>"
  (cl:format cl:nil "Header header~%int32 key~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Keyop)))
  "Returns full string definition for message of type 'Keyop"
  (cl:format cl:nil "Header header~%int32 key~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Keyop>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Keyop>))
  "Converts a ROS message object to a list"
  (cl:list 'Keyop
    (cl:cons ':header (header msg))
    (cl:cons ':key (key msg))
))
