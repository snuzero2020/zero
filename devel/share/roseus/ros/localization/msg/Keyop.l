;; Auto-generated. Do not edit!


(when (boundp 'localization::Keyop)
  (if (not (find-package "LOCALIZATION"))
    (make-package "LOCALIZATION"))
  (shadow 'Keyop (find-package "LOCALIZATION")))
(unless (find-package "LOCALIZATION::KEYOP")
  (make-package "LOCALIZATION::KEYOP"))

(in-package "ROS")
;;//! \htmlinclude Keyop.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass localization::Keyop
  :super ros::object
  :slots (_header _key ))

(defmethod localization::Keyop
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:key __key) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _key (round __key))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:key
   (&optional __key)
   (if __key (setq _key __key)) _key)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; int32 _key
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; int32 _key
       (write-long _key s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; int32 _key
     (setq _key (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get localization::Keyop :md5sum-) "b541b7ffb470da46407f12a133fc3e51")
(setf (get localization::Keyop :datatype-) "localization/Keyop")
(setf (get localization::Keyop :definition-)
      "Header header
int32 key
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

")



(provide :localization/Keyop "b541b7ffb470da46407f12a133fc3e51")


