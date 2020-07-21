;; Auto-generated. Do not edit!


(when (boundp 'clustering::Points)
  (if (not (find-package "CLUSTERING"))
    (make-package "CLUSTERING"))
  (shadow 'Points (find-package "CLUSTERING")))
(unless (find-package "CLUSTERING::POINTS")
  (make-package "CLUSTERING::POINTS"))

(in-package "ROS")
;;//! \htmlinclude Points.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass clustering::Points
  :super ros::object
  :slots (_header _count _points _clusters _channels _is_3d ))

(defmethod clustering::Points
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:count __count) 0)
    ((:points __points) (let (r) (dotimes (i 0) (push (instance geometry_msgs::Point :init) r)) r))
    ((:clusters __clusters) (make-array 0 :initial-element 0 :element-type :integer))
    ((:channels __channels) (make-array 0 :initial-element 0 :element-type :integer))
    ((:is_3d __is_3d) nil)
    )
   (send-super :init)
   (setq _header __header)
   (setq _count (round __count))
   (setq _points __points)
   (setq _clusters __clusters)
   (setq _channels __channels)
   (setq _is_3d __is_3d)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:count
   (&optional __count)
   (if __count (setq _count __count)) _count)
  (:points
   (&rest __points)
   (if (keywordp (car __points))
       (send* _points __points)
     (progn
       (if __points (setq _points (car __points)))
       _points)))
  (:clusters
   (&optional __clusters)
   (if __clusters (setq _clusters __clusters)) _clusters)
  (:channels
   (&optional __channels)
   (if __channels (setq _channels __channels)) _channels)
  (:is_3d
   (&optional __is_3d)
   (if __is_3d (setq _is_3d __is_3d)) _is_3d)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint32 _count
    4
    ;; geometry_msgs/Point[] _points
    (apply #'+ (send-all _points :serialization-length)) 4
    ;; int32[] _clusters
    (* 4    (length _clusters)) 4
    ;; int32[] _channels
    (* 4    (length _channels)) 4
    ;; bool _is_3d
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint32 _count
       (write-long _count s)
     ;; geometry_msgs/Point[] _points
     (write-long (length _points) s)
     (dolist (elem _points)
       (send elem :serialize s)
       )
     ;; int32[] _clusters
     (write-long (length _clusters) s)
     (dotimes (i (length _clusters))
       (write-long (elt _clusters i) s)
       )
     ;; int32[] _channels
     (write-long (length _channels) s)
     (dotimes (i (length _channels))
       (write-long (elt _channels i) s)
       )
     ;; bool _is_3d
       (if _is_3d (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint32 _count
     (setq _count (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; geometry_msgs/Point[] _points
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _points (let (r) (dotimes (i n) (push (instance geometry_msgs::Point :init) r)) r))
     (dolist (elem- _points)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; int32[] _clusters
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _clusters (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _clusters i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; int32[] _channels
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _channels (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _channels i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; bool _is_3d
     (setq _is_3d (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get clustering::Points :md5sum-) "33601f15053b6bb6f55b6bf5417eecb7")
(setf (get clustering::Points :datatype-) "clustering/Points")
(setf (get clustering::Points :definition-)
      "Header header
uint32 count
geometry_msgs/Point[] points
int32[] clusters
int32[] channels
bool is_3d

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

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

")



(provide :clustering/Points "33601f15053b6bb6f55b6bf5417eecb7")


