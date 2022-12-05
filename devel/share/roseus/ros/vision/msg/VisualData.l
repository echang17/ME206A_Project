;; Auto-generated. Do not edit!


(when (boundp 'vision::VisualData)
  (if (not (find-package "VISION"))
    (make-package "VISION"))
  (shadow 'VisualData (find-package "VISION")))
(unless (find-package "VISION::VISUALDATA")
  (make-package "VISION::VISUALDATA"))

(in-package "ROS")
;;//! \htmlinclude VisualData.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass vision::VisualData
  :super ros::object
  :slots (_m1 _m2 _m3 _m4 _human_ar ))

(defmethod vision::VisualData
  (:init
   (&key
    ((:m1 __m1) (instance geometry_msgs::PoseStamped :init))
    ((:m2 __m2) (instance geometry_msgs::PoseStamped :init))
    ((:m3 __m3) (instance geometry_msgs::PoseStamped :init))
    ((:m4 __m4) (instance geometry_msgs::PoseStamped :init))
    ((:human_ar __human_ar) (instance geometry_msgs::PoseStamped :init))
    )
   (send-super :init)
   (setq _m1 __m1)
   (setq _m2 __m2)
   (setq _m3 __m3)
   (setq _m4 __m4)
   (setq _human_ar __human_ar)
   self)
  (:m1
   (&rest __m1)
   (if (keywordp (car __m1))
       (send* _m1 __m1)
     (progn
       (if __m1 (setq _m1 (car __m1)))
       _m1)))
  (:m2
   (&rest __m2)
   (if (keywordp (car __m2))
       (send* _m2 __m2)
     (progn
       (if __m2 (setq _m2 (car __m2)))
       _m2)))
  (:m3
   (&rest __m3)
   (if (keywordp (car __m3))
       (send* _m3 __m3)
     (progn
       (if __m3 (setq _m3 (car __m3)))
       _m3)))
  (:m4
   (&rest __m4)
   (if (keywordp (car __m4))
       (send* _m4 __m4)
     (progn
       (if __m4 (setq _m4 (car __m4)))
       _m4)))
  (:human_ar
   (&rest __human_ar)
   (if (keywordp (car __human_ar))
       (send* _human_ar __human_ar)
     (progn
       (if __human_ar (setq _human_ar (car __human_ar)))
       _human_ar)))
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/PoseStamped _m1
    (send _m1 :serialization-length)
    ;; geometry_msgs/PoseStamped _m2
    (send _m2 :serialization-length)
    ;; geometry_msgs/PoseStamped _m3
    (send _m3 :serialization-length)
    ;; geometry_msgs/PoseStamped _m4
    (send _m4 :serialization-length)
    ;; geometry_msgs/PoseStamped _human_ar
    (send _human_ar :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/PoseStamped _m1
       (send _m1 :serialize s)
     ;; geometry_msgs/PoseStamped _m2
       (send _m2 :serialize s)
     ;; geometry_msgs/PoseStamped _m3
       (send _m3 :serialize s)
     ;; geometry_msgs/PoseStamped _m4
       (send _m4 :serialize s)
     ;; geometry_msgs/PoseStamped _human_ar
       (send _human_ar :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/PoseStamped _m1
     (send _m1 :deserialize buf ptr-) (incf ptr- (send _m1 :serialization-length))
   ;; geometry_msgs/PoseStamped _m2
     (send _m2 :deserialize buf ptr-) (incf ptr- (send _m2 :serialization-length))
   ;; geometry_msgs/PoseStamped _m3
     (send _m3 :deserialize buf ptr-) (incf ptr- (send _m3 :serialization-length))
   ;; geometry_msgs/PoseStamped _m4
     (send _m4 :deserialize buf ptr-) (incf ptr- (send _m4 :serialization-length))
   ;; geometry_msgs/PoseStamped _human_ar
     (send _human_ar :deserialize buf ptr-) (incf ptr- (send _human_ar :serialization-length))
   ;;
   self)
  )

(setf (get vision::VisualData :md5sum-) "d66a0c6279d93fd1e1e5ac20418fdde7")
(setf (get vision::VisualData :datatype-) "vision/VisualData")
(setf (get vision::VisualData :definition-)
      "geometry_msgs/PoseStamped m1
geometry_msgs/PoseStamped m2
geometry_msgs/PoseStamped m3
geometry_msgs/PoseStamped m4
geometry_msgs/PoseStamped human_ar

================================================================================
MSG: geometry_msgs/PoseStamped
# A Pose with reference coordinate frame and timestamp
Header header
Pose pose

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
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

")



(provide :vision/VisualData "d66a0c6279d93fd1e1e5ac20418fdde7")


