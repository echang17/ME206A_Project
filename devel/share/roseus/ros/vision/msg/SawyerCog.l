;; Auto-generated. Do not edit!


(when (boundp 'vision::SawyerCog)
  (if (not (find-package "VISION"))
    (make-package "VISION"))
  (shadow 'SawyerCog (find-package "VISION")))
(unless (find-package "VISION::SAWYERCOG")
  (make-package "VISION::SAWYERCOG"))

(in-package "ROS")
;;//! \htmlinclude SawyerCog.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass vision::SawyerCog
  :super ros::object
  :slots (_sawyer_end ))

(defmethod vision::SawyerCog
  (:init
   (&key
    ((:sawyer_end __sawyer_end) (instance geometry_msgs::Pose :init))
    )
   (send-super :init)
   (setq _sawyer_end __sawyer_end)
   self)
  (:sawyer_end
   (&rest __sawyer_end)
   (if (keywordp (car __sawyer_end))
       (send* _sawyer_end __sawyer_end)
     (progn
       (if __sawyer_end (setq _sawyer_end (car __sawyer_end)))
       _sawyer_end)))
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Pose _sawyer_end
    (send _sawyer_end :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Pose _sawyer_end
       (send _sawyer_end :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Pose _sawyer_end
     (send _sawyer_end :deserialize buf ptr-) (incf ptr- (send _sawyer_end :serialization-length))
   ;;
   self)
  )

(setf (get vision::SawyerCog :md5sum-) "c11a2e283bb2bfb477384cd2ce2a8c12")
(setf (get vision::SawyerCog :datatype-) "vision/SawyerCog")
(setf (get vision::SawyerCog :definition-)
      "geometry_msgs/Pose sawyer_end
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



(provide :vision/SawyerCog "c11a2e283bb2bfb477384cd2ce2a8c12")


