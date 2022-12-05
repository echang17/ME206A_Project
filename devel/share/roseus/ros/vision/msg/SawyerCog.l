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
  :slots (_sawyer_cog _sawyer_init ))

(defmethod vision::SawyerCog
  (:init
   (&key
    ((:sawyer_cog __sawyer_cog) (instance geometry_msgs::Pose :init))
    ((:sawyer_init __sawyer_init) (instance geometry_msgs::Pose :init))
    )
   (send-super :init)
   (setq _sawyer_cog __sawyer_cog)
   (setq _sawyer_init __sawyer_init)
   self)
  (:sawyer_cog
   (&rest __sawyer_cog)
   (if (keywordp (car __sawyer_cog))
       (send* _sawyer_cog __sawyer_cog)
     (progn
       (if __sawyer_cog (setq _sawyer_cog (car __sawyer_cog)))
       _sawyer_cog)))
  (:sawyer_init
   (&rest __sawyer_init)
   (if (keywordp (car __sawyer_init))
       (send* _sawyer_init __sawyer_init)
     (progn
       (if __sawyer_init (setq _sawyer_init (car __sawyer_init)))
       _sawyer_init)))
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Pose _sawyer_cog
    (send _sawyer_cog :serialization-length)
    ;; geometry_msgs/Pose _sawyer_init
    (send _sawyer_init :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Pose _sawyer_cog
       (send _sawyer_cog :serialize s)
     ;; geometry_msgs/Pose _sawyer_init
       (send _sawyer_init :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Pose _sawyer_cog
     (send _sawyer_cog :deserialize buf ptr-) (incf ptr- (send _sawyer_cog :serialization-length))
   ;; geometry_msgs/Pose _sawyer_init
     (send _sawyer_init :deserialize buf ptr-) (incf ptr- (send _sawyer_init :serialization-length))
   ;;
   self)
  )

(setf (get vision::SawyerCog :md5sum-) "a24c9cfeb22ccc885dd31286abb4d19d")
(setf (get vision::SawyerCog :datatype-) "vision/SawyerCog")
(setf (get vision::SawyerCog :definition-)
      "geometry_msgs/Pose sawyer_cog
geometry_msgs/Pose sawyer_init
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



(provide :vision/SawyerCog "a24c9cfeb22ccc885dd31286abb4d19d")


