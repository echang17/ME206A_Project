; Auto-generated. Do not edit!


(cl:in-package vision-msg)


;//! \htmlinclude SawyerCog.msg.html

(cl:defclass <SawyerCog> (roslisp-msg-protocol:ros-message)
  ((sawyer_cog
    :reader sawyer_cog
    :initarg :sawyer_cog
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose))
   (sawyer_init
    :reader sawyer_init
    :initarg :sawyer_init
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose)))
)

(cl:defclass SawyerCog (<SawyerCog>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SawyerCog>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SawyerCog)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name vision-msg:<SawyerCog> is deprecated: use vision-msg:SawyerCog instead.")))

(cl:ensure-generic-function 'sawyer_cog-val :lambda-list '(m))
(cl:defmethod sawyer_cog-val ((m <SawyerCog>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vision-msg:sawyer_cog-val is deprecated.  Use vision-msg:sawyer_cog instead.")
  (sawyer_cog m))

(cl:ensure-generic-function 'sawyer_init-val :lambda-list '(m))
(cl:defmethod sawyer_init-val ((m <SawyerCog>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vision-msg:sawyer_init-val is deprecated.  Use vision-msg:sawyer_init instead.")
  (sawyer_init m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SawyerCog>) ostream)
  "Serializes a message object of type '<SawyerCog>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'sawyer_cog) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'sawyer_init) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SawyerCog>) istream)
  "Deserializes a message object of type '<SawyerCog>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'sawyer_cog) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'sawyer_init) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SawyerCog>)))
  "Returns string type for a message object of type '<SawyerCog>"
  "vision/SawyerCog")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SawyerCog)))
  "Returns string type for a message object of type 'SawyerCog"
  "vision/SawyerCog")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SawyerCog>)))
  "Returns md5sum for a message object of type '<SawyerCog>"
  "a24c9cfeb22ccc885dd31286abb4d19d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SawyerCog)))
  "Returns md5sum for a message object of type 'SawyerCog"
  "a24c9cfeb22ccc885dd31286abb4d19d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SawyerCog>)))
  "Returns full string definition for message of type '<SawyerCog>"
  (cl:format cl:nil "geometry_msgs/Pose sawyer_cog~%geometry_msgs/Pose sawyer_init~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SawyerCog)))
  "Returns full string definition for message of type 'SawyerCog"
  (cl:format cl:nil "geometry_msgs/Pose sawyer_cog~%geometry_msgs/Pose sawyer_init~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SawyerCog>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'sawyer_cog))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'sawyer_init))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SawyerCog>))
  "Converts a ROS message object to a list"
  (cl:list 'SawyerCog
    (cl:cons ':sawyer_cog (sawyer_cog msg))
    (cl:cons ':sawyer_init (sawyer_init msg))
))
