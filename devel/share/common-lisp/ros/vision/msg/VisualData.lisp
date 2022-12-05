; Auto-generated. Do not edit!


(cl:in-package vision-msg)


;//! \htmlinclude VisualData.msg.html

(cl:defclass <VisualData> (roslisp-msg-protocol:ros-message)
  ((m1
    :reader m1
    :initarg :m1
    :type geometry_msgs-msg:PoseStamped
    :initform (cl:make-instance 'geometry_msgs-msg:PoseStamped))
   (m2
    :reader m2
    :initarg :m2
    :type geometry_msgs-msg:PoseStamped
    :initform (cl:make-instance 'geometry_msgs-msg:PoseStamped))
   (m3
    :reader m3
    :initarg :m3
    :type geometry_msgs-msg:PoseStamped
    :initform (cl:make-instance 'geometry_msgs-msg:PoseStamped))
   (m4
    :reader m4
    :initarg :m4
    :type geometry_msgs-msg:PoseStamped
    :initform (cl:make-instance 'geometry_msgs-msg:PoseStamped))
   (human_ar
    :reader human_ar
    :initarg :human_ar
    :type geometry_msgs-msg:PoseStamped
    :initform (cl:make-instance 'geometry_msgs-msg:PoseStamped)))
)

(cl:defclass VisualData (<VisualData>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <VisualData>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'VisualData)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name vision-msg:<VisualData> is deprecated: use vision-msg:VisualData instead.")))

(cl:ensure-generic-function 'm1-val :lambda-list '(m))
(cl:defmethod m1-val ((m <VisualData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vision-msg:m1-val is deprecated.  Use vision-msg:m1 instead.")
  (m1 m))

(cl:ensure-generic-function 'm2-val :lambda-list '(m))
(cl:defmethod m2-val ((m <VisualData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vision-msg:m2-val is deprecated.  Use vision-msg:m2 instead.")
  (m2 m))

(cl:ensure-generic-function 'm3-val :lambda-list '(m))
(cl:defmethod m3-val ((m <VisualData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vision-msg:m3-val is deprecated.  Use vision-msg:m3 instead.")
  (m3 m))

(cl:ensure-generic-function 'm4-val :lambda-list '(m))
(cl:defmethod m4-val ((m <VisualData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vision-msg:m4-val is deprecated.  Use vision-msg:m4 instead.")
  (m4 m))

(cl:ensure-generic-function 'human_ar-val :lambda-list '(m))
(cl:defmethod human_ar-val ((m <VisualData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vision-msg:human_ar-val is deprecated.  Use vision-msg:human_ar instead.")
  (human_ar m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <VisualData>) ostream)
  "Serializes a message object of type '<VisualData>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'm1) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'm2) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'm3) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'm4) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'human_ar) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <VisualData>) istream)
  "Deserializes a message object of type '<VisualData>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'm1) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'm2) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'm3) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'm4) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'human_ar) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<VisualData>)))
  "Returns string type for a message object of type '<VisualData>"
  "vision/VisualData")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'VisualData)))
  "Returns string type for a message object of type 'VisualData"
  "vision/VisualData")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<VisualData>)))
  "Returns md5sum for a message object of type '<VisualData>"
  "d66a0c6279d93fd1e1e5ac20418fdde7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'VisualData)))
  "Returns md5sum for a message object of type 'VisualData"
  "d66a0c6279d93fd1e1e5ac20418fdde7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<VisualData>)))
  "Returns full string definition for message of type '<VisualData>"
  (cl:format cl:nil "geometry_msgs/PoseStamped m1~%geometry_msgs/PoseStamped m2~%geometry_msgs/PoseStamped m3~%geometry_msgs/PoseStamped m4~%geometry_msgs/PoseStamped human_ar~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'VisualData)))
  "Returns full string definition for message of type 'VisualData"
  (cl:format cl:nil "geometry_msgs/PoseStamped m1~%geometry_msgs/PoseStamped m2~%geometry_msgs/PoseStamped m3~%geometry_msgs/PoseStamped m4~%geometry_msgs/PoseStamped human_ar~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <VisualData>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'm1))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'm2))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'm3))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'm4))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'human_ar))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <VisualData>))
  "Converts a ROS message object to a list"
  (cl:list 'VisualData
    (cl:cons ':m1 (m1 msg))
    (cl:cons ':m2 (m2 msg))
    (cl:cons ':m3 (m3 msg))
    (cl:cons ':m4 (m4 msg))
    (cl:cons ':human_ar (human_ar msg))
))
