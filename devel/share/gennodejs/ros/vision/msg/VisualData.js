// Auto-generated. Do not edit!

// (in-package vision.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class VisualData {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.obj_length = null;
      this.obj_width = null;
      this.lift_location = null;
      this.m1 = null;
      this.m2 = null;
      this.m3 = null;
      this.m4 = null;
      this.human_ar = null;
    }
    else {
      if (initObj.hasOwnProperty('obj_length')) {
        this.obj_length = initObj.obj_length
      }
      else {
        this.obj_length = 0.0;
      }
      if (initObj.hasOwnProperty('obj_width')) {
        this.obj_width = initObj.obj_width
      }
      else {
        this.obj_width = 0.0;
      }
      if (initObj.hasOwnProperty('lift_location')) {
        this.lift_location = initObj.lift_location
      }
      else {
        this.lift_location = new geometry_msgs.msg.PoseStamped();
      }
      if (initObj.hasOwnProperty('m1')) {
        this.m1 = initObj.m1
      }
      else {
        this.m1 = new geometry_msgs.msg.PoseStamped();
      }
      if (initObj.hasOwnProperty('m2')) {
        this.m2 = initObj.m2
      }
      else {
        this.m2 = new geometry_msgs.msg.PoseStamped();
      }
      if (initObj.hasOwnProperty('m3')) {
        this.m3 = initObj.m3
      }
      else {
        this.m3 = new geometry_msgs.msg.PoseStamped();
      }
      if (initObj.hasOwnProperty('m4')) {
        this.m4 = initObj.m4
      }
      else {
        this.m4 = new geometry_msgs.msg.PoseStamped();
      }
      if (initObj.hasOwnProperty('human_ar')) {
        this.human_ar = initObj.human_ar
      }
      else {
        this.human_ar = new geometry_msgs.msg.PoseStamped();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type VisualData
    // Serialize message field [obj_length]
    bufferOffset = _serializer.float64(obj.obj_length, buffer, bufferOffset);
    // Serialize message field [obj_width]
    bufferOffset = _serializer.float64(obj.obj_width, buffer, bufferOffset);
    // Serialize message field [lift_location]
    bufferOffset = geometry_msgs.msg.PoseStamped.serialize(obj.lift_location, buffer, bufferOffset);
    // Serialize message field [m1]
    bufferOffset = geometry_msgs.msg.PoseStamped.serialize(obj.m1, buffer, bufferOffset);
    // Serialize message field [m2]
    bufferOffset = geometry_msgs.msg.PoseStamped.serialize(obj.m2, buffer, bufferOffset);
    // Serialize message field [m3]
    bufferOffset = geometry_msgs.msg.PoseStamped.serialize(obj.m3, buffer, bufferOffset);
    // Serialize message field [m4]
    bufferOffset = geometry_msgs.msg.PoseStamped.serialize(obj.m4, buffer, bufferOffset);
    // Serialize message field [human_ar]
    bufferOffset = geometry_msgs.msg.PoseStamped.serialize(obj.human_ar, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type VisualData
    let len;
    let data = new VisualData(null);
    // Deserialize message field [obj_length]
    data.obj_length = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [obj_width]
    data.obj_width = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [lift_location]
    data.lift_location = geometry_msgs.msg.PoseStamped.deserialize(buffer, bufferOffset);
    // Deserialize message field [m1]
    data.m1 = geometry_msgs.msg.PoseStamped.deserialize(buffer, bufferOffset);
    // Deserialize message field [m2]
    data.m2 = geometry_msgs.msg.PoseStamped.deserialize(buffer, bufferOffset);
    // Deserialize message field [m3]
    data.m3 = geometry_msgs.msg.PoseStamped.deserialize(buffer, bufferOffset);
    // Deserialize message field [m4]
    data.m4 = geometry_msgs.msg.PoseStamped.deserialize(buffer, bufferOffset);
    // Deserialize message field [human_ar]
    data.human_ar = geometry_msgs.msg.PoseStamped.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += geometry_msgs.msg.PoseStamped.getMessageSize(object.lift_location);
    length += geometry_msgs.msg.PoseStamped.getMessageSize(object.m1);
    length += geometry_msgs.msg.PoseStamped.getMessageSize(object.m2);
    length += geometry_msgs.msg.PoseStamped.getMessageSize(object.m3);
    length += geometry_msgs.msg.PoseStamped.getMessageSize(object.m4);
    length += geometry_msgs.msg.PoseStamped.getMessageSize(object.human_ar);
    return length + 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'vision/VisualData';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'fc3c80b5cc5d31b4f28a30cd37bc7cc2';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 obj_length
    float64 obj_width
    geometry_msgs/PoseStamped lift_location
    geometry_msgs/PoseStamped m1
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new VisualData(null);
    if (msg.obj_length !== undefined) {
      resolved.obj_length = msg.obj_length;
    }
    else {
      resolved.obj_length = 0.0
    }

    if (msg.obj_width !== undefined) {
      resolved.obj_width = msg.obj_width;
    }
    else {
      resolved.obj_width = 0.0
    }

    if (msg.lift_location !== undefined) {
      resolved.lift_location = geometry_msgs.msg.PoseStamped.Resolve(msg.lift_location)
    }
    else {
      resolved.lift_location = new geometry_msgs.msg.PoseStamped()
    }

    if (msg.m1 !== undefined) {
      resolved.m1 = geometry_msgs.msg.PoseStamped.Resolve(msg.m1)
    }
    else {
      resolved.m1 = new geometry_msgs.msg.PoseStamped()
    }

    if (msg.m2 !== undefined) {
      resolved.m2 = geometry_msgs.msg.PoseStamped.Resolve(msg.m2)
    }
    else {
      resolved.m2 = new geometry_msgs.msg.PoseStamped()
    }

    if (msg.m3 !== undefined) {
      resolved.m3 = geometry_msgs.msg.PoseStamped.Resolve(msg.m3)
    }
    else {
      resolved.m3 = new geometry_msgs.msg.PoseStamped()
    }

    if (msg.m4 !== undefined) {
      resolved.m4 = geometry_msgs.msg.PoseStamped.Resolve(msg.m4)
    }
    else {
      resolved.m4 = new geometry_msgs.msg.PoseStamped()
    }

    if (msg.human_ar !== undefined) {
      resolved.human_ar = geometry_msgs.msg.PoseStamped.Resolve(msg.human_ar)
    }
    else {
      resolved.human_ar = new geometry_msgs.msg.PoseStamped()
    }

    return resolved;
    }
};

module.exports = VisualData;
