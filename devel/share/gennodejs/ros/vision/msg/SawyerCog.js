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

class SawyerCog {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.sawyer_end = null;
    }
    else {
      if (initObj.hasOwnProperty('sawyer_end')) {
        this.sawyer_end = initObj.sawyer_end
      }
      else {
        this.sawyer_end = new geometry_msgs.msg.Pose();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SawyerCog
    // Serialize message field [sawyer_end]
    bufferOffset = geometry_msgs.msg.Pose.serialize(obj.sawyer_end, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SawyerCog
    let len;
    let data = new SawyerCog(null);
    // Deserialize message field [sawyer_end]
    data.sawyer_end = geometry_msgs.msg.Pose.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 56;
  }

  static datatype() {
    // Returns string type for a message object
    return 'vision/SawyerCog';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c11a2e283bb2bfb477384cd2ce2a8c12';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    geometry_msgs/Pose sawyer_end
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
    const resolved = new SawyerCog(null);
    if (msg.sawyer_end !== undefined) {
      resolved.sawyer_end = geometry_msgs.msg.Pose.Resolve(msg.sawyer_end)
    }
    else {
      resolved.sawyer_end = new geometry_msgs.msg.Pose()
    }

    return resolved;
    }
};

module.exports = SawyerCog;
