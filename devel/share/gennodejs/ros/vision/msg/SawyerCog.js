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
      this.sawyer_cog = null;
      this.sawyer_init = null;
    }
    else {
      if (initObj.hasOwnProperty('sawyer_cog')) {
        this.sawyer_cog = initObj.sawyer_cog
      }
      else {
        this.sawyer_cog = new geometry_msgs.msg.Pose();
      }
      if (initObj.hasOwnProperty('sawyer_init')) {
        this.sawyer_init = initObj.sawyer_init
      }
      else {
        this.sawyer_init = new geometry_msgs.msg.Pose();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SawyerCog
    // Serialize message field [sawyer_cog]
    bufferOffset = geometry_msgs.msg.Pose.serialize(obj.sawyer_cog, buffer, bufferOffset);
    // Serialize message field [sawyer_init]
    bufferOffset = geometry_msgs.msg.Pose.serialize(obj.sawyer_init, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SawyerCog
    let len;
    let data = new SawyerCog(null);
    // Deserialize message field [sawyer_cog]
    data.sawyer_cog = geometry_msgs.msg.Pose.deserialize(buffer, bufferOffset);
    // Deserialize message field [sawyer_init]
    data.sawyer_init = geometry_msgs.msg.Pose.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 112;
  }

  static datatype() {
    // Returns string type for a message object
    return 'vision/SawyerCog';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a24c9cfeb22ccc885dd31286abb4d19d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    geometry_msgs/Pose sawyer_cog
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SawyerCog(null);
    if (msg.sawyer_cog !== undefined) {
      resolved.sawyer_cog = geometry_msgs.msg.Pose.Resolve(msg.sawyer_cog)
    }
    else {
      resolved.sawyer_cog = new geometry_msgs.msg.Pose()
    }

    if (msg.sawyer_init !== undefined) {
      resolved.sawyer_init = geometry_msgs.msg.Pose.Resolve(msg.sawyer_init)
    }
    else {
      resolved.sawyer_init = new geometry_msgs.msg.Pose()
    }

    return resolved;
    }
};

module.exports = SawyerCog;
