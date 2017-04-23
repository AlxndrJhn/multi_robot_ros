// Auto-generated. Do not edit!

// (in-package marker_controller.msg)


"use strict";

let _serializer = require('../base_serialize.js');
let _deserializer = require('../base_deserialize.js');
let _finder = require('../find.js');
let TargetPose = require('./TargetPose.js');

//-----------------------------------------------------------

class TargetPoses {
  constructor() {
    this.targets = [];
  }

  static serialize(obj, bufferInfo) {
    // Serializes a message object of type TargetPoses
    // Serialize the length for message field [targets]
    bufferInfo = _serializer.uint32(obj.targets.length, bufferInfo);
    // Serialize message field [targets]
    obj.targets.forEach((val) => {
      bufferInfo = TargetPose.serialize(val, bufferInfo);
    });
    return bufferInfo;
  }

  static deserialize(buffer) {
    //deserializes a message object of type TargetPoses
    let tmp;
    let len;
    let data = new TargetPoses();
    // Deserialize array length for message field [targets]
    tmp = _deserializer.uint32(buffer);
    len = tmp.data;
    buffer = tmp.buffer;
    // Deserialize message field [targets]
    data.targets = new Array(len);
    for (let i = 0; i < len; ++i) {
      tmp = TargetPose.deserialize(buffer);
      data.targets[i] = tmp.data;
      buffer = tmp.buffer;
    }
    return {
      data: data,
      buffer: buffer
    }
  }

  static datatype() {
    // Returns string type for a message object
    return 'marker_controller/TargetPoses';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '3a8172c4ff6212bb43118476f3f97c7b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    TargetPose[] targets
    
    ================================================================================
    MSG: marker_controller/TargetPose
    uint32 id
    geometry_msgs/PoseStamped pose
    
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
    # 0: no frame
    # 1: global frame
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

};

module.exports = TargetPoses;
