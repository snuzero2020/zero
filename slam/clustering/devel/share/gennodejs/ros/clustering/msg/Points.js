// Auto-generated. Do not edit!

// (in-package clustering.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class Points {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.count = null;
      this.points = null;
      this.clusters = null;
      this.channels = null;
      this.is_3d = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('count')) {
        this.count = initObj.count
      }
      else {
        this.count = 0;
      }
      if (initObj.hasOwnProperty('points')) {
        this.points = initObj.points
      }
      else {
        this.points = [];
      }
      if (initObj.hasOwnProperty('clusters')) {
        this.clusters = initObj.clusters
      }
      else {
        this.clusters = [];
      }
      if (initObj.hasOwnProperty('channels')) {
        this.channels = initObj.channels
      }
      else {
        this.channels = [];
      }
      if (initObj.hasOwnProperty('is_3d')) {
        this.is_3d = initObj.is_3d
      }
      else {
        this.is_3d = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Points
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [count]
    bufferOffset = _serializer.uint32(obj.count, buffer, bufferOffset);
    // Serialize message field [points]
    // Serialize the length for message field [points]
    bufferOffset = _serializer.uint32(obj.points.length, buffer, bufferOffset);
    obj.points.forEach((val) => {
      bufferOffset = geometry_msgs.msg.Point.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [clusters]
    bufferOffset = _arraySerializer.int32(obj.clusters, buffer, bufferOffset, null);
    // Serialize message field [channels]
    bufferOffset = _arraySerializer.int32(obj.channels, buffer, bufferOffset, null);
    // Serialize message field [is_3d]
    bufferOffset = _serializer.bool(obj.is_3d, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Points
    let len;
    let data = new Points(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [count]
    data.count = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [points]
    // Deserialize array length for message field [points]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.points = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.points[i] = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [clusters]
    data.clusters = _arrayDeserializer.int32(buffer, bufferOffset, null)
    // Deserialize message field [channels]
    data.channels = _arrayDeserializer.int32(buffer, bufferOffset, null)
    // Deserialize message field [is_3d]
    data.is_3d = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += 24 * object.points.length;
    length += 4 * object.clusters.length;
    length += 4 * object.channels.length;
    return length + 17;
  }

  static datatype() {
    // Returns string type for a message object
    return 'clustering/Points';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '33601f15053b6bb6f55b6bf5417eecb7';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Points(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.count !== undefined) {
      resolved.count = msg.count;
    }
    else {
      resolved.count = 0
    }

    if (msg.points !== undefined) {
      resolved.points = new Array(msg.points.length);
      for (let i = 0; i < resolved.points.length; ++i) {
        resolved.points[i] = geometry_msgs.msg.Point.Resolve(msg.points[i]);
      }
    }
    else {
      resolved.points = []
    }

    if (msg.clusters !== undefined) {
      resolved.clusters = msg.clusters;
    }
    else {
      resolved.clusters = []
    }

    if (msg.channels !== undefined) {
      resolved.channels = msg.channels;
    }
    else {
      resolved.channels = []
    }

    if (msg.is_3d !== undefined) {
      resolved.is_3d = msg.is_3d;
    }
    else {
      resolved.is_3d = false
    }

    return resolved;
    }
};

module.exports = Points;
