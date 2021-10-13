// Auto-generated. Do not edit!

// (in-package darknet_ros_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let ComputeBox = require('./ComputeBox.js');

//-----------------------------------------------------------

class ComputeBoxes {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.compute_box = null;
    }
    else {
      if (initObj.hasOwnProperty('compute_box')) {
        this.compute_box = initObj.compute_box
      }
      else {
        this.compute_box = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ComputeBoxes
    // Serialize message field [compute_box]
    // Serialize the length for message field [compute_box]
    bufferOffset = _serializer.uint32(obj.compute_box.length, buffer, bufferOffset);
    obj.compute_box.forEach((val) => {
      bufferOffset = ComputeBox.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ComputeBoxes
    let len;
    let data = new ComputeBoxes(null);
    // Deserialize message field [compute_box]
    // Deserialize array length for message field [compute_box]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.compute_box = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.compute_box[i] = ComputeBox.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.compute_box.forEach((val) => {
      length += ComputeBox.getMessageSize(val);
    });
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'darknet_ros_msgs/ComputeBoxes';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '1a0189e0ccd8b7029250c17e4ab7ef8f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    ComputeBox[] compute_box
    ================================================================================
    MSG: darknet_ros_msgs/ComputeBox
    int16 id
    string Class
    float64 probability
    float64 xmin
    float64 ymin
    float64 xmax
    float64 ymax
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ComputeBoxes(null);
    if (msg.compute_box !== undefined) {
      resolved.compute_box = new Array(msg.compute_box.length);
      for (let i = 0; i < resolved.compute_box.length; ++i) {
        resolved.compute_box[i] = ComputeBox.Resolve(msg.compute_box[i]);
      }
    }
    else {
      resolved.compute_box = []
    }

    return resolved;
    }
};

module.exports = ComputeBoxes;
