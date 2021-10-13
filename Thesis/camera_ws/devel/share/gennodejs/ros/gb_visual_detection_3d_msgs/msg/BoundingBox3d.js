// Auto-generated. Do not edit!

// (in-package gb_visual_detection_3d_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class BoundingBox3d {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.Class = null;
      this.probability = null;
      this.xmin = null;
      this.ymin = null;
      this.xmax = null;
      this.ymax = null;
      this.zmin = null;
      this.zmax = null;
      this.xmin_2d = null;
      this.ymin_2d = null;
      this.xmax_2d = null;
      this.ymax_2d = null;
    }
    else {
      if (initObj.hasOwnProperty('Class')) {
        this.Class = initObj.Class
      }
      else {
        this.Class = '';
      }
      if (initObj.hasOwnProperty('probability')) {
        this.probability = initObj.probability
      }
      else {
        this.probability = 0.0;
      }
      if (initObj.hasOwnProperty('xmin')) {
        this.xmin = initObj.xmin
      }
      else {
        this.xmin = 0.0;
      }
      if (initObj.hasOwnProperty('ymin')) {
        this.ymin = initObj.ymin
      }
      else {
        this.ymin = 0.0;
      }
      if (initObj.hasOwnProperty('xmax')) {
        this.xmax = initObj.xmax
      }
      else {
        this.xmax = 0.0;
      }
      if (initObj.hasOwnProperty('ymax')) {
        this.ymax = initObj.ymax
      }
      else {
        this.ymax = 0.0;
      }
      if (initObj.hasOwnProperty('zmin')) {
        this.zmin = initObj.zmin
      }
      else {
        this.zmin = 0.0;
      }
      if (initObj.hasOwnProperty('zmax')) {
        this.zmax = initObj.zmax
      }
      else {
        this.zmax = 0.0;
      }
      if (initObj.hasOwnProperty('xmin_2d')) {
        this.xmin_2d = initObj.xmin_2d
      }
      else {
        this.xmin_2d = 0;
      }
      if (initObj.hasOwnProperty('ymin_2d')) {
        this.ymin_2d = initObj.ymin_2d
      }
      else {
        this.ymin_2d = 0;
      }
      if (initObj.hasOwnProperty('xmax_2d')) {
        this.xmax_2d = initObj.xmax_2d
      }
      else {
        this.xmax_2d = 0;
      }
      if (initObj.hasOwnProperty('ymax_2d')) {
        this.ymax_2d = initObj.ymax_2d
      }
      else {
        this.ymax_2d = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type BoundingBox3d
    // Serialize message field [Class]
    bufferOffset = _serializer.string(obj.Class, buffer, bufferOffset);
    // Serialize message field [probability]
    bufferOffset = _serializer.float64(obj.probability, buffer, bufferOffset);
    // Serialize message field [xmin]
    bufferOffset = _serializer.float64(obj.xmin, buffer, bufferOffset);
    // Serialize message field [ymin]
    bufferOffset = _serializer.float64(obj.ymin, buffer, bufferOffset);
    // Serialize message field [xmax]
    bufferOffset = _serializer.float64(obj.xmax, buffer, bufferOffset);
    // Serialize message field [ymax]
    bufferOffset = _serializer.float64(obj.ymax, buffer, bufferOffset);
    // Serialize message field [zmin]
    bufferOffset = _serializer.float64(obj.zmin, buffer, bufferOffset);
    // Serialize message field [zmax]
    bufferOffset = _serializer.float64(obj.zmax, buffer, bufferOffset);
    // Serialize message field [xmin_2d]
    bufferOffset = _serializer.int64(obj.xmin_2d, buffer, bufferOffset);
    // Serialize message field [ymin_2d]
    bufferOffset = _serializer.int64(obj.ymin_2d, buffer, bufferOffset);
    // Serialize message field [xmax_2d]
    bufferOffset = _serializer.int64(obj.xmax_2d, buffer, bufferOffset);
    // Serialize message field [ymax_2d]
    bufferOffset = _serializer.int64(obj.ymax_2d, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type BoundingBox3d
    let len;
    let data = new BoundingBox3d(null);
    // Deserialize message field [Class]
    data.Class = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [probability]
    data.probability = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [xmin]
    data.xmin = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [ymin]
    data.ymin = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [xmax]
    data.xmax = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [ymax]
    data.ymax = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [zmin]
    data.zmin = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [zmax]
    data.zmax = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [xmin_2d]
    data.xmin_2d = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [ymin_2d]
    data.ymin_2d = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [xmax_2d]
    data.xmax_2d = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [ymax_2d]
    data.ymax_2d = _deserializer.int64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.Class.length;
    return length + 92;
  }

  static datatype() {
    // Returns string type for a message object
    return 'gb_visual_detection_3d_msgs/BoundingBox3d';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '2ce9b0738c577c56440a33d88e32daf4';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string Class
    float64 probability
    float64 xmin
    float64 ymin
    float64 xmax
    float64 ymax
    float64 zmin
    float64 zmax
    int64 xmin_2d
    int64 ymin_2d
    int64 xmax_2d
    int64 ymax_2d
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new BoundingBox3d(null);
    if (msg.Class !== undefined) {
      resolved.Class = msg.Class;
    }
    else {
      resolved.Class = ''
    }

    if (msg.probability !== undefined) {
      resolved.probability = msg.probability;
    }
    else {
      resolved.probability = 0.0
    }

    if (msg.xmin !== undefined) {
      resolved.xmin = msg.xmin;
    }
    else {
      resolved.xmin = 0.0
    }

    if (msg.ymin !== undefined) {
      resolved.ymin = msg.ymin;
    }
    else {
      resolved.ymin = 0.0
    }

    if (msg.xmax !== undefined) {
      resolved.xmax = msg.xmax;
    }
    else {
      resolved.xmax = 0.0
    }

    if (msg.ymax !== undefined) {
      resolved.ymax = msg.ymax;
    }
    else {
      resolved.ymax = 0.0
    }

    if (msg.zmin !== undefined) {
      resolved.zmin = msg.zmin;
    }
    else {
      resolved.zmin = 0.0
    }

    if (msg.zmax !== undefined) {
      resolved.zmax = msg.zmax;
    }
    else {
      resolved.zmax = 0.0
    }

    if (msg.xmin_2d !== undefined) {
      resolved.xmin_2d = msg.xmin_2d;
    }
    else {
      resolved.xmin_2d = 0
    }

    if (msg.ymin_2d !== undefined) {
      resolved.ymin_2d = msg.ymin_2d;
    }
    else {
      resolved.ymin_2d = 0
    }

    if (msg.xmax_2d !== undefined) {
      resolved.xmax_2d = msg.xmax_2d;
    }
    else {
      resolved.xmax_2d = 0
    }

    if (msg.ymax_2d !== undefined) {
      resolved.ymax_2d = msg.ymax_2d;
    }
    else {
      resolved.ymax_2d = 0
    }

    return resolved;
    }
};

module.exports = BoundingBox3d;
