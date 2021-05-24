// Auto-generated. Do not edit!

// (in-package can_interface.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class WheelRPM {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.left_front = null;
      this.left_back = null;
      this.right_front = null;
      this.right_back = null;
    }
    else {
      if (initObj.hasOwnProperty('left_front')) {
        this.left_front = initObj.left_front
      }
      else {
        this.left_front = 0.0;
      }
      if (initObj.hasOwnProperty('left_back')) {
        this.left_back = initObj.left_back
      }
      else {
        this.left_back = 0.0;
      }
      if (initObj.hasOwnProperty('right_front')) {
        this.right_front = initObj.right_front
      }
      else {
        this.right_front = 0.0;
      }
      if (initObj.hasOwnProperty('right_back')) {
        this.right_back = initObj.right_back
      }
      else {
        this.right_back = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type WheelRPM
    // Serialize message field [left_front]
    bufferOffset = _serializer.float32(obj.left_front, buffer, bufferOffset);
    // Serialize message field [left_back]
    bufferOffset = _serializer.float32(obj.left_back, buffer, bufferOffset);
    // Serialize message field [right_front]
    bufferOffset = _serializer.float32(obj.right_front, buffer, bufferOffset);
    // Serialize message field [right_back]
    bufferOffset = _serializer.float32(obj.right_back, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type WheelRPM
    let len;
    let data = new WheelRPM(null);
    // Deserialize message field [left_front]
    data.left_front = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [left_back]
    data.left_back = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [right_front]
    data.right_front = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [right_back]
    data.right_back = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'can_interface/WheelRPM';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '7409db75cdf11985ad54203bb29757ca';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32 left_front
    float32 left_back
    float32 right_front
    float32 right_back
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new WheelRPM(null);
    if (msg.left_front !== undefined) {
      resolved.left_front = msg.left_front;
    }
    else {
      resolved.left_front = 0.0
    }

    if (msg.left_back !== undefined) {
      resolved.left_back = msg.left_back;
    }
    else {
      resolved.left_back = 0.0
    }

    if (msg.right_front !== undefined) {
      resolved.right_front = msg.right_front;
    }
    else {
      resolved.right_front = 0.0
    }

    if (msg.right_back !== undefined) {
      resolved.right_back = msg.right_back;
    }
    else {
      resolved.right_back = 0.0
    }

    return resolved;
    }
};

module.exports = WheelRPM;
