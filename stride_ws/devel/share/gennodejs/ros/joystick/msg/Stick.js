// Auto-generated. Do not edit!

// (in-package joystick.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class Stick {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.travel = null;
      this.angle = null;
    }
    else {
      if (initObj.hasOwnProperty('travel')) {
        this.travel = initObj.travel
      }
      else {
        this.travel = 0.0;
      }
      if (initObj.hasOwnProperty('angle')) {
        this.angle = initObj.angle
      }
      else {
        this.angle = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Stick
    // Serialize message field [travel]
    bufferOffset = _serializer.float32(obj.travel, buffer, bufferOffset);
    // Serialize message field [angle]
    bufferOffset = _serializer.float32(obj.angle, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Stick
    let len;
    let data = new Stick(null);
    // Deserialize message field [travel]
    data.travel = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [angle]
    data.angle = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'joystick/Stick';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4ac9ad86115642357f119d6100be459c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32 travel # between 0 and 1
    float32 angle  # radian
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Stick(null);
    if (msg.travel !== undefined) {
      resolved.travel = msg.travel;
    }
    else {
      resolved.travel = 0.0
    }

    if (msg.angle !== undefined) {
      resolved.angle = msg.angle;
    }
    else {
      resolved.angle = 0.0
    }

    return resolved;
    }
};

module.exports = Stick;
