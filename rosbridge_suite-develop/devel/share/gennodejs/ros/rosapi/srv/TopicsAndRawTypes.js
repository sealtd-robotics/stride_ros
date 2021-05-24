// Auto-generated. Do not edit!

// (in-package rosapi.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class TopicsAndRawTypesRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type TopicsAndRawTypesRequest
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type TopicsAndRawTypesRequest
    let len;
    let data = new TopicsAndRawTypesRequest(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'rosapi/TopicsAndRawTypesRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd41d8cd98f00b204e9800998ecf8427e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new TopicsAndRawTypesRequest(null);
    return resolved;
    }
};

class TopicsAndRawTypesResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.topics = null;
      this.types = null;
      this.typedefs_full_text = null;
    }
    else {
      if (initObj.hasOwnProperty('topics')) {
        this.topics = initObj.topics
      }
      else {
        this.topics = [];
      }
      if (initObj.hasOwnProperty('types')) {
        this.types = initObj.types
      }
      else {
        this.types = [];
      }
      if (initObj.hasOwnProperty('typedefs_full_text')) {
        this.typedefs_full_text = initObj.typedefs_full_text
      }
      else {
        this.typedefs_full_text = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type TopicsAndRawTypesResponse
    // Serialize message field [topics]
    bufferOffset = _arraySerializer.string(obj.topics, buffer, bufferOffset, null);
    // Serialize message field [types]
    bufferOffset = _arraySerializer.string(obj.types, buffer, bufferOffset, null);
    // Serialize message field [typedefs_full_text]
    bufferOffset = _arraySerializer.string(obj.typedefs_full_text, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type TopicsAndRawTypesResponse
    let len;
    let data = new TopicsAndRawTypesResponse(null);
    // Deserialize message field [topics]
    data.topics = _arrayDeserializer.string(buffer, bufferOffset, null)
    // Deserialize message field [types]
    data.types = _arrayDeserializer.string(buffer, bufferOffset, null)
    // Deserialize message field [typedefs_full_text]
    data.typedefs_full_text = _arrayDeserializer.string(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.topics.forEach((val) => {
      length += 4 + val.length;
    });
    object.types.forEach((val) => {
      length += 4 + val.length;
    });
    object.typedefs_full_text.forEach((val) => {
      length += 4 + val.length;
    });
    return length + 12;
  }

  static datatype() {
    // Returns string type for a service object
    return 'rosapi/TopicsAndRawTypesResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'e1432466c8f64316723276ba07c59d12';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string[] topics
    string[] types
    string[] typedefs_full_text
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new TopicsAndRawTypesResponse(null);
    if (msg.topics !== undefined) {
      resolved.topics = msg.topics;
    }
    else {
      resolved.topics = []
    }

    if (msg.types !== undefined) {
      resolved.types = msg.types;
    }
    else {
      resolved.types = []
    }

    if (msg.typedefs_full_text !== undefined) {
      resolved.typedefs_full_text = msg.typedefs_full_text;
    }
    else {
      resolved.typedefs_full_text = []
    }

    return resolved;
    }
};

module.exports = {
  Request: TopicsAndRawTypesRequest,
  Response: TopicsAndRawTypesResponse,
  md5sum() { return 'e1432466c8f64316723276ba07c59d12'; },
  datatype() { return 'rosapi/TopicsAndRawTypes'; }
};
