// Auto-generated. Do not edit!

// (in-package hunter_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class HunterDriverState {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.driver_voltage = null;
      this.driver_temperature = null;
      this.driver_state = null;
    }
    else {
      if (initObj.hasOwnProperty('driver_voltage')) {
        this.driver_voltage = initObj.driver_voltage
      }
      else {
        this.driver_voltage = 0.0;
      }
      if (initObj.hasOwnProperty('driver_temperature')) {
        this.driver_temperature = initObj.driver_temperature
      }
      else {
        this.driver_temperature = 0.0;
      }
      if (initObj.hasOwnProperty('driver_state')) {
        this.driver_state = initObj.driver_state
      }
      else {
        this.driver_state = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type HunterDriverState
    // Serialize message field [driver_voltage]
    bufferOffset = _serializer.float64(obj.driver_voltage, buffer, bufferOffset);
    // Serialize message field [driver_temperature]
    bufferOffset = _serializer.float64(obj.driver_temperature, buffer, bufferOffset);
    // Serialize message field [driver_state]
    bufferOffset = _serializer.uint8(obj.driver_state, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type HunterDriverState
    let len;
    let data = new HunterDriverState(null);
    // Deserialize message field [driver_voltage]
    data.driver_voltage = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [driver_temperature]
    data.driver_temperature = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [driver_state]
    data.driver_state = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 17;
  }

  static datatype() {
    // Returns string type for a message object
    return 'hunter_msgs/HunterDriverState';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'bcb2bfdfab7869cfb37bcfdc3a86c1d4';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 driver_voltage
    float64 driver_temperature
    uint8   driver_state
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new HunterDriverState(null);
    if (msg.driver_voltage !== undefined) {
      resolved.driver_voltage = msg.driver_voltage;
    }
    else {
      resolved.driver_voltage = 0.0
    }

    if (msg.driver_temperature !== undefined) {
      resolved.driver_temperature = msg.driver_temperature;
    }
    else {
      resolved.driver_temperature = 0.0
    }

    if (msg.driver_state !== undefined) {
      resolved.driver_state = msg.driver_state;
    }
    else {
      resolved.driver_state = 0
    }

    return resolved;
    }
};

module.exports = HunterDriverState;
