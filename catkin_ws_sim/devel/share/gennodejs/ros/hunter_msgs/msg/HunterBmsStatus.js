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

class HunterBmsStatus {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.SOC = null;
      this.SOH = null;
      this.battery_voltage = null;
      this.battery_current = null;
      this.battery_temperature = null;
      this.Alarm_Status_1 = null;
      this.Alarm_Status_2 = null;
      this.Warning_Status_1 = null;
      this.Warning_Status_2 = null;
    }
    else {
      if (initObj.hasOwnProperty('SOC')) {
        this.SOC = initObj.SOC
      }
      else {
        this.SOC = 0;
      }
      if (initObj.hasOwnProperty('SOH')) {
        this.SOH = initObj.SOH
      }
      else {
        this.SOH = 0;
      }
      if (initObj.hasOwnProperty('battery_voltage')) {
        this.battery_voltage = initObj.battery_voltage
      }
      else {
        this.battery_voltage = 0.0;
      }
      if (initObj.hasOwnProperty('battery_current')) {
        this.battery_current = initObj.battery_current
      }
      else {
        this.battery_current = 0.0;
      }
      if (initObj.hasOwnProperty('battery_temperature')) {
        this.battery_temperature = initObj.battery_temperature
      }
      else {
        this.battery_temperature = 0.0;
      }
      if (initObj.hasOwnProperty('Alarm_Status_1')) {
        this.Alarm_Status_1 = initObj.Alarm_Status_1
      }
      else {
        this.Alarm_Status_1 = 0;
      }
      if (initObj.hasOwnProperty('Alarm_Status_2')) {
        this.Alarm_Status_2 = initObj.Alarm_Status_2
      }
      else {
        this.Alarm_Status_2 = 0;
      }
      if (initObj.hasOwnProperty('Warning_Status_1')) {
        this.Warning_Status_1 = initObj.Warning_Status_1
      }
      else {
        this.Warning_Status_1 = 0;
      }
      if (initObj.hasOwnProperty('Warning_Status_2')) {
        this.Warning_Status_2 = initObj.Warning_Status_2
      }
      else {
        this.Warning_Status_2 = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type HunterBmsStatus
    // Serialize message field [SOC]
    bufferOffset = _serializer.uint8(obj.SOC, buffer, bufferOffset);
    // Serialize message field [SOH]
    bufferOffset = _serializer.uint8(obj.SOH, buffer, bufferOffset);
    // Serialize message field [battery_voltage]
    bufferOffset = _serializer.float64(obj.battery_voltage, buffer, bufferOffset);
    // Serialize message field [battery_current]
    bufferOffset = _serializer.float64(obj.battery_current, buffer, bufferOffset);
    // Serialize message field [battery_temperature]
    bufferOffset = _serializer.float64(obj.battery_temperature, buffer, bufferOffset);
    // Serialize message field [Alarm_Status_1]
    bufferOffset = _serializer.uint8(obj.Alarm_Status_1, buffer, bufferOffset);
    // Serialize message field [Alarm_Status_2]
    bufferOffset = _serializer.uint8(obj.Alarm_Status_2, buffer, bufferOffset);
    // Serialize message field [Warning_Status_1]
    bufferOffset = _serializer.uint8(obj.Warning_Status_1, buffer, bufferOffset);
    // Serialize message field [Warning_Status_2]
    bufferOffset = _serializer.uint8(obj.Warning_Status_2, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type HunterBmsStatus
    let len;
    let data = new HunterBmsStatus(null);
    // Deserialize message field [SOC]
    data.SOC = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [SOH]
    data.SOH = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [battery_voltage]
    data.battery_voltage = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [battery_current]
    data.battery_current = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [battery_temperature]
    data.battery_temperature = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [Alarm_Status_1]
    data.Alarm_Status_1 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [Alarm_Status_2]
    data.Alarm_Status_2 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [Warning_Status_1]
    data.Warning_Status_1 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [Warning_Status_2]
    data.Warning_Status_2 = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 30;
  }

  static datatype() {
    // Returns string type for a message object
    return 'hunter_msgs/HunterBmsStatus';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'fcfbadcd70fcf25cf217105d70ab4e45';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    #BMS date
    uint8 SOC
    uint8 SOH
    float64 battery_voltage
    float64 battery_current
    float64 battery_temperature
    
    #BMS status
    uint8 Alarm_Status_1
    uint8 Alarm_Status_2
    uint8 Warning_Status_1
    uint8 Warning_Status_2
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new HunterBmsStatus(null);
    if (msg.SOC !== undefined) {
      resolved.SOC = msg.SOC;
    }
    else {
      resolved.SOC = 0
    }

    if (msg.SOH !== undefined) {
      resolved.SOH = msg.SOH;
    }
    else {
      resolved.SOH = 0
    }

    if (msg.battery_voltage !== undefined) {
      resolved.battery_voltage = msg.battery_voltage;
    }
    else {
      resolved.battery_voltage = 0.0
    }

    if (msg.battery_current !== undefined) {
      resolved.battery_current = msg.battery_current;
    }
    else {
      resolved.battery_current = 0.0
    }

    if (msg.battery_temperature !== undefined) {
      resolved.battery_temperature = msg.battery_temperature;
    }
    else {
      resolved.battery_temperature = 0.0
    }

    if (msg.Alarm_Status_1 !== undefined) {
      resolved.Alarm_Status_1 = msg.Alarm_Status_1;
    }
    else {
      resolved.Alarm_Status_1 = 0
    }

    if (msg.Alarm_Status_2 !== undefined) {
      resolved.Alarm_Status_2 = msg.Alarm_Status_2;
    }
    else {
      resolved.Alarm_Status_2 = 0
    }

    if (msg.Warning_Status_1 !== undefined) {
      resolved.Warning_Status_1 = msg.Warning_Status_1;
    }
    else {
      resolved.Warning_Status_1 = 0
    }

    if (msg.Warning_Status_2 !== undefined) {
      resolved.Warning_Status_2 = msg.Warning_Status_2;
    }
    else {
      resolved.Warning_Status_2 = 0
    }

    return resolved;
    }
};

module.exports = HunterBmsStatus;
