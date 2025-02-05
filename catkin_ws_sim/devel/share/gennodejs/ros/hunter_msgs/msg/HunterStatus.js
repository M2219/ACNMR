// Auto-generated. Do not edit!

// (in-package hunter_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let HunterMotorState = require('./HunterMotorState.js');
let HunterDriverState = require('./HunterDriverState.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class HunterStatus {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.linear_velocity = null;
      this.steering_angle = null;
      this.base_state = null;
      this.control_mode = null;
      this.fault_code = null;
      this.battery_voltage = null;
      this.park_mode = null;
      this.motor_states = null;
      this.driver_states = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('linear_velocity')) {
        this.linear_velocity = initObj.linear_velocity
      }
      else {
        this.linear_velocity = 0.0;
      }
      if (initObj.hasOwnProperty('steering_angle')) {
        this.steering_angle = initObj.steering_angle
      }
      else {
        this.steering_angle = 0.0;
      }
      if (initObj.hasOwnProperty('base_state')) {
        this.base_state = initObj.base_state
      }
      else {
        this.base_state = 0;
      }
      if (initObj.hasOwnProperty('control_mode')) {
        this.control_mode = initObj.control_mode
      }
      else {
        this.control_mode = 0;
      }
      if (initObj.hasOwnProperty('fault_code')) {
        this.fault_code = initObj.fault_code
      }
      else {
        this.fault_code = 0;
      }
      if (initObj.hasOwnProperty('battery_voltage')) {
        this.battery_voltage = initObj.battery_voltage
      }
      else {
        this.battery_voltage = 0.0;
      }
      if (initObj.hasOwnProperty('park_mode')) {
        this.park_mode = initObj.park_mode
      }
      else {
        this.park_mode = 0;
      }
      if (initObj.hasOwnProperty('motor_states')) {
        this.motor_states = initObj.motor_states
      }
      else {
        this.motor_states = new Array(3).fill(new HunterMotorState());
      }
      if (initObj.hasOwnProperty('driver_states')) {
        this.driver_states = initObj.driver_states
      }
      else {
        this.driver_states = new Array(3).fill(new HunterDriverState());
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type HunterStatus
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [linear_velocity]
    bufferOffset = _serializer.float64(obj.linear_velocity, buffer, bufferOffset);
    // Serialize message field [steering_angle]
    bufferOffset = _serializer.float64(obj.steering_angle, buffer, bufferOffset);
    // Serialize message field [base_state]
    bufferOffset = _serializer.uint8(obj.base_state, buffer, bufferOffset);
    // Serialize message field [control_mode]
    bufferOffset = _serializer.uint8(obj.control_mode, buffer, bufferOffset);
    // Serialize message field [fault_code]
    bufferOffset = _serializer.uint16(obj.fault_code, buffer, bufferOffset);
    // Serialize message field [battery_voltage]
    bufferOffset = _serializer.float64(obj.battery_voltage, buffer, bufferOffset);
    // Serialize message field [park_mode]
    bufferOffset = _serializer.uint8(obj.park_mode, buffer, bufferOffset);
    // Check that the constant length array field [motor_states] has the right length
    if (obj.motor_states.length !== 3) {
      throw new Error('Unable to serialize array field motor_states - length must be 3')
    }
    // Serialize message field [motor_states]
    obj.motor_states.forEach((val) => {
      bufferOffset = HunterMotorState.serialize(val, buffer, bufferOffset);
    });
    // Check that the constant length array field [driver_states] has the right length
    if (obj.driver_states.length !== 3) {
      throw new Error('Unable to serialize array field driver_states - length must be 3')
    }
    // Serialize message field [driver_states]
    obj.driver_states.forEach((val) => {
      bufferOffset = HunterDriverState.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type HunterStatus
    let len;
    let data = new HunterStatus(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [linear_velocity]
    data.linear_velocity = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [steering_angle]
    data.steering_angle = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [base_state]
    data.base_state = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [control_mode]
    data.control_mode = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [fault_code]
    data.fault_code = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [battery_voltage]
    data.battery_voltage = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [park_mode]
    data.park_mode = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [motor_states]
    len = 3;
    data.motor_states = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.motor_states[i] = HunterMotorState.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [driver_states]
    len = 3;
    data.driver_states = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.driver_states[i] = HunterDriverState.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 176;
  }

  static datatype() {
    // Returns string type for a message object
    return 'hunter_msgs/HunterStatus';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ca557f5b13e1dbb8e7caa7cc072ca03c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    
    int8 MOTOR_ID_FRONT = 0
    int8 MOTOR_ID_REAR_LEFT = 1
    int8 MOTOR_ID_REAR_RIGHT = 2
    
    # motion state
    float64 linear_velocity
    float64 steering_angle
    
    # base state
    uint8 base_state
    uint8 control_mode
    uint16 fault_code
    float64 battery_voltage
    uint8 park_mode
    
    # motor state
    HunterMotorState[3] motor_states
    # driver state
    HunterDriverState[3] driver_states
    
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
    MSG: hunter_msgs/HunterMotorState
    float64 current
    float64 rpm
    float64 temperature
    float64 motor_pose
    
    ================================================================================
    MSG: hunter_msgs/HunterDriverState
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
    const resolved = new HunterStatus(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.linear_velocity !== undefined) {
      resolved.linear_velocity = msg.linear_velocity;
    }
    else {
      resolved.linear_velocity = 0.0
    }

    if (msg.steering_angle !== undefined) {
      resolved.steering_angle = msg.steering_angle;
    }
    else {
      resolved.steering_angle = 0.0
    }

    if (msg.base_state !== undefined) {
      resolved.base_state = msg.base_state;
    }
    else {
      resolved.base_state = 0
    }

    if (msg.control_mode !== undefined) {
      resolved.control_mode = msg.control_mode;
    }
    else {
      resolved.control_mode = 0
    }

    if (msg.fault_code !== undefined) {
      resolved.fault_code = msg.fault_code;
    }
    else {
      resolved.fault_code = 0
    }

    if (msg.battery_voltage !== undefined) {
      resolved.battery_voltage = msg.battery_voltage;
    }
    else {
      resolved.battery_voltage = 0.0
    }

    if (msg.park_mode !== undefined) {
      resolved.park_mode = msg.park_mode;
    }
    else {
      resolved.park_mode = 0
    }

    if (msg.motor_states !== undefined) {
      resolved.motor_states = new Array(3)
      for (let i = 0; i < resolved.motor_states.length; ++i) {
        if (msg.motor_states.length > i) {
          resolved.motor_states[i] = HunterMotorState.Resolve(msg.motor_states[i]);
        }
        else {
          resolved.motor_states[i] = new HunterMotorState();
        }
      }
    }
    else {
      resolved.motor_states = new Array(3).fill(new HunterMotorState())
    }

    if (msg.driver_states !== undefined) {
      resolved.driver_states = new Array(3)
      for (let i = 0; i < resolved.driver_states.length; ++i) {
        if (msg.driver_states.length > i) {
          resolved.driver_states[i] = HunterDriverState.Resolve(msg.driver_states[i]);
        }
        else {
          resolved.driver_states[i] = new HunterDriverState();
        }
      }
    }
    else {
      resolved.driver_states = new Array(3).fill(new HunterDriverState())
    }

    return resolved;
    }
};

// Constants for message
HunterStatus.Constants = {
  MOTOR_ID_FRONT: 0,
  MOTOR_ID_REAR_LEFT: 1,
  MOTOR_ID_REAR_RIGHT: 2,
}

module.exports = HunterStatus;
