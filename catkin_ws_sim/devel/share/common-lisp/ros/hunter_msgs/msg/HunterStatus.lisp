; Auto-generated. Do not edit!


(cl:in-package hunter_msgs-msg)


;//! \htmlinclude HunterStatus.msg.html

(cl:defclass <HunterStatus> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (linear_velocity
    :reader linear_velocity
    :initarg :linear_velocity
    :type cl:float
    :initform 0.0)
   (steering_angle
    :reader steering_angle
    :initarg :steering_angle
    :type cl:float
    :initform 0.0)
   (base_state
    :reader base_state
    :initarg :base_state
    :type cl:fixnum
    :initform 0)
   (control_mode
    :reader control_mode
    :initarg :control_mode
    :type cl:fixnum
    :initform 0)
   (fault_code
    :reader fault_code
    :initarg :fault_code
    :type cl:fixnum
    :initform 0)
   (battery_voltage
    :reader battery_voltage
    :initarg :battery_voltage
    :type cl:float
    :initform 0.0)
   (park_mode
    :reader park_mode
    :initarg :park_mode
    :type cl:fixnum
    :initform 0)
   (motor_states
    :reader motor_states
    :initarg :motor_states
    :type (cl:vector hunter_msgs-msg:HunterMotorState)
   :initform (cl:make-array 3 :element-type 'hunter_msgs-msg:HunterMotorState :initial-element (cl:make-instance 'hunter_msgs-msg:HunterMotorState)))
   (driver_states
    :reader driver_states
    :initarg :driver_states
    :type (cl:vector hunter_msgs-msg:HunterDriverState)
   :initform (cl:make-array 3 :element-type 'hunter_msgs-msg:HunterDriverState :initial-element (cl:make-instance 'hunter_msgs-msg:HunterDriverState))))
)

(cl:defclass HunterStatus (<HunterStatus>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <HunterStatus>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'HunterStatus)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hunter_msgs-msg:<HunterStatus> is deprecated: use hunter_msgs-msg:HunterStatus instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <HunterStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hunter_msgs-msg:header-val is deprecated.  Use hunter_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'linear_velocity-val :lambda-list '(m))
(cl:defmethod linear_velocity-val ((m <HunterStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hunter_msgs-msg:linear_velocity-val is deprecated.  Use hunter_msgs-msg:linear_velocity instead.")
  (linear_velocity m))

(cl:ensure-generic-function 'steering_angle-val :lambda-list '(m))
(cl:defmethod steering_angle-val ((m <HunterStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hunter_msgs-msg:steering_angle-val is deprecated.  Use hunter_msgs-msg:steering_angle instead.")
  (steering_angle m))

(cl:ensure-generic-function 'base_state-val :lambda-list '(m))
(cl:defmethod base_state-val ((m <HunterStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hunter_msgs-msg:base_state-val is deprecated.  Use hunter_msgs-msg:base_state instead.")
  (base_state m))

(cl:ensure-generic-function 'control_mode-val :lambda-list '(m))
(cl:defmethod control_mode-val ((m <HunterStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hunter_msgs-msg:control_mode-val is deprecated.  Use hunter_msgs-msg:control_mode instead.")
  (control_mode m))

(cl:ensure-generic-function 'fault_code-val :lambda-list '(m))
(cl:defmethod fault_code-val ((m <HunterStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hunter_msgs-msg:fault_code-val is deprecated.  Use hunter_msgs-msg:fault_code instead.")
  (fault_code m))

(cl:ensure-generic-function 'battery_voltage-val :lambda-list '(m))
(cl:defmethod battery_voltage-val ((m <HunterStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hunter_msgs-msg:battery_voltage-val is deprecated.  Use hunter_msgs-msg:battery_voltage instead.")
  (battery_voltage m))

(cl:ensure-generic-function 'park_mode-val :lambda-list '(m))
(cl:defmethod park_mode-val ((m <HunterStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hunter_msgs-msg:park_mode-val is deprecated.  Use hunter_msgs-msg:park_mode instead.")
  (park_mode m))

(cl:ensure-generic-function 'motor_states-val :lambda-list '(m))
(cl:defmethod motor_states-val ((m <HunterStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hunter_msgs-msg:motor_states-val is deprecated.  Use hunter_msgs-msg:motor_states instead.")
  (motor_states m))

(cl:ensure-generic-function 'driver_states-val :lambda-list '(m))
(cl:defmethod driver_states-val ((m <HunterStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hunter_msgs-msg:driver_states-val is deprecated.  Use hunter_msgs-msg:driver_states instead.")
  (driver_states m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<HunterStatus>)))
    "Constants for message type '<HunterStatus>"
  '((:MOTOR_ID_FRONT . 0)
    (:MOTOR_ID_REAR_LEFT . 1)
    (:MOTOR_ID_REAR_RIGHT . 2))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'HunterStatus)))
    "Constants for message type 'HunterStatus"
  '((:MOTOR_ID_FRONT . 0)
    (:MOTOR_ID_REAR_LEFT . 1)
    (:MOTOR_ID_REAR_RIGHT . 2))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <HunterStatus>) ostream)
  "Serializes a message object of type '<HunterStatus>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'linear_velocity))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'steering_angle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'base_state)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'control_mode)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'fault_code)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'fault_code)) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'battery_voltage))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'park_mode)) ostream)
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'motor_states))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'driver_states))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <HunterStatus>) istream)
  "Deserializes a message object of type '<HunterStatus>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'linear_velocity) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'steering_angle) (roslisp-utils:decode-double-float-bits bits)))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'base_state)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'control_mode)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'fault_code)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'fault_code)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'battery_voltage) (roslisp-utils:decode-double-float-bits bits)))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'park_mode)) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'motor_states) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'motor_states)))
    (cl:dotimes (i 3)
    (cl:setf (cl:aref vals i) (cl:make-instance 'hunter_msgs-msg:HunterMotorState))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream)))
  (cl:setf (cl:slot-value msg 'driver_states) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'driver_states)))
    (cl:dotimes (i 3)
    (cl:setf (cl:aref vals i) (cl:make-instance 'hunter_msgs-msg:HunterDriverState))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<HunterStatus>)))
  "Returns string type for a message object of type '<HunterStatus>"
  "hunter_msgs/HunterStatus")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'HunterStatus)))
  "Returns string type for a message object of type 'HunterStatus"
  "hunter_msgs/HunterStatus")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<HunterStatus>)))
  "Returns md5sum for a message object of type '<HunterStatus>"
  "ca557f5b13e1dbb8e7caa7cc072ca03c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'HunterStatus)))
  "Returns md5sum for a message object of type 'HunterStatus"
  "ca557f5b13e1dbb8e7caa7cc072ca03c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<HunterStatus>)))
  "Returns full string definition for message of type '<HunterStatus>"
  (cl:format cl:nil "Header header~%~%int8 MOTOR_ID_FRONT = 0~%int8 MOTOR_ID_REAR_LEFT = 1~%int8 MOTOR_ID_REAR_RIGHT = 2~%~%# motion state~%float64 linear_velocity~%float64 steering_angle~%~%# base state~%uint8 base_state~%uint8 control_mode~%uint16 fault_code~%float64 battery_voltage~%uint8 park_mode~%~%# motor state~%HunterMotorState[3] motor_states~%# driver state~%HunterDriverState[3] driver_states~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: hunter_msgs/HunterMotorState~%float64 current~%float64 rpm~%float64 temperature~%float64 motor_pose~%~%================================================================================~%MSG: hunter_msgs/HunterDriverState~%float64 driver_voltage~%float64 driver_temperature~%uint8   driver_state~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'HunterStatus)))
  "Returns full string definition for message of type 'HunterStatus"
  (cl:format cl:nil "Header header~%~%int8 MOTOR_ID_FRONT = 0~%int8 MOTOR_ID_REAR_LEFT = 1~%int8 MOTOR_ID_REAR_RIGHT = 2~%~%# motion state~%float64 linear_velocity~%float64 steering_angle~%~%# base state~%uint8 base_state~%uint8 control_mode~%uint16 fault_code~%float64 battery_voltage~%uint8 park_mode~%~%# motor state~%HunterMotorState[3] motor_states~%# driver state~%HunterDriverState[3] driver_states~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: hunter_msgs/HunterMotorState~%float64 current~%float64 rpm~%float64 temperature~%float64 motor_pose~%~%================================================================================~%MSG: hunter_msgs/HunterDriverState~%float64 driver_voltage~%float64 driver_temperature~%uint8   driver_state~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <HunterStatus>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     8
     8
     1
     1
     2
     8
     1
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'motor_states) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'driver_states) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <HunterStatus>))
  "Converts a ROS message object to a list"
  (cl:list 'HunterStatus
    (cl:cons ':header (header msg))
    (cl:cons ':linear_velocity (linear_velocity msg))
    (cl:cons ':steering_angle (steering_angle msg))
    (cl:cons ':base_state (base_state msg))
    (cl:cons ':control_mode (control_mode msg))
    (cl:cons ':fault_code (fault_code msg))
    (cl:cons ':battery_voltage (battery_voltage msg))
    (cl:cons ':park_mode (park_mode msg))
    (cl:cons ':motor_states (motor_states msg))
    (cl:cons ':driver_states (driver_states msg))
))
