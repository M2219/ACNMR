; Auto-generated. Do not edit!


(cl:in-package hunter_msgs-msg)


;//! \htmlinclude HunterBmsStatus.msg.html

(cl:defclass <HunterBmsStatus> (roslisp-msg-protocol:ros-message)
  ((SOC
    :reader SOC
    :initarg :SOC
    :type cl:fixnum
    :initform 0)
   (SOH
    :reader SOH
    :initarg :SOH
    :type cl:fixnum
    :initform 0)
   (battery_voltage
    :reader battery_voltage
    :initarg :battery_voltage
    :type cl:float
    :initform 0.0)
   (battery_current
    :reader battery_current
    :initarg :battery_current
    :type cl:float
    :initform 0.0)
   (battery_temperature
    :reader battery_temperature
    :initarg :battery_temperature
    :type cl:float
    :initform 0.0)
   (Alarm_Status_1
    :reader Alarm_Status_1
    :initarg :Alarm_Status_1
    :type cl:fixnum
    :initform 0)
   (Alarm_Status_2
    :reader Alarm_Status_2
    :initarg :Alarm_Status_2
    :type cl:fixnum
    :initform 0)
   (Warning_Status_1
    :reader Warning_Status_1
    :initarg :Warning_Status_1
    :type cl:fixnum
    :initform 0)
   (Warning_Status_2
    :reader Warning_Status_2
    :initarg :Warning_Status_2
    :type cl:fixnum
    :initform 0))
)

(cl:defclass HunterBmsStatus (<HunterBmsStatus>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <HunterBmsStatus>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'HunterBmsStatus)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hunter_msgs-msg:<HunterBmsStatus> is deprecated: use hunter_msgs-msg:HunterBmsStatus instead.")))

(cl:ensure-generic-function 'SOC-val :lambda-list '(m))
(cl:defmethod SOC-val ((m <HunterBmsStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hunter_msgs-msg:SOC-val is deprecated.  Use hunter_msgs-msg:SOC instead.")
  (SOC m))

(cl:ensure-generic-function 'SOH-val :lambda-list '(m))
(cl:defmethod SOH-val ((m <HunterBmsStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hunter_msgs-msg:SOH-val is deprecated.  Use hunter_msgs-msg:SOH instead.")
  (SOH m))

(cl:ensure-generic-function 'battery_voltage-val :lambda-list '(m))
(cl:defmethod battery_voltage-val ((m <HunterBmsStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hunter_msgs-msg:battery_voltage-val is deprecated.  Use hunter_msgs-msg:battery_voltage instead.")
  (battery_voltage m))

(cl:ensure-generic-function 'battery_current-val :lambda-list '(m))
(cl:defmethod battery_current-val ((m <HunterBmsStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hunter_msgs-msg:battery_current-val is deprecated.  Use hunter_msgs-msg:battery_current instead.")
  (battery_current m))

(cl:ensure-generic-function 'battery_temperature-val :lambda-list '(m))
(cl:defmethod battery_temperature-val ((m <HunterBmsStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hunter_msgs-msg:battery_temperature-val is deprecated.  Use hunter_msgs-msg:battery_temperature instead.")
  (battery_temperature m))

(cl:ensure-generic-function 'Alarm_Status_1-val :lambda-list '(m))
(cl:defmethod Alarm_Status_1-val ((m <HunterBmsStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hunter_msgs-msg:Alarm_Status_1-val is deprecated.  Use hunter_msgs-msg:Alarm_Status_1 instead.")
  (Alarm_Status_1 m))

(cl:ensure-generic-function 'Alarm_Status_2-val :lambda-list '(m))
(cl:defmethod Alarm_Status_2-val ((m <HunterBmsStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hunter_msgs-msg:Alarm_Status_2-val is deprecated.  Use hunter_msgs-msg:Alarm_Status_2 instead.")
  (Alarm_Status_2 m))

(cl:ensure-generic-function 'Warning_Status_1-val :lambda-list '(m))
(cl:defmethod Warning_Status_1-val ((m <HunterBmsStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hunter_msgs-msg:Warning_Status_1-val is deprecated.  Use hunter_msgs-msg:Warning_Status_1 instead.")
  (Warning_Status_1 m))

(cl:ensure-generic-function 'Warning_Status_2-val :lambda-list '(m))
(cl:defmethod Warning_Status_2-val ((m <HunterBmsStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hunter_msgs-msg:Warning_Status_2-val is deprecated.  Use hunter_msgs-msg:Warning_Status_2 instead.")
  (Warning_Status_2 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <HunterBmsStatus>) ostream)
  "Serializes a message object of type '<HunterBmsStatus>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'SOC)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'SOH)) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'battery_voltage))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'battery_current))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'battery_temperature))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'Alarm_Status_1)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'Alarm_Status_2)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'Warning_Status_1)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'Warning_Status_2)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <HunterBmsStatus>) istream)
  "Deserializes a message object of type '<HunterBmsStatus>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'SOC)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'SOH)) (cl:read-byte istream))
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
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'battery_current) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'battery_temperature) (roslisp-utils:decode-double-float-bits bits)))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'Alarm_Status_1)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'Alarm_Status_2)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'Warning_Status_1)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'Warning_Status_2)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<HunterBmsStatus>)))
  "Returns string type for a message object of type '<HunterBmsStatus>"
  "hunter_msgs/HunterBmsStatus")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'HunterBmsStatus)))
  "Returns string type for a message object of type 'HunterBmsStatus"
  "hunter_msgs/HunterBmsStatus")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<HunterBmsStatus>)))
  "Returns md5sum for a message object of type '<HunterBmsStatus>"
  "fcfbadcd70fcf25cf217105d70ab4e45")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'HunterBmsStatus)))
  "Returns md5sum for a message object of type 'HunterBmsStatus"
  "fcfbadcd70fcf25cf217105d70ab4e45")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<HunterBmsStatus>)))
  "Returns full string definition for message of type '<HunterBmsStatus>"
  (cl:format cl:nil "#BMS date~%uint8 SOC~%uint8 SOH~%float64 battery_voltage~%float64 battery_current~%float64 battery_temperature~%~%#BMS status~%uint8 Alarm_Status_1~%uint8 Alarm_Status_2~%uint8 Warning_Status_1~%uint8 Warning_Status_2~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'HunterBmsStatus)))
  "Returns full string definition for message of type 'HunterBmsStatus"
  (cl:format cl:nil "#BMS date~%uint8 SOC~%uint8 SOH~%float64 battery_voltage~%float64 battery_current~%float64 battery_temperature~%~%#BMS status~%uint8 Alarm_Status_1~%uint8 Alarm_Status_2~%uint8 Warning_Status_1~%uint8 Warning_Status_2~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <HunterBmsStatus>))
  (cl:+ 0
     1
     1
     8
     8
     8
     1
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <HunterBmsStatus>))
  "Converts a ROS message object to a list"
  (cl:list 'HunterBmsStatus
    (cl:cons ':SOC (SOC msg))
    (cl:cons ':SOH (SOH msg))
    (cl:cons ':battery_voltage (battery_voltage msg))
    (cl:cons ':battery_current (battery_current msg))
    (cl:cons ':battery_temperature (battery_temperature msg))
    (cl:cons ':Alarm_Status_1 (Alarm_Status_1 msg))
    (cl:cons ':Alarm_Status_2 (Alarm_Status_2 msg))
    (cl:cons ':Warning_Status_1 (Warning_Status_1 msg))
    (cl:cons ':Warning_Status_2 (Warning_Status_2 msg))
))
