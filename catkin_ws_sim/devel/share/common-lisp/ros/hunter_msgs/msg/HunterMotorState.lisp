; Auto-generated. Do not edit!


(cl:in-package hunter_msgs-msg)


;//! \htmlinclude HunterMotorState.msg.html

(cl:defclass <HunterMotorState> (roslisp-msg-protocol:ros-message)
  ((current
    :reader current
    :initarg :current
    :type cl:float
    :initform 0.0)
   (rpm
    :reader rpm
    :initarg :rpm
    :type cl:float
    :initform 0.0)
   (temperature
    :reader temperature
    :initarg :temperature
    :type cl:float
    :initform 0.0)
   (motor_pose
    :reader motor_pose
    :initarg :motor_pose
    :type cl:float
    :initform 0.0))
)

(cl:defclass HunterMotorState (<HunterMotorState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <HunterMotorState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'HunterMotorState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hunter_msgs-msg:<HunterMotorState> is deprecated: use hunter_msgs-msg:HunterMotorState instead.")))

(cl:ensure-generic-function 'current-val :lambda-list '(m))
(cl:defmethod current-val ((m <HunterMotorState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hunter_msgs-msg:current-val is deprecated.  Use hunter_msgs-msg:current instead.")
  (current m))

(cl:ensure-generic-function 'rpm-val :lambda-list '(m))
(cl:defmethod rpm-val ((m <HunterMotorState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hunter_msgs-msg:rpm-val is deprecated.  Use hunter_msgs-msg:rpm instead.")
  (rpm m))

(cl:ensure-generic-function 'temperature-val :lambda-list '(m))
(cl:defmethod temperature-val ((m <HunterMotorState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hunter_msgs-msg:temperature-val is deprecated.  Use hunter_msgs-msg:temperature instead.")
  (temperature m))

(cl:ensure-generic-function 'motor_pose-val :lambda-list '(m))
(cl:defmethod motor_pose-val ((m <HunterMotorState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hunter_msgs-msg:motor_pose-val is deprecated.  Use hunter_msgs-msg:motor_pose instead.")
  (motor_pose m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <HunterMotorState>) ostream)
  "Serializes a message object of type '<HunterMotorState>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'current))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'rpm))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'temperature))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'motor_pose))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <HunterMotorState>) istream)
  "Deserializes a message object of type '<HunterMotorState>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'current) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'rpm) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'temperature) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'motor_pose) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<HunterMotorState>)))
  "Returns string type for a message object of type '<HunterMotorState>"
  "hunter_msgs/HunterMotorState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'HunterMotorState)))
  "Returns string type for a message object of type 'HunterMotorState"
  "hunter_msgs/HunterMotorState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<HunterMotorState>)))
  "Returns md5sum for a message object of type '<HunterMotorState>"
  "6d4fa3048ea5c7fdf6db5094fce69da0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'HunterMotorState)))
  "Returns md5sum for a message object of type 'HunterMotorState"
  "6d4fa3048ea5c7fdf6db5094fce69da0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<HunterMotorState>)))
  "Returns full string definition for message of type '<HunterMotorState>"
  (cl:format cl:nil "float64 current~%float64 rpm~%float64 temperature~%float64 motor_pose~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'HunterMotorState)))
  "Returns full string definition for message of type 'HunterMotorState"
  (cl:format cl:nil "float64 current~%float64 rpm~%float64 temperature~%float64 motor_pose~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <HunterMotorState>))
  (cl:+ 0
     8
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <HunterMotorState>))
  "Converts a ROS message object to a list"
  (cl:list 'HunterMotorState
    (cl:cons ':current (current msg))
    (cl:cons ':rpm (rpm msg))
    (cl:cons ':temperature (temperature msg))
    (cl:cons ':motor_pose (motor_pose msg))
))
