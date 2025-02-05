;; Auto-generated. Do not edit!


(when (boundp 'hunter_msgs::HunterStatus)
  (if (not (find-package "HUNTER_MSGS"))
    (make-package "HUNTER_MSGS"))
  (shadow 'HunterStatus (find-package "HUNTER_MSGS")))
(unless (find-package "HUNTER_MSGS::HUNTERSTATUS")
  (make-package "HUNTER_MSGS::HUNTERSTATUS"))

(in-package "ROS")
;;//! \htmlinclude HunterStatus.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(intern "*MOTOR_ID_FRONT*" (find-package "HUNTER_MSGS::HUNTERSTATUS"))
(shadow '*MOTOR_ID_FRONT* (find-package "HUNTER_MSGS::HUNTERSTATUS"))
(defconstant hunter_msgs::HunterStatus::*MOTOR_ID_FRONT* 0)
(intern "*MOTOR_ID_REAR_LEFT*" (find-package "HUNTER_MSGS::HUNTERSTATUS"))
(shadow '*MOTOR_ID_REAR_LEFT* (find-package "HUNTER_MSGS::HUNTERSTATUS"))
(defconstant hunter_msgs::HunterStatus::*MOTOR_ID_REAR_LEFT* 1)
(intern "*MOTOR_ID_REAR_RIGHT*" (find-package "HUNTER_MSGS::HUNTERSTATUS"))
(shadow '*MOTOR_ID_REAR_RIGHT* (find-package "HUNTER_MSGS::HUNTERSTATUS"))
(defconstant hunter_msgs::HunterStatus::*MOTOR_ID_REAR_RIGHT* 2)

(defun hunter_msgs::HunterStatus-to-symbol (const)
  (cond
        ((= const 0) 'hunter_msgs::HunterStatus::*MOTOR_ID_FRONT*)
        ((= const 1) 'hunter_msgs::HunterStatus::*MOTOR_ID_REAR_LEFT*)
        ((= const 2) 'hunter_msgs::HunterStatus::*MOTOR_ID_REAR_RIGHT*)
        (t nil)))

(defclass hunter_msgs::HunterStatus
  :super ros::object
  :slots (_header _linear_velocity _steering_angle _base_state _control_mode _fault_code _battery_voltage _park_mode _motor_states _driver_states ))

(defmethod hunter_msgs::HunterStatus
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:linear_velocity __linear_velocity) 0.0)
    ((:steering_angle __steering_angle) 0.0)
    ((:base_state __base_state) 0)
    ((:control_mode __control_mode) 0)
    ((:fault_code __fault_code) 0)
    ((:battery_voltage __battery_voltage) 0.0)
    ((:park_mode __park_mode) 0)
    ((:motor_states __motor_states) (let (r) (dotimes (i 3) (push (instance hunter_msgs::HunterMotorState :init) r)) r))
    ((:driver_states __driver_states) (let (r) (dotimes (i 3) (push (instance hunter_msgs::HunterDriverState :init) r)) r))
    )
   (send-super :init)
   (setq _header __header)
   (setq _linear_velocity (float __linear_velocity))
   (setq _steering_angle (float __steering_angle))
   (setq _base_state (round __base_state))
   (setq _control_mode (round __control_mode))
   (setq _fault_code (round __fault_code))
   (setq _battery_voltage (float __battery_voltage))
   (setq _park_mode (round __park_mode))
   (setq _motor_states __motor_states)
   (setq _driver_states __driver_states)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:linear_velocity
   (&optional __linear_velocity)
   (if __linear_velocity (setq _linear_velocity __linear_velocity)) _linear_velocity)
  (:steering_angle
   (&optional __steering_angle)
   (if __steering_angle (setq _steering_angle __steering_angle)) _steering_angle)
  (:base_state
   (&optional __base_state)
   (if __base_state (setq _base_state __base_state)) _base_state)
  (:control_mode
   (&optional __control_mode)
   (if __control_mode (setq _control_mode __control_mode)) _control_mode)
  (:fault_code
   (&optional __fault_code)
   (if __fault_code (setq _fault_code __fault_code)) _fault_code)
  (:battery_voltage
   (&optional __battery_voltage)
   (if __battery_voltage (setq _battery_voltage __battery_voltage)) _battery_voltage)
  (:park_mode
   (&optional __park_mode)
   (if __park_mode (setq _park_mode __park_mode)) _park_mode)
  (:motor_states
   (&rest __motor_states)
   (if (keywordp (car __motor_states))
       (send* _motor_states __motor_states)
     (progn
       (if __motor_states (setq _motor_states (car __motor_states)))
       _motor_states)))
  (:driver_states
   (&rest __driver_states)
   (if (keywordp (car __driver_states))
       (send* _driver_states __driver_states)
     (progn
       (if __driver_states (setq _driver_states (car __driver_states)))
       _driver_states)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float64 _linear_velocity
    8
    ;; float64 _steering_angle
    8
    ;; uint8 _base_state
    1
    ;; uint8 _control_mode
    1
    ;; uint16 _fault_code
    2
    ;; float64 _battery_voltage
    8
    ;; uint8 _park_mode
    1
    ;; hunter_msgs/HunterMotorState[3] _motor_states
    (apply #'+ (send-all _motor_states :serialization-length))
    ;; hunter_msgs/HunterDriverState[3] _driver_states
    (apply #'+ (send-all _driver_states :serialization-length))
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float64 _linear_velocity
       (sys::poke _linear_velocity (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _steering_angle
       (sys::poke _steering_angle (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; uint8 _base_state
       (write-byte _base_state s)
     ;; uint8 _control_mode
       (write-byte _control_mode s)
     ;; uint16 _fault_code
       (write-word _fault_code s)
     ;; float64 _battery_voltage
       (sys::poke _battery_voltage (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; uint8 _park_mode
       (write-byte _park_mode s)
     ;; hunter_msgs/HunterMotorState[3] _motor_states
     (dolist (elem _motor_states)
       (send elem :serialize s)
       )
     ;; hunter_msgs/HunterDriverState[3] _driver_states
     (dolist (elem _driver_states)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float64 _linear_velocity
     (setq _linear_velocity (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _steering_angle
     (setq _steering_angle (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; uint8 _base_state
     (setq _base_state (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _control_mode
     (setq _control_mode (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint16 _fault_code
     (setq _fault_code (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; float64 _battery_voltage
     (setq _battery_voltage (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; uint8 _park_mode
     (setq _park_mode (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; hunter_msgs/HunterMotorState[3] _motor_states
   (dotimes (i 3)
     (send (elt _motor_states i) :deserialize buf ptr-) (incf ptr- (send (elt _motor_states i) :serialization-length))
     )
   ;; hunter_msgs/HunterDriverState[3] _driver_states
   (dotimes (i 3)
     (send (elt _driver_states i) :deserialize buf ptr-) (incf ptr- (send (elt _driver_states i) :serialization-length))
     )
   ;;
   self)
  )

(setf (get hunter_msgs::HunterStatus :md5sum-) "ca557f5b13e1dbb8e7caa7cc072ca03c")
(setf (get hunter_msgs::HunterStatus :datatype-) "hunter_msgs/HunterStatus")
(setf (get hunter_msgs::HunterStatus :definition-)
      "Header header

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

")



(provide :hunter_msgs/HunterStatus "ca557f5b13e1dbb8e7caa7cc072ca03c")


