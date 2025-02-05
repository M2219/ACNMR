;; Auto-generated. Do not edit!


(when (boundp 'hunter_msgs::HunterMotorState)
  (if (not (find-package "HUNTER_MSGS"))
    (make-package "HUNTER_MSGS"))
  (shadow 'HunterMotorState (find-package "HUNTER_MSGS")))
(unless (find-package "HUNTER_MSGS::HUNTERMOTORSTATE")
  (make-package "HUNTER_MSGS::HUNTERMOTORSTATE"))

(in-package "ROS")
;;//! \htmlinclude HunterMotorState.msg.html


(defclass hunter_msgs::HunterMotorState
  :super ros::object
  :slots (_current _rpm _temperature _motor_pose ))

(defmethod hunter_msgs::HunterMotorState
  (:init
   (&key
    ((:current __current) 0.0)
    ((:rpm __rpm) 0.0)
    ((:temperature __temperature) 0.0)
    ((:motor_pose __motor_pose) 0.0)
    )
   (send-super :init)
   (setq _current (float __current))
   (setq _rpm (float __rpm))
   (setq _temperature (float __temperature))
   (setq _motor_pose (float __motor_pose))
   self)
  (:current
   (&optional __current)
   (if __current (setq _current __current)) _current)
  (:rpm
   (&optional __rpm)
   (if __rpm (setq _rpm __rpm)) _rpm)
  (:temperature
   (&optional __temperature)
   (if __temperature (setq _temperature __temperature)) _temperature)
  (:motor_pose
   (&optional __motor_pose)
   (if __motor_pose (setq _motor_pose __motor_pose)) _motor_pose)
  (:serialization-length
   ()
   (+
    ;; float64 _current
    8
    ;; float64 _rpm
    8
    ;; float64 _temperature
    8
    ;; float64 _motor_pose
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _current
       (sys::poke _current (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _rpm
       (sys::poke _rpm (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _temperature
       (sys::poke _temperature (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _motor_pose
       (sys::poke _motor_pose (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _current
     (setq _current (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _rpm
     (setq _rpm (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _temperature
     (setq _temperature (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _motor_pose
     (setq _motor_pose (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get hunter_msgs::HunterMotorState :md5sum-) "6d4fa3048ea5c7fdf6db5094fce69da0")
(setf (get hunter_msgs::HunterMotorState :datatype-) "hunter_msgs/HunterMotorState")
(setf (get hunter_msgs::HunterMotorState :definition-)
      "float64 current
float64 rpm
float64 temperature
float64 motor_pose

")



(provide :hunter_msgs/HunterMotorState "6d4fa3048ea5c7fdf6db5094fce69da0")


