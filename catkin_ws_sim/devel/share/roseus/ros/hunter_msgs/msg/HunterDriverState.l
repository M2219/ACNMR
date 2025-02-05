;; Auto-generated. Do not edit!


(when (boundp 'hunter_msgs::HunterDriverState)
  (if (not (find-package "HUNTER_MSGS"))
    (make-package "HUNTER_MSGS"))
  (shadow 'HunterDriverState (find-package "HUNTER_MSGS")))
(unless (find-package "HUNTER_MSGS::HUNTERDRIVERSTATE")
  (make-package "HUNTER_MSGS::HUNTERDRIVERSTATE"))

(in-package "ROS")
;;//! \htmlinclude HunterDriverState.msg.html


(defclass hunter_msgs::HunterDriverState
  :super ros::object
  :slots (_driver_voltage _driver_temperature _driver_state ))

(defmethod hunter_msgs::HunterDriverState
  (:init
   (&key
    ((:driver_voltage __driver_voltage) 0.0)
    ((:driver_temperature __driver_temperature) 0.0)
    ((:driver_state __driver_state) 0)
    )
   (send-super :init)
   (setq _driver_voltage (float __driver_voltage))
   (setq _driver_temperature (float __driver_temperature))
   (setq _driver_state (round __driver_state))
   self)
  (:driver_voltage
   (&optional __driver_voltage)
   (if __driver_voltage (setq _driver_voltage __driver_voltage)) _driver_voltage)
  (:driver_temperature
   (&optional __driver_temperature)
   (if __driver_temperature (setq _driver_temperature __driver_temperature)) _driver_temperature)
  (:driver_state
   (&optional __driver_state)
   (if __driver_state (setq _driver_state __driver_state)) _driver_state)
  (:serialization-length
   ()
   (+
    ;; float64 _driver_voltage
    8
    ;; float64 _driver_temperature
    8
    ;; uint8 _driver_state
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _driver_voltage
       (sys::poke _driver_voltage (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _driver_temperature
       (sys::poke _driver_temperature (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; uint8 _driver_state
       (write-byte _driver_state s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _driver_voltage
     (setq _driver_voltage (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _driver_temperature
     (setq _driver_temperature (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; uint8 _driver_state
     (setq _driver_state (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get hunter_msgs::HunterDriverState :md5sum-) "bcb2bfdfab7869cfb37bcfdc3a86c1d4")
(setf (get hunter_msgs::HunterDriverState :datatype-) "hunter_msgs/HunterDriverState")
(setf (get hunter_msgs::HunterDriverState :definition-)
      "float64 driver_voltage
float64 driver_temperature
uint8   driver_state

")



(provide :hunter_msgs/HunterDriverState "bcb2bfdfab7869cfb37bcfdc3a86c1d4")


