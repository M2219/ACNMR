
(cl:in-package :asdf)

(defsystem "hunter_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "HunterBmsStatus" :depends-on ("_package_HunterBmsStatus"))
    (:file "_package_HunterBmsStatus" :depends-on ("_package"))
    (:file "HunterDriverState" :depends-on ("_package_HunterDriverState"))
    (:file "_package_HunterDriverState" :depends-on ("_package"))
    (:file "HunterMotorState" :depends-on ("_package_HunterMotorState"))
    (:file "_package_HunterMotorState" :depends-on ("_package"))
    (:file "HunterStatus" :depends-on ("_package_HunterStatus"))
    (:file "_package_HunterStatus" :depends-on ("_package"))
  ))