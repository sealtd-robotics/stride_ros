
(cl:in-package :asdf)

(defsystem "can_interface-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "WheelRPM" :depends-on ("_package_WheelRPM"))
    (:file "_package_WheelRPM" :depends-on ("_package"))
  ))