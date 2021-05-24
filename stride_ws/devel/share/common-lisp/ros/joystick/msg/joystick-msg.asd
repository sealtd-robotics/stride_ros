
(cl:in-package :asdf)

(defsystem "joystick-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Stick" :depends-on ("_package_Stick"))
    (:file "_package_Stick" :depends-on ("_package"))
  ))