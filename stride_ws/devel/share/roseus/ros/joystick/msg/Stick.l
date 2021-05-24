;; Auto-generated. Do not edit!


(when (boundp 'joystick::Stick)
  (if (not (find-package "JOYSTICK"))
    (make-package "JOYSTICK"))
  (shadow 'Stick (find-package "JOYSTICK")))
(unless (find-package "JOYSTICK::STICK")
  (make-package "JOYSTICK::STICK"))

(in-package "ROS")
;;//! \htmlinclude Stick.msg.html


(defclass joystick::Stick
  :super ros::object
  :slots (_travel _angle ))

(defmethod joystick::Stick
  (:init
   (&key
    ((:travel __travel) 0.0)
    ((:angle __angle) 0.0)
    )
   (send-super :init)
   (setq _travel (float __travel))
   (setq _angle (float __angle))
   self)
  (:travel
   (&optional __travel)
   (if __travel (setq _travel __travel)) _travel)
  (:angle
   (&optional __angle)
   (if __angle (setq _angle __angle)) _angle)
  (:serialization-length
   ()
   (+
    ;; float32 _travel
    4
    ;; float32 _angle
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _travel
       (sys::poke _travel (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _angle
       (sys::poke _angle (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _travel
     (setq _travel (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _angle
     (setq _angle (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get joystick::Stick :md5sum-) "4ac9ad86115642357f119d6100be459c")
(setf (get joystick::Stick :datatype-) "joystick/Stick")
(setf (get joystick::Stick :definition-)
      "float32 travel # between 0 and 1
float32 angle  # radian
")



(provide :joystick/Stick "4ac9ad86115642357f119d6100be459c")


