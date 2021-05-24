;; Auto-generated. Do not edit!


(when (boundp 'can_interface::WheelRPM)
  (if (not (find-package "CAN_INTERFACE"))
    (make-package "CAN_INTERFACE"))
  (shadow 'WheelRPM (find-package "CAN_INTERFACE")))
(unless (find-package "CAN_INTERFACE::WHEELRPM")
  (make-package "CAN_INTERFACE::WHEELRPM"))

(in-package "ROS")
;;//! \htmlinclude WheelRPM.msg.html


(defclass can_interface::WheelRPM
  :super ros::object
  :slots (_left_front _left_back _right_front _right_back ))

(defmethod can_interface::WheelRPM
  (:init
   (&key
    ((:left_front __left_front) 0.0)
    ((:left_back __left_back) 0.0)
    ((:right_front __right_front) 0.0)
    ((:right_back __right_back) 0.0)
    )
   (send-super :init)
   (setq _left_front (float __left_front))
   (setq _left_back (float __left_back))
   (setq _right_front (float __right_front))
   (setq _right_back (float __right_back))
   self)
  (:left_front
   (&optional __left_front)
   (if __left_front (setq _left_front __left_front)) _left_front)
  (:left_back
   (&optional __left_back)
   (if __left_back (setq _left_back __left_back)) _left_back)
  (:right_front
   (&optional __right_front)
   (if __right_front (setq _right_front __right_front)) _right_front)
  (:right_back
   (&optional __right_back)
   (if __right_back (setq _right_back __right_back)) _right_back)
  (:serialization-length
   ()
   (+
    ;; float32 _left_front
    4
    ;; float32 _left_back
    4
    ;; float32 _right_front
    4
    ;; float32 _right_back
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _left_front
       (sys::poke _left_front (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _left_back
       (sys::poke _left_back (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _right_front
       (sys::poke _right_front (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _right_back
       (sys::poke _right_back (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _left_front
     (setq _left_front (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _left_back
     (setq _left_back (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _right_front
     (setq _right_front (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _right_back
     (setq _right_back (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get can_interface::WheelRPM :md5sum-) "7409db75cdf11985ad54203bb29757ca")
(setf (get can_interface::WheelRPM :datatype-) "can_interface/WheelRPM")
(setf (get can_interface::WheelRPM :definition-)
      "float32 left_front
float32 left_back
float32 right_front
float32 right_back
")



(provide :can_interface/WheelRPM "7409db75cdf11985ad54203bb29757ca")


