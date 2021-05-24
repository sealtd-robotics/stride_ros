;; Auto-generated. Do not edit!


(when (boundp 'rosapi::TopicsAndRawTypes)
  (if (not (find-package "ROSAPI"))
    (make-package "ROSAPI"))
  (shadow 'TopicsAndRawTypes (find-package "ROSAPI")))
(unless (find-package "ROSAPI::TOPICSANDRAWTYPES")
  (make-package "ROSAPI::TOPICSANDRAWTYPES"))
(unless (find-package "ROSAPI::TOPICSANDRAWTYPESREQUEST")
  (make-package "ROSAPI::TOPICSANDRAWTYPESREQUEST"))
(unless (find-package "ROSAPI::TOPICSANDRAWTYPESRESPONSE")
  (make-package "ROSAPI::TOPICSANDRAWTYPESRESPONSE"))

(in-package "ROS")





(defclass rosapi::TopicsAndRawTypesRequest
  :super ros::object
  :slots ())

(defmethod rosapi::TopicsAndRawTypesRequest
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass rosapi::TopicsAndRawTypesResponse
  :super ros::object
  :slots (_topics _types _typedefs_full_text ))

(defmethod rosapi::TopicsAndRawTypesResponse
  (:init
   (&key
    ((:topics __topics) (let (r) (dotimes (i 0) (push "" r)) r))
    ((:types __types) (let (r) (dotimes (i 0) (push "" r)) r))
    ((:typedefs_full_text __typedefs_full_text) (let (r) (dotimes (i 0) (push "" r)) r))
    )
   (send-super :init)
   (setq _topics __topics)
   (setq _types __types)
   (setq _typedefs_full_text __typedefs_full_text)
   self)
  (:topics
   (&optional __topics)
   (if __topics (setq _topics __topics)) _topics)
  (:types
   (&optional __types)
   (if __types (setq _types __types)) _types)
  (:typedefs_full_text
   (&optional __typedefs_full_text)
   (if __typedefs_full_text (setq _typedefs_full_text __typedefs_full_text)) _typedefs_full_text)
  (:serialization-length
   ()
   (+
    ;; string[] _topics
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _topics)) 4
    ;; string[] _types
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _types)) 4
    ;; string[] _typedefs_full_text
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _typedefs_full_text)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string[] _topics
     (write-long (length _topics) s)
     (dolist (elem _topics)
       (write-long (length elem) s) (princ elem s)
       )
     ;; string[] _types
     (write-long (length _types) s)
     (dolist (elem _types)
       (write-long (length elem) s) (princ elem s)
       )
     ;; string[] _typedefs_full_text
     (write-long (length _typedefs_full_text) s)
     (dolist (elem _typedefs_full_text)
       (write-long (length elem) s) (princ elem s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string[] _topics
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _topics (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _topics i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;; string[] _types
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _types (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _types i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;; string[] _typedefs_full_text
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _typedefs_full_text (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _typedefs_full_text i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;;
   self)
  )

(defclass rosapi::TopicsAndRawTypes
  :super ros::object
  :slots ())

(setf (get rosapi::TopicsAndRawTypes :md5sum-) "e1432466c8f64316723276ba07c59d12")
(setf (get rosapi::TopicsAndRawTypes :datatype-) "rosapi/TopicsAndRawTypes")
(setf (get rosapi::TopicsAndRawTypes :request) rosapi::TopicsAndRawTypesRequest)
(setf (get rosapi::TopicsAndRawTypes :response) rosapi::TopicsAndRawTypesResponse)

(defmethod rosapi::TopicsAndRawTypesRequest
  (:response () (instance rosapi::TopicsAndRawTypesResponse :init)))

(setf (get rosapi::TopicsAndRawTypesRequest :md5sum-) "e1432466c8f64316723276ba07c59d12")
(setf (get rosapi::TopicsAndRawTypesRequest :datatype-) "rosapi/TopicsAndRawTypesRequest")
(setf (get rosapi::TopicsAndRawTypesRequest :definition-)
      "
---
string[] topics
string[] types
string[] typedefs_full_text

")

(setf (get rosapi::TopicsAndRawTypesResponse :md5sum-) "e1432466c8f64316723276ba07c59d12")
(setf (get rosapi::TopicsAndRawTypesResponse :datatype-) "rosapi/TopicsAndRawTypesResponse")
(setf (get rosapi::TopicsAndRawTypesResponse :definition-)
      "
---
string[] topics
string[] types
string[] typedefs_full_text

")



(provide :rosapi/TopicsAndRawTypes "e1432466c8f64316723276ba07c59d12")


