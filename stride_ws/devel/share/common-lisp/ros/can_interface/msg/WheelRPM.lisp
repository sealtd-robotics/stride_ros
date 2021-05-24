; Auto-generated. Do not edit!


(cl:in-package can_interface-msg)


;//! \htmlinclude WheelRPM.msg.html

(cl:defclass <WheelRPM> (roslisp-msg-protocol:ros-message)
  ((left_front
    :reader left_front
    :initarg :left_front
    :type cl:float
    :initform 0.0)
   (left_back
    :reader left_back
    :initarg :left_back
    :type cl:float
    :initform 0.0)
   (right_front
    :reader right_front
    :initarg :right_front
    :type cl:float
    :initform 0.0)
   (right_back
    :reader right_back
    :initarg :right_back
    :type cl:float
    :initform 0.0))
)

(cl:defclass WheelRPM (<WheelRPM>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <WheelRPM>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'WheelRPM)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name can_interface-msg:<WheelRPM> is deprecated: use can_interface-msg:WheelRPM instead.")))

(cl:ensure-generic-function 'left_front-val :lambda-list '(m))
(cl:defmethod left_front-val ((m <WheelRPM>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader can_interface-msg:left_front-val is deprecated.  Use can_interface-msg:left_front instead.")
  (left_front m))

(cl:ensure-generic-function 'left_back-val :lambda-list '(m))
(cl:defmethod left_back-val ((m <WheelRPM>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader can_interface-msg:left_back-val is deprecated.  Use can_interface-msg:left_back instead.")
  (left_back m))

(cl:ensure-generic-function 'right_front-val :lambda-list '(m))
(cl:defmethod right_front-val ((m <WheelRPM>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader can_interface-msg:right_front-val is deprecated.  Use can_interface-msg:right_front instead.")
  (right_front m))

(cl:ensure-generic-function 'right_back-val :lambda-list '(m))
(cl:defmethod right_back-val ((m <WheelRPM>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader can_interface-msg:right_back-val is deprecated.  Use can_interface-msg:right_back instead.")
  (right_back m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <WheelRPM>) ostream)
  "Serializes a message object of type '<WheelRPM>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'left_front))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'left_back))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'right_front))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'right_back))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <WheelRPM>) istream)
  "Deserializes a message object of type '<WheelRPM>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'left_front) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'left_back) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'right_front) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'right_back) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<WheelRPM>)))
  "Returns string type for a message object of type '<WheelRPM>"
  "can_interface/WheelRPM")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'WheelRPM)))
  "Returns string type for a message object of type 'WheelRPM"
  "can_interface/WheelRPM")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<WheelRPM>)))
  "Returns md5sum for a message object of type '<WheelRPM>"
  "7409db75cdf11985ad54203bb29757ca")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'WheelRPM)))
  "Returns md5sum for a message object of type 'WheelRPM"
  "7409db75cdf11985ad54203bb29757ca")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<WheelRPM>)))
  "Returns full string definition for message of type '<WheelRPM>"
  (cl:format cl:nil "float32 left_front~%float32 left_back~%float32 right_front~%float32 right_back~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'WheelRPM)))
  "Returns full string definition for message of type 'WheelRPM"
  (cl:format cl:nil "float32 left_front~%float32 left_back~%float32 right_front~%float32 right_back~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <WheelRPM>))
  (cl:+ 0
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <WheelRPM>))
  "Converts a ROS message object to a list"
  (cl:list 'WheelRPM
    (cl:cons ':left_front (left_front msg))
    (cl:cons ':left_back (left_back msg))
    (cl:cons ':right_front (right_front msg))
    (cl:cons ':right_back (right_back msg))
))
