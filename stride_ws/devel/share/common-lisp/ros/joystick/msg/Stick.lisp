; Auto-generated. Do not edit!


(cl:in-package joystick-msg)


;//! \htmlinclude Stick.msg.html

(cl:defclass <Stick> (roslisp-msg-protocol:ros-message)
  ((travel
    :reader travel
    :initarg :travel
    :type cl:float
    :initform 0.0)
   (angle
    :reader angle
    :initarg :angle
    :type cl:float
    :initform 0.0))
)

(cl:defclass Stick (<Stick>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Stick>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Stick)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name joystick-msg:<Stick> is deprecated: use joystick-msg:Stick instead.")))

(cl:ensure-generic-function 'travel-val :lambda-list '(m))
(cl:defmethod travel-val ((m <Stick>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader joystick-msg:travel-val is deprecated.  Use joystick-msg:travel instead.")
  (travel m))

(cl:ensure-generic-function 'angle-val :lambda-list '(m))
(cl:defmethod angle-val ((m <Stick>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader joystick-msg:angle-val is deprecated.  Use joystick-msg:angle instead.")
  (angle m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Stick>) ostream)
  "Serializes a message object of type '<Stick>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'travel))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'angle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Stick>) istream)
  "Deserializes a message object of type '<Stick>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'travel) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'angle) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Stick>)))
  "Returns string type for a message object of type '<Stick>"
  "joystick/Stick")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Stick)))
  "Returns string type for a message object of type 'Stick"
  "joystick/Stick")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Stick>)))
  "Returns md5sum for a message object of type '<Stick>"
  "4ac9ad86115642357f119d6100be459c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Stick)))
  "Returns md5sum for a message object of type 'Stick"
  "4ac9ad86115642357f119d6100be459c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Stick>)))
  "Returns full string definition for message of type '<Stick>"
  (cl:format cl:nil "float32 travel # between 0 and 1~%float32 angle  # radian~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Stick)))
  "Returns full string definition for message of type 'Stick"
  (cl:format cl:nil "float32 travel # between 0 and 1~%float32 angle  # radian~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Stick>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Stick>))
  "Converts a ROS message object to a list"
  (cl:list 'Stick
    (cl:cons ':travel (travel msg))
    (cl:cons ':angle (angle msg))
))
