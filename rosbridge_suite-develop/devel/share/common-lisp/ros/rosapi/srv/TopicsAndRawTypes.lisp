; Auto-generated. Do not edit!


(cl:in-package rosapi-srv)


;//! \htmlinclude TopicsAndRawTypes-request.msg.html

(cl:defclass <TopicsAndRawTypes-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass TopicsAndRawTypes-request (<TopicsAndRawTypes-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TopicsAndRawTypes-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TopicsAndRawTypes-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rosapi-srv:<TopicsAndRawTypes-request> is deprecated: use rosapi-srv:TopicsAndRawTypes-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TopicsAndRawTypes-request>) ostream)
  "Serializes a message object of type '<TopicsAndRawTypes-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TopicsAndRawTypes-request>) istream)
  "Deserializes a message object of type '<TopicsAndRawTypes-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TopicsAndRawTypes-request>)))
  "Returns string type for a service object of type '<TopicsAndRawTypes-request>"
  "rosapi/TopicsAndRawTypesRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TopicsAndRawTypes-request)))
  "Returns string type for a service object of type 'TopicsAndRawTypes-request"
  "rosapi/TopicsAndRawTypesRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TopicsAndRawTypes-request>)))
  "Returns md5sum for a message object of type '<TopicsAndRawTypes-request>"
  "e1432466c8f64316723276ba07c59d12")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TopicsAndRawTypes-request)))
  "Returns md5sum for a message object of type 'TopicsAndRawTypes-request"
  "e1432466c8f64316723276ba07c59d12")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TopicsAndRawTypes-request>)))
  "Returns full string definition for message of type '<TopicsAndRawTypes-request>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TopicsAndRawTypes-request)))
  "Returns full string definition for message of type 'TopicsAndRawTypes-request"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TopicsAndRawTypes-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TopicsAndRawTypes-request>))
  "Converts a ROS message object to a list"
  (cl:list 'TopicsAndRawTypes-request
))
;//! \htmlinclude TopicsAndRawTypes-response.msg.html

(cl:defclass <TopicsAndRawTypes-response> (roslisp-msg-protocol:ros-message)
  ((topics
    :reader topics
    :initarg :topics
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element ""))
   (types
    :reader types
    :initarg :types
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element ""))
   (typedefs_full_text
    :reader typedefs_full_text
    :initarg :typedefs_full_text
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element "")))
)

(cl:defclass TopicsAndRawTypes-response (<TopicsAndRawTypes-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TopicsAndRawTypes-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TopicsAndRawTypes-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rosapi-srv:<TopicsAndRawTypes-response> is deprecated: use rosapi-srv:TopicsAndRawTypes-response instead.")))

(cl:ensure-generic-function 'topics-val :lambda-list '(m))
(cl:defmethod topics-val ((m <TopicsAndRawTypes-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rosapi-srv:topics-val is deprecated.  Use rosapi-srv:topics instead.")
  (topics m))

(cl:ensure-generic-function 'types-val :lambda-list '(m))
(cl:defmethod types-val ((m <TopicsAndRawTypes-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rosapi-srv:types-val is deprecated.  Use rosapi-srv:types instead.")
  (types m))

(cl:ensure-generic-function 'typedefs_full_text-val :lambda-list '(m))
(cl:defmethod typedefs_full_text-val ((m <TopicsAndRawTypes-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rosapi-srv:typedefs_full_text-val is deprecated.  Use rosapi-srv:typedefs_full_text instead.")
  (typedefs_full_text m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TopicsAndRawTypes-response>) ostream)
  "Serializes a message object of type '<TopicsAndRawTypes-response>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'topics))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((__ros_str_len (cl:length ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) ele))
   (cl:slot-value msg 'topics))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'types))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((__ros_str_len (cl:length ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) ele))
   (cl:slot-value msg 'types))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'typedefs_full_text))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((__ros_str_len (cl:length ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) ele))
   (cl:slot-value msg 'typedefs_full_text))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TopicsAndRawTypes-response>) istream)
  "Deserializes a message object of type '<TopicsAndRawTypes-response>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'topics) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'topics)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:aref vals i) __ros_str_idx) (cl:code-char (cl:read-byte istream))))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'types) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'types)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:aref vals i) __ros_str_idx) (cl:code-char (cl:read-byte istream))))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'typedefs_full_text) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'typedefs_full_text)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:aref vals i) __ros_str_idx) (cl:code-char (cl:read-byte istream))))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TopicsAndRawTypes-response>)))
  "Returns string type for a service object of type '<TopicsAndRawTypes-response>"
  "rosapi/TopicsAndRawTypesResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TopicsAndRawTypes-response)))
  "Returns string type for a service object of type 'TopicsAndRawTypes-response"
  "rosapi/TopicsAndRawTypesResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TopicsAndRawTypes-response>)))
  "Returns md5sum for a message object of type '<TopicsAndRawTypes-response>"
  "e1432466c8f64316723276ba07c59d12")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TopicsAndRawTypes-response)))
  "Returns md5sum for a message object of type 'TopicsAndRawTypes-response"
  "e1432466c8f64316723276ba07c59d12")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TopicsAndRawTypes-response>)))
  "Returns full string definition for message of type '<TopicsAndRawTypes-response>"
  (cl:format cl:nil "string[] topics~%string[] types~%string[] typedefs_full_text~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TopicsAndRawTypes-response)))
  "Returns full string definition for message of type 'TopicsAndRawTypes-response"
  (cl:format cl:nil "string[] topics~%string[] types~%string[] typedefs_full_text~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TopicsAndRawTypes-response>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'topics) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'types) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'typedefs_full_text) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TopicsAndRawTypes-response>))
  "Converts a ROS message object to a list"
  (cl:list 'TopicsAndRawTypes-response
    (cl:cons ':topics (topics msg))
    (cl:cons ':types (types msg))
    (cl:cons ':typedefs_full_text (typedefs_full_text msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'TopicsAndRawTypes)))
  'TopicsAndRawTypes-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'TopicsAndRawTypes)))
  'TopicsAndRawTypes-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TopicsAndRawTypes)))
  "Returns string type for a service object of type '<TopicsAndRawTypes>"
  "rosapi/TopicsAndRawTypes")