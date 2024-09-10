; Auto-generated. Do not edit!


(cl:in-package bumperbot_examples-srv)


;//! \htmlinclude GetTransform-request.msg.html

(cl:defclass <GetTransform-request> (roslisp-msg-protocol:ros-message)
  ((frame_id
    :reader frame_id
    :initarg :frame_id
    :type cl:string
    :initform "")
   (child_frame_id
    :reader child_frame_id
    :initarg :child_frame_id
    :type cl:string
    :initform ""))
)

(cl:defclass GetTransform-request (<GetTransform-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetTransform-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetTransform-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name bumperbot_examples-srv:<GetTransform-request> is deprecated: use bumperbot_examples-srv:GetTransform-request instead.")))

(cl:ensure-generic-function 'frame_id-val :lambda-list '(m))
(cl:defmethod frame_id-val ((m <GetTransform-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bumperbot_examples-srv:frame_id-val is deprecated.  Use bumperbot_examples-srv:frame_id instead.")
  (frame_id m))

(cl:ensure-generic-function 'child_frame_id-val :lambda-list '(m))
(cl:defmethod child_frame_id-val ((m <GetTransform-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bumperbot_examples-srv:child_frame_id-val is deprecated.  Use bumperbot_examples-srv:child_frame_id instead.")
  (child_frame_id m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetTransform-request>) ostream)
  "Serializes a message object of type '<GetTransform-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'frame_id))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'frame_id))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'child_frame_id))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'child_frame_id))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetTransform-request>) istream)
  "Deserializes a message object of type '<GetTransform-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'frame_id) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'frame_id) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'child_frame_id) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'child_frame_id) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetTransform-request>)))
  "Returns string type for a service object of type '<GetTransform-request>"
  "bumperbot_examples/GetTransformRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetTransform-request)))
  "Returns string type for a service object of type 'GetTransform-request"
  "bumperbot_examples/GetTransformRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetTransform-request>)))
  "Returns md5sum for a message object of type '<GetTransform-request>"
  "d44d0066649a82d7be9dc68623df55fc")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetTransform-request)))
  "Returns md5sum for a message object of type 'GetTransform-request"
  "d44d0066649a82d7be9dc68623df55fc")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetTransform-request>)))
  "Returns full string definition for message of type '<GetTransform-request>"
  (cl:format cl:nil "#Request~%string frame_id~%string child_frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetTransform-request)))
  "Returns full string definition for message of type 'GetTransform-request"
  (cl:format cl:nil "#Request~%string frame_id~%string child_frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetTransform-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'frame_id))
     4 (cl:length (cl:slot-value msg 'child_frame_id))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetTransform-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetTransform-request
    (cl:cons ':frame_id (frame_id msg))
    (cl:cons ':child_frame_id (child_frame_id msg))
))
;//! \htmlinclude GetTransform-response.msg.html

(cl:defclass <GetTransform-response> (roslisp-msg-protocol:ros-message)
  ((transform
    :reader transform
    :initarg :transform
    :type geometry_msgs-msg:TransformStamped
    :initform (cl:make-instance 'geometry_msgs-msg:TransformStamped))
   (success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass GetTransform-response (<GetTransform-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetTransform-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetTransform-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name bumperbot_examples-srv:<GetTransform-response> is deprecated: use bumperbot_examples-srv:GetTransform-response instead.")))

(cl:ensure-generic-function 'transform-val :lambda-list '(m))
(cl:defmethod transform-val ((m <GetTransform-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bumperbot_examples-srv:transform-val is deprecated.  Use bumperbot_examples-srv:transform instead.")
  (transform m))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <GetTransform-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bumperbot_examples-srv:success-val is deprecated.  Use bumperbot_examples-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetTransform-response>) ostream)
  "Serializes a message object of type '<GetTransform-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'transform) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetTransform-response>) istream)
  "Deserializes a message object of type '<GetTransform-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'transform) istream)
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetTransform-response>)))
  "Returns string type for a service object of type '<GetTransform-response>"
  "bumperbot_examples/GetTransformResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetTransform-response)))
  "Returns string type for a service object of type 'GetTransform-response"
  "bumperbot_examples/GetTransformResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetTransform-response>)))
  "Returns md5sum for a message object of type '<GetTransform-response>"
  "d44d0066649a82d7be9dc68623df55fc")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetTransform-response)))
  "Returns md5sum for a message object of type 'GetTransform-response"
  "d44d0066649a82d7be9dc68623df55fc")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetTransform-response>)))
  "Returns full string definition for message of type '<GetTransform-response>"
  (cl:format cl:nil "#Response~%geometry_msgs/TransformStamped transform~%bool success~%~%================================================================================~%MSG: geometry_msgs/TransformStamped~%# This expresses a transform from coordinate frame header.frame_id~%# to the coordinate frame child_frame_id~%#~%# This message is mostly used by the ~%# <a href=\"http://wiki.ros.org/tf\">tf</a> package. ~%# See its documentation for more information.~%~%Header header~%string child_frame_id # the frame id of the child frame~%Transform transform~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Transform~%# This represents the transform between two coordinate frames in free space.~%~%Vector3 translation~%Quaternion rotation~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetTransform-response)))
  "Returns full string definition for message of type 'GetTransform-response"
  (cl:format cl:nil "#Response~%geometry_msgs/TransformStamped transform~%bool success~%~%================================================================================~%MSG: geometry_msgs/TransformStamped~%# This expresses a transform from coordinate frame header.frame_id~%# to the coordinate frame child_frame_id~%#~%# This message is mostly used by the ~%# <a href=\"http://wiki.ros.org/tf\">tf</a> package. ~%# See its documentation for more information.~%~%Header header~%string child_frame_id # the frame id of the child frame~%Transform transform~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Transform~%# This represents the transform between two coordinate frames in free space.~%~%Vector3 translation~%Quaternion rotation~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetTransform-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'transform))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetTransform-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetTransform-response
    (cl:cons ':transform (transform msg))
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetTransform)))
  'GetTransform-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetTransform)))
  'GetTransform-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetTransform)))
  "Returns string type for a service object of type '<GetTransform>"
  "bumperbot_examples/GetTransform")