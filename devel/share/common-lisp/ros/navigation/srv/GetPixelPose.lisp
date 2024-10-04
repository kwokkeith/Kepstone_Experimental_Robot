; Auto-generated. Do not edit!


(cl:in-package navigation-srv)


;//! \htmlinclude GetPixelPose-request.msg.html

(cl:defclass <GetPixelPose-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass GetPixelPose-request (<GetPixelPose-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetPixelPose-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetPixelPose-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name navigation-srv:<GetPixelPose-request> is deprecated: use navigation-srv:GetPixelPose-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetPixelPose-request>) ostream)
  "Serializes a message object of type '<GetPixelPose-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetPixelPose-request>) istream)
  "Deserializes a message object of type '<GetPixelPose-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetPixelPose-request>)))
  "Returns string type for a service object of type '<GetPixelPose-request>"
  "navigation/GetPixelPoseRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetPixelPose-request)))
  "Returns string type for a service object of type 'GetPixelPose-request"
  "navigation/GetPixelPoseRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetPixelPose-request>)))
  "Returns md5sum for a message object of type '<GetPixelPose-request>"
  "7c5941ad5d679949278959effa1051da")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetPixelPose-request)))
  "Returns md5sum for a message object of type 'GetPixelPose-request"
  "7c5941ad5d679949278959effa1051da")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetPixelPose-request>)))
  "Returns full string definition for message of type '<GetPixelPose-request>"
  (cl:format cl:nil "#Request~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetPixelPose-request)))
  "Returns full string definition for message of type 'GetPixelPose-request"
  (cl:format cl:nil "#Request~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetPixelPose-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetPixelPose-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetPixelPose-request
))
;//! \htmlinclude GetPixelPose-response.msg.html

(cl:defclass <GetPixelPose-response> (roslisp-msg-protocol:ros-message)
  ((pixel_pose
    :reader pixel_pose
    :initarg :pixel_pose
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point)))
)

(cl:defclass GetPixelPose-response (<GetPixelPose-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetPixelPose-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetPixelPose-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name navigation-srv:<GetPixelPose-response> is deprecated: use navigation-srv:GetPixelPose-response instead.")))

(cl:ensure-generic-function 'pixel_pose-val :lambda-list '(m))
(cl:defmethod pixel_pose-val ((m <GetPixelPose-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader navigation-srv:pixel_pose-val is deprecated.  Use navigation-srv:pixel_pose instead.")
  (pixel_pose m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetPixelPose-response>) ostream)
  "Serializes a message object of type '<GetPixelPose-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pixel_pose) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetPixelPose-response>) istream)
  "Deserializes a message object of type '<GetPixelPose-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pixel_pose) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetPixelPose-response>)))
  "Returns string type for a service object of type '<GetPixelPose-response>"
  "navigation/GetPixelPoseResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetPixelPose-response)))
  "Returns string type for a service object of type 'GetPixelPose-response"
  "navigation/GetPixelPoseResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetPixelPose-response>)))
  "Returns md5sum for a message object of type '<GetPixelPose-response>"
  "7c5941ad5d679949278959effa1051da")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetPixelPose-response)))
  "Returns md5sum for a message object of type 'GetPixelPose-response"
  "7c5941ad5d679949278959effa1051da")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetPixelPose-response>)))
  "Returns full string definition for message of type '<GetPixelPose-response>"
  (cl:format cl:nil "#Response~%geometry_msgs/Point pixel_pose~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetPixelPose-response)))
  "Returns full string definition for message of type 'GetPixelPose-response"
  (cl:format cl:nil "#Response~%geometry_msgs/Point pixel_pose~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetPixelPose-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pixel_pose))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetPixelPose-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetPixelPose-response
    (cl:cons ':pixel_pose (pixel_pose msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetPixelPose)))
  'GetPixelPose-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetPixelPose)))
  'GetPixelPose-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetPixelPose)))
  "Returns string type for a service object of type '<GetPixelPose>"
  "navigation/GetPixelPose")