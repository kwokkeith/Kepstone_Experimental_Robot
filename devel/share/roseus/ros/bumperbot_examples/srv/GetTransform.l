;; Auto-generated. Do not edit!


(when (boundp 'bumperbot_examples::GetTransform)
  (if (not (find-package "BUMPERBOT_EXAMPLES"))
    (make-package "BUMPERBOT_EXAMPLES"))
  (shadow 'GetTransform (find-package "BUMPERBOT_EXAMPLES")))
(unless (find-package "BUMPERBOT_EXAMPLES::GETTRANSFORM")
  (make-package "BUMPERBOT_EXAMPLES::GETTRANSFORM"))
(unless (find-package "BUMPERBOT_EXAMPLES::GETTRANSFORMREQUEST")
  (make-package "BUMPERBOT_EXAMPLES::GETTRANSFORMREQUEST"))
(unless (find-package "BUMPERBOT_EXAMPLES::GETTRANSFORMRESPONSE")
  (make-package "BUMPERBOT_EXAMPLES::GETTRANSFORMRESPONSE"))

(in-package "ROS")



(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass bumperbot_examples::GetTransformRequest
  :super ros::object
  :slots (_frame_id _child_frame_id ))

(defmethod bumperbot_examples::GetTransformRequest
  (:init
   (&key
    ((:frame_id __frame_id) "")
    ((:child_frame_id __child_frame_id) "")
    )
   (send-super :init)
   (setq _frame_id (string __frame_id))
   (setq _child_frame_id (string __child_frame_id))
   self)
  (:frame_id
   (&optional __frame_id)
   (if __frame_id (setq _frame_id __frame_id)) _frame_id)
  (:child_frame_id
   (&optional __child_frame_id)
   (if __child_frame_id (setq _child_frame_id __child_frame_id)) _child_frame_id)
  (:serialization-length
   ()
   (+
    ;; string _frame_id
    4 (length _frame_id)
    ;; string _child_frame_id
    4 (length _child_frame_id)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _frame_id
       (write-long (length _frame_id) s) (princ _frame_id s)
     ;; string _child_frame_id
       (write-long (length _child_frame_id) s) (princ _child_frame_id s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _frame_id
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _frame_id (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _child_frame_id
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _child_frame_id (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass bumperbot_examples::GetTransformResponse
  :super ros::object
  :slots (_transform _success ))

(defmethod bumperbot_examples::GetTransformResponse
  (:init
   (&key
    ((:transform __transform) (instance geometry_msgs::TransformStamped :init))
    ((:success __success) nil)
    )
   (send-super :init)
   (setq _transform __transform)
   (setq _success __success)
   self)
  (:transform
   (&rest __transform)
   (if (keywordp (car __transform))
       (send* _transform __transform)
     (progn
       (if __transform (setq _transform (car __transform)))
       _transform)))
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/TransformStamped _transform
    (send _transform :serialization-length)
    ;; bool _success
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/TransformStamped _transform
       (send _transform :serialize s)
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/TransformStamped _transform
     (send _transform :deserialize buf ptr-) (incf ptr- (send _transform :serialization-length))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass bumperbot_examples::GetTransform
  :super ros::object
  :slots ())

(setf (get bumperbot_examples::GetTransform :md5sum-) "d44d0066649a82d7be9dc68623df55fc")
(setf (get bumperbot_examples::GetTransform :datatype-) "bumperbot_examples/GetTransform")
(setf (get bumperbot_examples::GetTransform :request) bumperbot_examples::GetTransformRequest)
(setf (get bumperbot_examples::GetTransform :response) bumperbot_examples::GetTransformResponse)

(defmethod bumperbot_examples::GetTransformRequest
  (:response () (instance bumperbot_examples::GetTransformResponse :init)))

(setf (get bumperbot_examples::GetTransformRequest :md5sum-) "d44d0066649a82d7be9dc68623df55fc")
(setf (get bumperbot_examples::GetTransformRequest :datatype-) "bumperbot_examples/GetTransformRequest")
(setf (get bumperbot_examples::GetTransformRequest :definition-)
      "#Request
string frame_id
string child_frame_id
---
#Response
geometry_msgs/TransformStamped transform
bool success

================================================================================
MSG: geometry_msgs/TransformStamped
# This expresses a transform from coordinate frame header.frame_id
# to the coordinate frame child_frame_id
#
# This message is mostly used by the 
# <a href=\"http://wiki.ros.org/tf\">tf</a> package. 
# See its documentation for more information.

Header header
string child_frame_id # the frame id of the child frame
Transform transform

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: geometry_msgs/Transform
# This represents the transform between two coordinate frames in free space.

Vector3 translation
Quaternion rotation

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
")

(setf (get bumperbot_examples::GetTransformResponse :md5sum-) "d44d0066649a82d7be9dc68623df55fc")
(setf (get bumperbot_examples::GetTransformResponse :datatype-) "bumperbot_examples/GetTransformResponse")
(setf (get bumperbot_examples::GetTransformResponse :definition-)
      "#Request
string frame_id
string child_frame_id
---
#Response
geometry_msgs/TransformStamped transform
bool success

================================================================================
MSG: geometry_msgs/TransformStamped
# This expresses a transform from coordinate frame header.frame_id
# to the coordinate frame child_frame_id
#
# This message is mostly used by the 
# <a href=\"http://wiki.ros.org/tf\">tf</a> package. 
# See its documentation for more information.

Header header
string child_frame_id # the frame id of the child frame
Transform transform

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: geometry_msgs/Transform
# This represents the transform between two coordinate frames in free space.

Vector3 translation
Quaternion rotation

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
")



(provide :bumperbot_examples/GetTransform "d44d0066649a82d7be9dc68623df55fc")


