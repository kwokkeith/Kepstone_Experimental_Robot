;; Auto-generated. Do not edit!


(when (boundp 'navigation::GetPixelPose)
  (if (not (find-package "NAVIGATION"))
    (make-package "NAVIGATION"))
  (shadow 'GetPixelPose (find-package "NAVIGATION")))
(unless (find-package "NAVIGATION::GETPIXELPOSE")
  (make-package "NAVIGATION::GETPIXELPOSE"))
(unless (find-package "NAVIGATION::GETPIXELPOSEREQUEST")
  (make-package "NAVIGATION::GETPIXELPOSEREQUEST"))
(unless (find-package "NAVIGATION::GETPIXELPOSERESPONSE")
  (make-package "NAVIGATION::GETPIXELPOSERESPONSE"))

(in-package "ROS")



(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass navigation::GetPixelPoseRequest
  :super ros::object
  :slots ())

(defmethod navigation::GetPixelPoseRequest
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

(defclass navigation::GetPixelPoseResponse
  :super ros::object
  :slots (_pixel_pose ))

(defmethod navigation::GetPixelPoseResponse
  (:init
   (&key
    ((:pixel_pose __pixel_pose) (instance geometry_msgs::Point :init))
    )
   (send-super :init)
   (setq _pixel_pose __pixel_pose)
   self)
  (:pixel_pose
   (&rest __pixel_pose)
   (if (keywordp (car __pixel_pose))
       (send* _pixel_pose __pixel_pose)
     (progn
       (if __pixel_pose (setq _pixel_pose (car __pixel_pose)))
       _pixel_pose)))
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Point _pixel_pose
    (send _pixel_pose :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Point _pixel_pose
       (send _pixel_pose :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Point _pixel_pose
     (send _pixel_pose :deserialize buf ptr-) (incf ptr- (send _pixel_pose :serialization-length))
   ;;
   self)
  )

(defclass navigation::GetPixelPose
  :super ros::object
  :slots ())

(setf (get navigation::GetPixelPose :md5sum-) "7c5941ad5d679949278959effa1051da")
(setf (get navigation::GetPixelPose :datatype-) "navigation/GetPixelPose")
(setf (get navigation::GetPixelPose :request) navigation::GetPixelPoseRequest)
(setf (get navigation::GetPixelPose :response) navigation::GetPixelPoseResponse)

(defmethod navigation::GetPixelPoseRequest
  (:response () (instance navigation::GetPixelPoseResponse :init)))

(setf (get navigation::GetPixelPoseRequest :md5sum-) "7c5941ad5d679949278959effa1051da")
(setf (get navigation::GetPixelPoseRequest :datatype-) "navigation/GetPixelPoseRequest")
(setf (get navigation::GetPixelPoseRequest :definition-)
      "#Request
---
#Response
geometry_msgs/Point pixel_pose

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
")

(setf (get navigation::GetPixelPoseResponse :md5sum-) "7c5941ad5d679949278959effa1051da")
(setf (get navigation::GetPixelPoseResponse :datatype-) "navigation/GetPixelPoseResponse")
(setf (get navigation::GetPixelPoseResponse :definition-)
      "#Request
---
#Response
geometry_msgs/Point pixel_pose

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
")



(provide :navigation/GetPixelPose "7c5941ad5d679949278959effa1051da")


