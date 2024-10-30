
(cl:in-package :asdf)

(defsystem "navigation-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "ConvertPixelToMap" :depends-on ("_package_ConvertPixelToMap"))
    (:file "_package_ConvertPixelToMap" :depends-on ("_package"))
    (:file "ConvertPixelWaypointsToMap" :depends-on ("_package_ConvertPixelWaypointsToMap"))
    (:file "_package_ConvertPixelWaypointsToMap" :depends-on ("_package"))
    (:file "GetAmclPose" :depends-on ("_package_GetAmclPose"))
    (:file "_package_GetAmclPose" :depends-on ("_package"))
    (:file "GetPixelPose" :depends-on ("_package_GetPixelPose"))
    (:file "_package_GetPixelPose" :depends-on ("_package"))
    (:file "GetWaypoints" :depends-on ("_package_GetWaypoints"))
    (:file "_package_GetWaypoints" :depends-on ("_package"))
    (:file "InitiateCoveragePath" :depends-on ("_package_InitiateCoveragePath"))
    (:file "_package_InitiateCoveragePath" :depends-on ("_package"))
  ))