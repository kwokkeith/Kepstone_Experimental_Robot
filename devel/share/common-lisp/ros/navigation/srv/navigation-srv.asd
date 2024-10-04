
(cl:in-package :asdf)

(defsystem "navigation-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "GetAmclPose" :depends-on ("_package_GetAmclPose"))
    (:file "_package_GetAmclPose" :depends-on ("_package"))
    (:file "GetPixelPose" :depends-on ("_package_GetPixelPose"))
    (:file "_package_GetPixelPose" :depends-on ("_package"))
  ))