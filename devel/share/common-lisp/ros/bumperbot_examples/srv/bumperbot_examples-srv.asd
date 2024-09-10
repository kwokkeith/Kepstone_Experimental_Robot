
(cl:in-package :asdf)

(defsystem "bumperbot_examples-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "AddTwoInts" :depends-on ("_package_AddTwoInts"))
    (:file "_package_AddTwoInts" :depends-on ("_package"))
    (:file "GetTransform" :depends-on ("_package_GetTransform"))
    (:file "_package_GetTransform" :depends-on ("_package"))
  ))