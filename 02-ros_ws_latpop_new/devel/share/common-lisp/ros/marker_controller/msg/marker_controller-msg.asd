
(cl:in-package :asdf)

(defsystem "marker_controller-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "TargetPose" :depends-on ("_package_TargetPose"))
    (:file "_package_TargetPose" :depends-on ("_package"))
    (:file "TargetPoses" :depends-on ("_package_TargetPoses"))
    (:file "_package_TargetPoses" :depends-on ("_package"))
  ))