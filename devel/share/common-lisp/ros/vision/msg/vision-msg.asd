
(cl:in-package :asdf)

(defsystem "vision-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "SawyerCog" :depends-on ("_package_SawyerCog"))
    (:file "_package_SawyerCog" :depends-on ("_package"))
    (:file "VisualData" :depends-on ("_package_VisualData"))
    (:file "_package_VisualData" :depends-on ("_package"))
  ))