
(cl:in-package :asdf)

(defsystem "EposManager-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "MotorInfo" :depends-on ("_package_MotorInfo"))
    (:file "_package_MotorInfo" :depends-on ("_package"))
    (:file "EPOSControl" :depends-on ("_package_EPOSControl"))
    (:file "_package_EPOSControl" :depends-on ("_package"))
    (:file "GroupMotorInfo" :depends-on ("_package_GroupMotorInfo"))
    (:file "_package_GroupMotorInfo" :depends-on ("_package"))
    (:file "GroupEPOSControl" :depends-on ("_package_GroupEPOSControl"))
    (:file "_package_GroupEPOSControl" :depends-on ("_package"))
  ))