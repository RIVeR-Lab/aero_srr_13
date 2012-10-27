
(cl:in-package :asdf)

(defsystem "oryxsrr_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :sensor_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "OccupancyGrid" :depends-on ("_package_OccupancyGrid"))
    (:file "_package_OccupancyGrid" :depends-on ("_package"))
    (:file "SoftwareStop" :depends-on ("_package_SoftwareStop"))
    (:file "_package_SoftwareStop" :depends-on ("_package"))
  ))