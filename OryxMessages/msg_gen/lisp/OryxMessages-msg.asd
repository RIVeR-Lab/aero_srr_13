
(cl:in-package :asdf)

(defsystem "OryxMessages-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Temperature" :depends-on ("_package_Temperature"))
    (:file "_package_Temperature" :depends-on ("_package"))
    (:file "Battery" :depends-on ("_package_Battery"))
    (:file "_package_Battery" :depends-on ("_package"))
    (:file "BlobList" :depends-on ("_package_BlobList"))
    (:file "_package_BlobList" :depends-on ("_package"))
    (:file "Blob" :depends-on ("_package_Blob"))
    (:file "_package_Blob" :depends-on ("_package"))
  ))