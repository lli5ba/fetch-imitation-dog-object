
(cl:in-package :asdf)

(defsystem "pan_tilt_control_pkg-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "centroid" :depends-on ("_package_centroid"))
    (:file "_package_centroid" :depends-on ("_package"))
  ))