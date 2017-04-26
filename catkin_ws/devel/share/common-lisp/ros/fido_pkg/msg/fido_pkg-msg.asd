
(cl:in-package :asdf)

(defsystem "fido_pkg-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "BallPixel" :depends-on ("_package_BallPixel"))
    (:file "_package_BallPixel" :depends-on ("_package"))
    (:file "PanTilt" :depends-on ("_package_PanTilt"))
    (:file "_package_PanTilt" :depends-on ("_package"))
  ))