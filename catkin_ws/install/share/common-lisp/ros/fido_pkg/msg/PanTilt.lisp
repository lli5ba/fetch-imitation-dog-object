; Auto-generated. Do not edit!


(cl:in-package fido_pkg-msg)


;//! \htmlinclude PanTilt.msg.html

(cl:defclass <PanTilt> (roslisp-msg-protocol:ros-message)
  ((pan
    :reader pan
    :initarg :pan
    :type cl:fixnum
    :initform 0)
   (tilt
    :reader tilt
    :initarg :tilt
    :type cl:fixnum
    :initform 0))
)

(cl:defclass PanTilt (<PanTilt>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PanTilt>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PanTilt)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name fido_pkg-msg:<PanTilt> is deprecated: use fido_pkg-msg:PanTilt instead.")))

(cl:ensure-generic-function 'pan-val :lambda-list '(m))
(cl:defmethod pan-val ((m <PanTilt>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fido_pkg-msg:pan-val is deprecated.  Use fido_pkg-msg:pan instead.")
  (pan m))

(cl:ensure-generic-function 'tilt-val :lambda-list '(m))
(cl:defmethod tilt-val ((m <PanTilt>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fido_pkg-msg:tilt-val is deprecated.  Use fido_pkg-msg:tilt instead.")
  (tilt m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PanTilt>) ostream)
  "Serializes a message object of type '<PanTilt>"
  (cl:let* ((signed (cl:slot-value msg 'pan)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'tilt)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PanTilt>) istream)
  "Deserializes a message object of type '<PanTilt>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'pan) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'tilt) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PanTilt>)))
  "Returns string type for a message object of type '<PanTilt>"
  "fido_pkg/PanTilt")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PanTilt)))
  "Returns string type for a message object of type 'PanTilt"
  "fido_pkg/PanTilt")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PanTilt>)))
  "Returns md5sum for a message object of type '<PanTilt>"
  "e5401181ff33a9d514daeb8a647b3152")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PanTilt)))
  "Returns md5sum for a message object of type 'PanTilt"
  "e5401181ff33a9d514daeb8a647b3152")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PanTilt>)))
  "Returns full string definition for message of type '<PanTilt>"
  (cl:format cl:nil "int16 pan~%int16 tilt ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PanTilt)))
  "Returns full string definition for message of type 'PanTilt"
  (cl:format cl:nil "int16 pan~%int16 tilt ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PanTilt>))
  (cl:+ 0
     2
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PanTilt>))
  "Converts a ROS message object to a list"
  (cl:list 'PanTilt
    (cl:cons ':pan (pan msg))
    (cl:cons ':tilt (tilt msg))
))
