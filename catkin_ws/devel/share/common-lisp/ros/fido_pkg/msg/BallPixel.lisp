; Auto-generated. Do not edit!


(cl:in-package fido_pkg-msg)


;//! \htmlinclude BallPixel.msg.html

(cl:defclass <BallPixel> (roslisp-msg-protocol:ros-message)
  ((x
    :reader x
    :initarg :x
    :type cl:fixnum
    :initform 0)
   (y
    :reader y
    :initarg :y
    :type cl:fixnum
    :initform 0)
   (ballChange
    :reader ballChange
    :initarg :ballChange
    :type cl:boolean
    :initform cl:nil)
   (ballLost
    :reader ballLost
    :initarg :ballLost
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass BallPixel (<BallPixel>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <BallPixel>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'BallPixel)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name fido_pkg-msg:<BallPixel> is deprecated: use fido_pkg-msg:BallPixel instead.")))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <BallPixel>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fido_pkg-msg:x-val is deprecated.  Use fido_pkg-msg:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <BallPixel>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fido_pkg-msg:y-val is deprecated.  Use fido_pkg-msg:y instead.")
  (y m))

(cl:ensure-generic-function 'ballChange-val :lambda-list '(m))
(cl:defmethod ballChange-val ((m <BallPixel>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fido_pkg-msg:ballChange-val is deprecated.  Use fido_pkg-msg:ballChange instead.")
  (ballChange m))

(cl:ensure-generic-function 'ballLost-val :lambda-list '(m))
(cl:defmethod ballLost-val ((m <BallPixel>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fido_pkg-msg:ballLost-val is deprecated.  Use fido_pkg-msg:ballLost instead.")
  (ballLost m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <BallPixel>) ostream)
  "Serializes a message object of type '<BallPixel>"
  (cl:let* ((signed (cl:slot-value msg 'x)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'y)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ballChange) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ballLost) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <BallPixel>) istream)
  "Deserializes a message object of type '<BallPixel>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'x) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'y) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:setf (cl:slot-value msg 'ballChange) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'ballLost) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<BallPixel>)))
  "Returns string type for a message object of type '<BallPixel>"
  "fido_pkg/BallPixel")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'BallPixel)))
  "Returns string type for a message object of type 'BallPixel"
  "fido_pkg/BallPixel")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<BallPixel>)))
  "Returns md5sum for a message object of type '<BallPixel>"
  "ff21d8d44d40abdb39ee65a3a54bf55b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'BallPixel)))
  "Returns md5sum for a message object of type 'BallPixel"
  "ff21d8d44d40abdb39ee65a3a54bf55b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<BallPixel>)))
  "Returns full string definition for message of type '<BallPixel>"
  (cl:format cl:nil "int16 x~%int16 y~%bool ballChange  # True if these coordinates are for a different ball than the last message so can clear out memory for computing velocity~%bool ballLost    # True if no target was found in the image~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'BallPixel)))
  "Returns full string definition for message of type 'BallPixel"
  (cl:format cl:nil "int16 x~%int16 y~%bool ballChange  # True if these coordinates are for a different ball than the last message so can clear out memory for computing velocity~%bool ballLost    # True if no target was found in the image~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <BallPixel>))
  (cl:+ 0
     2
     2
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <BallPixel>))
  "Converts a ROS message object to a list"
  (cl:list 'BallPixel
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
    (cl:cons ':ballChange (ballChange msg))
    (cl:cons ':ballLost (ballLost msg))
))
