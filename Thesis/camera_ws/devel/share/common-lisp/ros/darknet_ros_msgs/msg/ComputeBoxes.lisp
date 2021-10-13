; Auto-generated. Do not edit!


(cl:in-package darknet_ros_msgs-msg)


;//! \htmlinclude ComputeBoxes.msg.html

(cl:defclass <ComputeBoxes> (roslisp-msg-protocol:ros-message)
  ((compute_box
    :reader compute_box
    :initarg :compute_box
    :type (cl:vector darknet_ros_msgs-msg:ComputeBox)
   :initform (cl:make-array 0 :element-type 'darknet_ros_msgs-msg:ComputeBox :initial-element (cl:make-instance 'darknet_ros_msgs-msg:ComputeBox))))
)

(cl:defclass ComputeBoxes (<ComputeBoxes>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ComputeBoxes>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ComputeBoxes)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name darknet_ros_msgs-msg:<ComputeBoxes> is deprecated: use darknet_ros_msgs-msg:ComputeBoxes instead.")))

(cl:ensure-generic-function 'compute_box-val :lambda-list '(m))
(cl:defmethod compute_box-val ((m <ComputeBoxes>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader darknet_ros_msgs-msg:compute_box-val is deprecated.  Use darknet_ros_msgs-msg:compute_box instead.")
  (compute_box m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ComputeBoxes>) ostream)
  "Serializes a message object of type '<ComputeBoxes>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'compute_box))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'compute_box))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ComputeBoxes>) istream)
  "Deserializes a message object of type '<ComputeBoxes>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'compute_box) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'compute_box)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'darknet_ros_msgs-msg:ComputeBox))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ComputeBoxes>)))
  "Returns string type for a message object of type '<ComputeBoxes>"
  "darknet_ros_msgs/ComputeBoxes")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ComputeBoxes)))
  "Returns string type for a message object of type 'ComputeBoxes"
  "darknet_ros_msgs/ComputeBoxes")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ComputeBoxes>)))
  "Returns md5sum for a message object of type '<ComputeBoxes>"
  "1a0189e0ccd8b7029250c17e4ab7ef8f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ComputeBoxes)))
  "Returns md5sum for a message object of type 'ComputeBoxes"
  "1a0189e0ccd8b7029250c17e4ab7ef8f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ComputeBoxes>)))
  "Returns full string definition for message of type '<ComputeBoxes>"
  (cl:format cl:nil "ComputeBox[] compute_box~%================================================================================~%MSG: darknet_ros_msgs/ComputeBox~%int16 id~%string Class~%float64 probability~%float64 xmin~%float64 ymin~%float64 xmax~%float64 ymax~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ComputeBoxes)))
  "Returns full string definition for message of type 'ComputeBoxes"
  (cl:format cl:nil "ComputeBox[] compute_box~%================================================================================~%MSG: darknet_ros_msgs/ComputeBox~%int16 id~%string Class~%float64 probability~%float64 xmin~%float64 ymin~%float64 xmax~%float64 ymax~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ComputeBoxes>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'compute_box) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ComputeBoxes>))
  "Converts a ROS message object to a list"
  (cl:list 'ComputeBoxes
    (cl:cons ':compute_box (compute_box msg))
))
