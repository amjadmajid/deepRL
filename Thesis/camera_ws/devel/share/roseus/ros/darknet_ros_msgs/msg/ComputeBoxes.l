;; Auto-generated. Do not edit!


(when (boundp 'darknet_ros_msgs::ComputeBoxes)
  (if (not (find-package "DARKNET_ROS_MSGS"))
    (make-package "DARKNET_ROS_MSGS"))
  (shadow 'ComputeBoxes (find-package "DARKNET_ROS_MSGS")))
(unless (find-package "DARKNET_ROS_MSGS::COMPUTEBOXES")
  (make-package "DARKNET_ROS_MSGS::COMPUTEBOXES"))

(in-package "ROS")
;;//! \htmlinclude ComputeBoxes.msg.html


(defclass darknet_ros_msgs::ComputeBoxes
  :super ros::object
  :slots (_compute_box ))

(defmethod darknet_ros_msgs::ComputeBoxes
  (:init
   (&key
    ((:compute_box __compute_box) (let (r) (dotimes (i 0) (push (instance darknet_ros_msgs::ComputeBox :init) r)) r))
    )
   (send-super :init)
   (setq _compute_box __compute_box)
   self)
  (:compute_box
   (&rest __compute_box)
   (if (keywordp (car __compute_box))
       (send* _compute_box __compute_box)
     (progn
       (if __compute_box (setq _compute_box (car __compute_box)))
       _compute_box)))
  (:serialization-length
   ()
   (+
    ;; darknet_ros_msgs/ComputeBox[] _compute_box
    (apply #'+ (send-all _compute_box :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; darknet_ros_msgs/ComputeBox[] _compute_box
     (write-long (length _compute_box) s)
     (dolist (elem _compute_box)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; darknet_ros_msgs/ComputeBox[] _compute_box
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _compute_box (let (r) (dotimes (i n) (push (instance darknet_ros_msgs::ComputeBox :init) r)) r))
     (dolist (elem- _compute_box)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get darknet_ros_msgs::ComputeBoxes :md5sum-) "1a0189e0ccd8b7029250c17e4ab7ef8f")
(setf (get darknet_ros_msgs::ComputeBoxes :datatype-) "darknet_ros_msgs/ComputeBoxes")
(setf (get darknet_ros_msgs::ComputeBoxes :definition-)
      "ComputeBox[] compute_box
================================================================================
MSG: darknet_ros_msgs/ComputeBox
int16 id
string Class
float64 probability
float64 xmin
float64 ymin
float64 xmax
float64 ymax

")



(provide :darknet_ros_msgs/ComputeBoxes "1a0189e0ccd8b7029250c17e4ab7ef8f")


