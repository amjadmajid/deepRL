;; Auto-generated. Do not edit!


(when (boundp 'darknet_ros_msgs::ComputeBox)
  (if (not (find-package "DARKNET_ROS_MSGS"))
    (make-package "DARKNET_ROS_MSGS"))
  (shadow 'ComputeBox (find-package "DARKNET_ROS_MSGS")))
(unless (find-package "DARKNET_ROS_MSGS::COMPUTEBOX")
  (make-package "DARKNET_ROS_MSGS::COMPUTEBOX"))

(in-package "ROS")
;;//! \htmlinclude ComputeBox.msg.html


(defclass darknet_ros_msgs::ComputeBox
  :super ros::object
  :slots (_id _Class _probability _xmin _ymin _xmax _ymax ))

(defmethod darknet_ros_msgs::ComputeBox
  (:init
   (&key
    ((:id __id) 0)
    ((:Class __Class) "")
    ((:probability __probability) 0.0)
    ((:xmin __xmin) 0.0)
    ((:ymin __ymin) 0.0)
    ((:xmax __xmax) 0.0)
    ((:ymax __ymax) 0.0)
    )
   (send-super :init)
   (setq _id (round __id))
   (setq _Class (string __Class))
   (setq _probability (float __probability))
   (setq _xmin (float __xmin))
   (setq _ymin (float __ymin))
   (setq _xmax (float __xmax))
   (setq _ymax (float __ymax))
   self)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:Class
   (&optional __Class)
   (if __Class (setq _Class __Class)) _Class)
  (:probability
   (&optional __probability)
   (if __probability (setq _probability __probability)) _probability)
  (:xmin
   (&optional __xmin)
   (if __xmin (setq _xmin __xmin)) _xmin)
  (:ymin
   (&optional __ymin)
   (if __ymin (setq _ymin __ymin)) _ymin)
  (:xmax
   (&optional __xmax)
   (if __xmax (setq _xmax __xmax)) _xmax)
  (:ymax
   (&optional __ymax)
   (if __ymax (setq _ymax __ymax)) _ymax)
  (:serialization-length
   ()
   (+
    ;; int16 _id
    2
    ;; string _Class
    4 (length _Class)
    ;; float64 _probability
    8
    ;; float64 _xmin
    8
    ;; float64 _ymin
    8
    ;; float64 _xmax
    8
    ;; float64 _ymax
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int16 _id
       (write-word _id s)
     ;; string _Class
       (write-long (length _Class) s) (princ _Class s)
     ;; float64 _probability
       (sys::poke _probability (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _xmin
       (sys::poke _xmin (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _ymin
       (sys::poke _ymin (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _xmax
       (sys::poke _xmax (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _ymax
       (sys::poke _ymax (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int16 _id
     (setq _id (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; string _Class
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _Class (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float64 _probability
     (setq _probability (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _xmin
     (setq _xmin (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _ymin
     (setq _ymin (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _xmax
     (setq _xmax (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _ymax
     (setq _ymax (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get darknet_ros_msgs::ComputeBox :md5sum-) "7173ddff0b758de1103e5df884a8f8b9")
(setf (get darknet_ros_msgs::ComputeBox :datatype-) "darknet_ros_msgs/ComputeBox")
(setf (get darknet_ros_msgs::ComputeBox :definition-)
      "int16 id
string Class
float64 probability
float64 xmin
float64 ymin
float64 xmax
float64 ymax

")



(provide :darknet_ros_msgs/ComputeBox "7173ddff0b758de1103e5df884a8f8b9")


