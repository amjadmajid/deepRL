;; Auto-generated. Do not edit!


(when (boundp 'gb_visual_detection_3d_msgs::BoundingBox3d)
  (if (not (find-package "GB_VISUAL_DETECTION_3D_MSGS"))
    (make-package "GB_VISUAL_DETECTION_3D_MSGS"))
  (shadow 'BoundingBox3d (find-package "GB_VISUAL_DETECTION_3D_MSGS")))
(unless (find-package "GB_VISUAL_DETECTION_3D_MSGS::BOUNDINGBOX3D")
  (make-package "GB_VISUAL_DETECTION_3D_MSGS::BOUNDINGBOX3D"))

(in-package "ROS")
;;//! \htmlinclude BoundingBox3d.msg.html


(defclass gb_visual_detection_3d_msgs::BoundingBox3d
  :super ros::object
  :slots (_Class _probability _xmin _ymin _xmax _ymax _zmin _zmax _xmin_2d _ymin_2d _xmax_2d _ymax_2d ))

(defmethod gb_visual_detection_3d_msgs::BoundingBox3d
  (:init
   (&key
    ((:Class __Class) "")
    ((:probability __probability) 0.0)
    ((:xmin __xmin) 0.0)
    ((:ymin __ymin) 0.0)
    ((:xmax __xmax) 0.0)
    ((:ymax __ymax) 0.0)
    ((:zmin __zmin) 0.0)
    ((:zmax __zmax) 0.0)
    ((:xmin_2d __xmin_2d) 0)
    ((:ymin_2d __ymin_2d) 0)
    ((:xmax_2d __xmax_2d) 0)
    ((:ymax_2d __ymax_2d) 0)
    )
   (send-super :init)
   (setq _Class (string __Class))
   (setq _probability (float __probability))
   (setq _xmin (float __xmin))
   (setq _ymin (float __ymin))
   (setq _xmax (float __xmax))
   (setq _ymax (float __ymax))
   (setq _zmin (float __zmin))
   (setq _zmax (float __zmax))
   (setq _xmin_2d (round __xmin_2d))
   (setq _ymin_2d (round __ymin_2d))
   (setq _xmax_2d (round __xmax_2d))
   (setq _ymax_2d (round __ymax_2d))
   self)
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
  (:zmin
   (&optional __zmin)
   (if __zmin (setq _zmin __zmin)) _zmin)
  (:zmax
   (&optional __zmax)
   (if __zmax (setq _zmax __zmax)) _zmax)
  (:xmin_2d
   (&optional __xmin_2d)
   (if __xmin_2d (setq _xmin_2d __xmin_2d)) _xmin_2d)
  (:ymin_2d
   (&optional __ymin_2d)
   (if __ymin_2d (setq _ymin_2d __ymin_2d)) _ymin_2d)
  (:xmax_2d
   (&optional __xmax_2d)
   (if __xmax_2d (setq _xmax_2d __xmax_2d)) _xmax_2d)
  (:ymax_2d
   (&optional __ymax_2d)
   (if __ymax_2d (setq _ymax_2d __ymax_2d)) _ymax_2d)
  (:serialization-length
   ()
   (+
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
    ;; float64 _zmin
    8
    ;; float64 _zmax
    8
    ;; int64 _xmin_2d
    8
    ;; int64 _ymin_2d
    8
    ;; int64 _xmax_2d
    8
    ;; int64 _ymax_2d
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
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
     ;; float64 _zmin
       (sys::poke _zmin (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _zmax
       (sys::poke _zmax (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; int64 _xmin_2d
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _xmin_2d (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _xmin_2d) (= (length (_xmin_2d . bv)) 2)) ;; bignum
              (write-long (ash (elt (_xmin_2d . bv) 0) 0) s)
              (write-long (ash (elt (_xmin_2d . bv) 1) -1) s))
             ((and (class _xmin_2d) (= (length (_xmin_2d . bv)) 1)) ;; big1
              (write-long (elt (_xmin_2d . bv) 0) s)
              (write-long (if (>= _xmin_2d 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _xmin_2d s)(write-long (if (>= _xmin_2d 0) 0 #xffffffff) s)))
     ;; int64 _ymin_2d
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _ymin_2d (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _ymin_2d) (= (length (_ymin_2d . bv)) 2)) ;; bignum
              (write-long (ash (elt (_ymin_2d . bv) 0) 0) s)
              (write-long (ash (elt (_ymin_2d . bv) 1) -1) s))
             ((and (class _ymin_2d) (= (length (_ymin_2d . bv)) 1)) ;; big1
              (write-long (elt (_ymin_2d . bv) 0) s)
              (write-long (if (>= _ymin_2d 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _ymin_2d s)(write-long (if (>= _ymin_2d 0) 0 #xffffffff) s)))
     ;; int64 _xmax_2d
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _xmax_2d (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _xmax_2d) (= (length (_xmax_2d . bv)) 2)) ;; bignum
              (write-long (ash (elt (_xmax_2d . bv) 0) 0) s)
              (write-long (ash (elt (_xmax_2d . bv) 1) -1) s))
             ((and (class _xmax_2d) (= (length (_xmax_2d . bv)) 1)) ;; big1
              (write-long (elt (_xmax_2d . bv) 0) s)
              (write-long (if (>= _xmax_2d 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _xmax_2d s)(write-long (if (>= _xmax_2d 0) 0 #xffffffff) s)))
     ;; int64 _ymax_2d
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _ymax_2d (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _ymax_2d) (= (length (_ymax_2d . bv)) 2)) ;; bignum
              (write-long (ash (elt (_ymax_2d . bv) 0) 0) s)
              (write-long (ash (elt (_ymax_2d . bv) 1) -1) s))
             ((and (class _ymax_2d) (= (length (_ymax_2d . bv)) 1)) ;; big1
              (write-long (elt (_ymax_2d . bv) 0) s)
              (write-long (if (>= _ymax_2d 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _ymax_2d s)(write-long (if (>= _ymax_2d 0) 0 #xffffffff) s)))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
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
   ;; float64 _zmin
     (setq _zmin (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _zmax
     (setq _zmax (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; int64 _xmin_2d
#+(or :alpha :irix6 :x86_64)
      (setf _xmin_2d (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _xmin_2d (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int64 _ymin_2d
#+(or :alpha :irix6 :x86_64)
      (setf _ymin_2d (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _ymin_2d (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int64 _xmax_2d
#+(or :alpha :irix6 :x86_64)
      (setf _xmax_2d (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _xmax_2d (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int64 _ymax_2d
#+(or :alpha :irix6 :x86_64)
      (setf _ymax_2d (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _ymax_2d (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;;
   self)
  )

(setf (get gb_visual_detection_3d_msgs::BoundingBox3d :md5sum-) "2ce9b0738c577c56440a33d88e32daf4")
(setf (get gb_visual_detection_3d_msgs::BoundingBox3d :datatype-) "gb_visual_detection_3d_msgs/BoundingBox3d")
(setf (get gb_visual_detection_3d_msgs::BoundingBox3d :definition-)
      "string Class
float64 probability
float64 xmin
float64 ymin
float64 xmax
float64 ymax
float64 zmin
float64 zmax
int64 xmin_2d
int64 ymin_2d
int64 xmax_2d
int64 ymax_2d

")



(provide :gb_visual_detection_3d_msgs/BoundingBox3d "2ce9b0738c577c56440a33d88e32daf4")


