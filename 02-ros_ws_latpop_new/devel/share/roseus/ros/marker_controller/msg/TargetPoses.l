;; Auto-generated. Do not edit!


(when (boundp 'marker_controller::TargetPoses)
  (if (not (find-package "MARKER_CONTROLLER"))
    (make-package "MARKER_CONTROLLER"))
  (shadow 'TargetPoses (find-package "MARKER_CONTROLLER")))
(unless (find-package "MARKER_CONTROLLER::TARGETPOSES")
  (make-package "MARKER_CONTROLLER::TARGETPOSES"))

(in-package "ROS")
;;//! \htmlinclude TargetPoses.msg.html


(defclass marker_controller::TargetPoses
  :super ros::object
  :slots (_targets ))

(defmethod marker_controller::TargetPoses
  (:init
   (&key
    ((:targets __targets) (let (r) (dotimes (i 0) (push (instance marker_controller::TargetPose :init) r)) r))
    )
   (send-super :init)
   (setq _targets __targets)
   self)
  (:targets
   (&rest __targets)
   (if (keywordp (car __targets))
       (send* _targets __targets)
     (progn
       (if __targets (setq _targets (car __targets)))
       _targets)))
  (:serialization-length
   ()
   (+
    ;; marker_controller/TargetPose[] _targets
    (apply #'+ (send-all _targets :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; marker_controller/TargetPose[] _targets
     (write-long (length _targets) s)
     (dolist (elem _targets)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; marker_controller/TargetPose[] _targets
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _targets (let (r) (dotimes (i n) (push (instance marker_controller::TargetPose :init) r)) r))
     (dolist (elem- _targets)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get marker_controller::TargetPoses :md5sum-) "3a8172c4ff6212bb43118476f3f97c7b")
(setf (get marker_controller::TargetPoses :datatype-) "marker_controller/TargetPoses")
(setf (get marker_controller::TargetPoses :definition-)
      "TargetPose[] targets

================================================================================
MSG: marker_controller/TargetPose
uint32 id
geometry_msgs/PoseStamped pose

================================================================================
MSG: geometry_msgs/PoseStamped
# A Pose with reference coordinate frame and timestamp
Header header
Pose pose

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

")



(provide :marker_controller/TargetPoses "3a8172c4ff6212bb43118476f3f97c7b")


