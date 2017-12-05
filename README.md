# my_graduation

$ roscore
$ sudo bash
$ roslaunch arc control.launch
$ roslaunch realsense_camera sr300_nodelet_rgbd.launch
$ rosrun arc_tf_publisher arc_tf_publisher 
$ rosrun deeplab_resnet_pkg semantic_segmentation.py 
$ rosrun object_pose_estimation object_pose_estimator 
$ rostopic pub /obj_pose/goal object_pose_eimation/ObjectPoseActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  object_label: 1" 

