# Open realsense sr300
roslaunch realsense_camera sr300_nodelet_rgbd.launch

# ROS connect with Arduino
rosrun rosserial_python serial_node.py /dev/ttyACM0

# Start the node to build data
rosrun trainning_data_builder trainning_data_builder_node 

