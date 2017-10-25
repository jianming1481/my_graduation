cmake_minimum_required(VERSION 2.8.3)
project(deeplab_resnet_pkg)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  rospy
  sensor_msgs
  cv_bridge
  image_transport
  std_msgs
  message_generation
)
find_package(Boost REQUIRED COMPONENTS system)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  Segmentation.srv
)
generate_messages(
   DEPENDENCIES sensor_msgs
)

catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES trainning_data_builder
  CATKIN_DEPENDS pcl_conversions pcl_ros roscpp sensor_msgs cv_bridge image_transport roscpp rospy std_msgs message_runtime
  DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
# add_executable(trainning_data_builder_node src/trainning_data_builder.cpp)
# add_executable(pcd_reader src/pcd_reader.cpp)

# target_link_libraries(trainning_data_builder_node ${catkin_LIBRARIES})
# target_link_libraries(pcd_reader ${catkin_LIBRARIES})
