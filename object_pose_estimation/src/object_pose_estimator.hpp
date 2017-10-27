#include <string>
#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/String.h>


#include <sensor_msgs/PointCloud2.h>
#include <actionlib/server/simple_action_server.h>
#include <object_pose_estimation/ObjectPoseAction.h>

#include <boost/thread/thread.hpp>
#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/time.h>
#include <pcl/console/print.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl_ros/transforms.h>
#include <pcl/console/parse.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/surface/mls.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <tf/transform_broadcaster.h>
#include "tf_conversions/tf_eigen.h"



enum ProcessingState{
    NADA,
    FOTO,
    SEGMETATION,
    ALIGMENT,
    POSE_ESTIMATION,
}state, next_state;

namespace ObjEstAction_namespace
{

class ObjEstAction
{
public:

  ObjEstAction(int argc, char **argv, std::string name) :
    as_(nh_, name, false),
    action_name_(name)
  {
    ROS_INFO("obj_pose READY!");
  }
  void cloudCB(const sensor_msgs::PointCloud2ConstPtr& input);
  void goalCB();
  void preemptCB();
  void pub_feedback(std::string msg,int progress);
  void pub_error(const char *err_msg);

protected:
    
  //------ROS--------//
  ros::NodeHandle nh_;

  actionlib::SimpleActionServer<object_pose_estimation::ObjectPoseAction> as_;
  std::string action_name_;

  object_pose_estimation::ObjectPoseFeedback feedback_;
  object_pose_estimation::ObjectPoseResult result_;

private:

};
}
