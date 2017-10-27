/***********************************************************************
 *  Object Pose Estimation
 *
 *  作者: Chien-Ming Lin (2017/10/27)
 * 
 *  The vision system to estimate the pose of object!
 * 
 *  Input:  Image:
 *          Point Cloud:
 *  Output: 6D Pose: 
 ***********************************************************************/
#include <string>
#include <boost/thread/thread.hpp>
#include <Eigen/Core>

// Include stuff about ROS (Include topic or service)
#include "ros_libraries.hpp"

// Include stuff about PCL
#include "pcl_libraries.hpp"

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

  // Constructor Function for ObjEstAction Class
  ObjEstAction(int argc, char **argv, std::string name) :
    // Initial the parameter
    as_(nh_, name, false),
    action_name_(name),
    scene_cloud(new PCT)
    {
      // Regist the action in ROS
      as_.registerGoalCallback(boost::bind(&ObjEstAction::goalCB, this));
      as_.registerPreemptCallback(boost::bind(&ObjEstAction::preemptCB, this));

      // Subscribe the point cloud from SR300
      cloud_sub = nh_.subscribe("/camera/depth_registered/points", 10, &ObjEstAction::cloudCB,this);
      segment_client = nh_.serviceClient<deeplab_resnet_pkg::Segmentation>("Semantic_Segmentation");

      // Strart action
      as_.start();
      ROS_INFO("Obj_estimate is ready!");
    }

  // Callback Function for Subscribe Point Cloud
  void cloudCB(const sensor_msgs::PointCloud2ConstPtr& input);
  // Callback Function for Waiting the Call of Action
  void goalCB();

  // Callback Function for Preempt the Action
  void preemptCB();

  // Send the Feed Back of Vision System
  void pub_feedback(std::string msg,int progress);

  // Send the Error Code of Vision System
  void pub_error(const char *err_msg);

  // Call deeplab_resnet Service to Do Semantic Segmentation
  bool segment();

  // Save Point Cloud Data
  void write_pcd_2_rospack(PCT::Ptr cloud, std::string f_name);
protected:

private:
  //------PCL--------//
  // The Point Cloud Data of Whole Scene
  PCT::Ptr scene_cloud ;

  //------ROS--------//
  ros::NodeHandle nh_;
  ros::Subscriber cloud_sub;
  ros::ServiceClient segment_client;

  // For action!
  actionlib::SimpleActionServer<object_pose_estimation::ObjectPoseAction> as_;
  std::string action_name_;
  object_pose_estimation::ObjectPoseFeedback feedback_;
  object_pose_estimation::ObjectPoseResult result_;

  // For service
  deeplab_resnet_pkg::Segmentation seg_srv;

  // To receive object name from action
  std::string obj_name;
  // To save foto or pcd
  std::string path;
};
}
