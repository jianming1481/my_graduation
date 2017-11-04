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
 #include <iostream> 
 #include <string>
#include <boost/thread/thread.hpp>
#include <Eigen/Core>

// Include stuff about ROS (Include topic or service)
#include "ros_libraries.hpp"

// Include stuff about PCL
#include "pcl_libraries.hpp"

using namespace std;

enum ProcessingState{
    NADA,
    FOTO,
    SEGMETATION,
    ALIGMENT,
    POSE_ESTIMATION
}state, next_state;

enum LabelList{
  Back_Ground,
  Hand_Weight,
  Crayons,
  Minion_Cup,
  Koopa,
  Robots_Everywhere
}goal_label;

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
    scene_cloud(new PCT),
    m_cloud(new PCT),
    object (new PointCloudT),
    object_aligned (new PointCloudT),
    scene (new PointCloudT),
    object_features (new FeatureCloudT),
    scene_features (new FeatureCloudT)
    {
      // Regist the action in ROS
      as_.registerGoalCallback(boost::bind(&ObjEstAction::goalCB, this));
      as_.registerPreemptCallback(boost::bind(&ObjEstAction::preemptCB, this));

      // Subscribe the point cloud from SR300
      cloud_sub = nh_.subscribe("/camera/depth_registered/points", 1, &ObjEstAction::cloudCB,this);
      segment_client = nh_.serviceClient<deeplab_resnet_pkg::Segmentation>("Semantic_Segmentation");

      // Test the label image is correct or not
      label_image_pub = nh_.advertise<sensor_msgs::Image>("service_label_image", 1);

      // Strart action
      as_.start();
      ROS_INFO("Obj_estimate is ready!");
    }

  ~ObjEstAction()
  {
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

  // Call deeplab_resnet Service to Do Semantic Segmentation and Get Label Image
  bool segment();

  // Extract the point cloud from scene_cloud based on label image
  void extract_cloud(sensor_msgs::Image label_image);

  // Estimate the pose of object
  void estimate_object_pose(PCT::Ptr object_cloud);

  // Save Point Cloud Data
  void write_pcd_2_rospack(PCT::Ptr cloud, std::string f_name,bool breakup_with_ex);
protected:

private:
  //------PCL--------//
  // The Point Cloud Data of Whole Scene
  PCT::Ptr scene_cloud ;
  // The Point Cloud to Storage Desired Object
  PCT::Ptr m_cloud;
  // Point Cloud with Normal
  PointCloudT::Ptr object;
  PointCloudT::Ptr object_aligned;
  PointCloudT::Ptr scene;

  // Point Cloud with FPFH Feature
  FeatureCloudT::Ptr object_features;
  FeatureCloudT::Ptr scene_features;

  //------ROS--------//
  ros::NodeHandle nh_;
  ros::Publisher label_image_pub;
  ros::Subscriber cloud_sub;
  ros::Subscriber label_sub;
  ros::ServiceClient segment_client;

  // For action!
  actionlib::SimpleActionServer<object_pose_estimation::ObjectPoseAction> as_;
  std::string action_name_;
  object_pose_estimation::ObjectPoseFeedback feedback_;
  object_pose_estimation::ObjectPoseResult result_;
  // To receive object name from action
  std::string obj_name;

  // For service
  deeplab_resnet_pkg::Segmentation seg_srv;

  // For topic
  sensor_msgs::Image L_Img;

  // The path to save foto or pcd
  std::string path;

  // Index of desired label in the image
  std::vector<int> index_list;
};
}
