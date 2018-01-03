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
      //General parameters for Correspond grouping
      pcl::console::parse_argument (argc, argv, "--model_ss", model_ss_);
      pcl::console::parse_argument (argc, argv, "--scene_ss", scene_ss_);
      pcl::console::parse_argument (argc, argv, "--rf_rad", rf_rad_);
      pcl::console::parse_argument (argc, argv, "--descr_rad", descr_rad_);
      pcl::console::parse_argument (argc, argv, "--cg_size", cg_size_);
      pcl::console::parse_argument (argc, argv, "--cg_thresh", cg_thresh_);
      pcl::console::parse_argument (argc, argv, "--icp_max_iter", icp_max_iter_);
      pcl::console::parse_argument (argc, argv, "--icp_corr_distance", icp_corr_distance_);
      pcl::console::parse_argument (argc, argv, "--hv_clutter_reg", hv_clutter_reg_);
      pcl::console::parse_argument (argc, argv, "--hv_inlier_th", hv_inlier_th_);
      pcl::console::parse_argument (argc, argv, "--hv_occlusion_th", hv_occlusion_th_);
      pcl::console::parse_argument (argc, argv, "--hv_rad_clutter", hv_rad_clutter_);
      pcl::console::parse_argument (argc, argv, "--hv_regularizer", hv_regularizer_);
      pcl::console::parse_argument (argc, argv, "--hv_rad_normals", hv_rad_normals_);
      pcl::console::parse_argument (argc, argv, "--hv_detect_clutter", hv_detect_clutter_); 

      // Regist the action in ROS
      as_.registerGoalCallback(boost::bind(&ObjEstAction::goalCB, this));
      as_.registerPreemptCallback(boost::bind(&ObjEstAction::preemptCB, this));

      // Subscribe the point cloud from SR300
      cloud_sub = nh_.subscribe("/camera/depth_registered/points", 1, &ObjEstAction::cloudCB,this);

      // Subscrib the Joint State of Robot
      joint_state_sub = nh_.subscribe("/robotis/present_joint_states", 1, &ObjEstAction::joint_state_CB, this);

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

  // Callback Function for Subscrib Joint State of robot
  void joint_state_CB(const sensor_msgs::JointState::ConstPtr& joint);

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

  // For some calculation
  Eigen::Quaterniond euler2Quaternion( double roll, double pitch, double yaw );

  // Estimate the pose of object
  void estimate_object_pose(PCT::Ptr object_cloud);
  void estimate_object_pose_CG(PCT::Ptr object_cloud);

  // Print rotation matrix
  void printRotateMatrix (const Eigen::Matrix4f & matrix);

  // Transfer Relative pose between Object and Camera to Object and Robot
  void transfer_2_robot_frame(Eigen::Matrix4f relative_transform);

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
  ros::Subscriber joint_state_sub;
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

  // Transformation frame from robot_arm_base to camera_rgb_optical
  Eigen::Matrix4f camera_rgb_optical_frame;
};
}
