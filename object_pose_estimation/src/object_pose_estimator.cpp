#include "object_pose_estimator.hpp"

using namespace ObjEstAction_namespace;

void ObjEstAction::goalCB()
{
  const object_pose_estimation::ObjectPoseGoalConstPtr goal = as_.acceptNewGoal();
  obj_name = goal->object_name;
  state = SEGMETATION;
}

void ObjEstAction::preemptCB()
{
}

void ObjEstAction::cloudCB(const sensor_msgs::PointCloud2ConstPtr& input)
{
  if(state==FOTO){
    pub_feedback("Grabbing point cloud...",20);
    pcl::fromROSMsg(*input,*scene_cloud);

#ifdef SaveCloud
    //write pcd
    write_pcd_2_rospack(scene_cloud,"scene_cloud.pcd");
#endif
    state = NADA;
  }
}

void ObjEstAction::pub_feedback(std::string msg,int progress)
{
  feedback_.msg = msg;
  feedback_.progress = progress;
  as_.publishFeedback(feedback_);
}

void ObjEstAction::pub_error(const char *err_msg)
{
}

bool ObjEstAction::segment()
{
  seg_srv.request.data = true;
  segment_client.call(seg_srv);
  state = NADA;
}

void ObjEstAction::write_pcd_2_rospack(PCT::Ptr cloud, std::string f_name){
  //Remove All PCD File in [package]/pcd_file/*.pcd      
  std::string sys_str;
  path = ros::package::getPath("object_pose_estimation");
  path.append("/pcd_file/");
  sys_str = "rm  " +  path + "*.pcd";
  std::cout << "[CMD] -> " << sys_str << std::endl;  
  system(sys_str.c_str());

  // Find the package to storage the pcd file
  path = ros::package::getPath("object_pose_estimation");
  path.append("/pcd_file/");
  path.append(f_name);

  std::cout << "Save PCD -> " << path << std::endl;
  pcl::PCDWriter writer;
  writer.write<PT> (path, *cloud, false);
}

int main (int argc, char **argv)
{
  ros::init(argc, argv, "obj_pose");
  ObjEstAction ObjEst(argc, argv,"obj_pose");
  ros::Rate loop_rate(100);
  while(ros::ok())
  {
    switch(state)
    {
      case NADA:
        break;
        
      case FOTO:
        // To Save the 2D Image or Point Cloud Data
        break;

      case SEGMETATION:
        // Call Service to Do Semantic Segmentation
        ObjEst.segment();
        break;

      default:
        ROS_INFO("Que!?");
        state = NADA;
      
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  
  return (0);
}