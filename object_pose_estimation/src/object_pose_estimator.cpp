#include "object_pose_estimator.hpp"

using namespace ObjEstAction_namespace;

void ObjEstAction::goalCB()
{

}

void ObjEstAction::preemptCB()
{

}

void ObjEstAction::cloudCB(const sensor_msgs::PointCloud2ConstPtr& input)
{
}

void ObjEstAction::pub_feedback(std::string msg,int progress)
{
}

void ObjEstAction::pub_error(const char *err_msg)
{
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
