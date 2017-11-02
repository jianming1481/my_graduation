#include "object_pose_estimator.hpp"

using namespace ObjEstAction_namespace;

void ObjEstAction::goalCB()
{
  const object_pose_estimation::ObjectPoseGoalConstPtr goal = as_.acceptNewGoal();
  // std::string Anita("I love you Lui!");
  // ROS_INFO("string: Goal is %s",Anita);
  // ROS_INFO("c_str : Goal is %s",goal->object_label;
  // goal_label = goal->object_label;
  ROS_INFO("Goal Label: %d",goal->object_label);
  
  switch(goal->object_label)
  {
    case Hand_Weight:
      goal_label = Hand_Weight;
      break;
    case Crayons:
      goal_label = Crayons;
      break;
    case Minion_Cup:
      goal_label = Minion_Cup;
      break;
    case Koopa:
      goal_label = Koopa;    
      break;
    case Robots_Everywhere:
      goal_label = Robots_Everywhere;  
      break;
    default:
      goal_label = Back_Ground;
  }
  state = FOTO;
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
    write_pcd_2_rospack(scene_cloud,"scene_cloud.pcd",true);
#endif
    state = SEGMETATION;
  }
}

void ObjEstAction::extract_cloud(sensor_msgs::Image label_image)
{
  int width = label_image.width;
  int height = label_image.height;
  int point_cloud_size = 0;
  ROS_INFO("Current Label: %d",goal_label);  
  
  for(int i=0;i<height;i++)
  {
    for(int j=0;j<width;j++)
    {
      if(label_image.data[i*width+j]==goal_label)
      {
        index_list.push_back(i*label_image.width+j);
        label_image.data[i*width+j] = 255;
        point_cloud_size++;
      }
    }
  }
  std::reverse(index_list.begin(),index_list.end());
  if(point_cloud_size==0)
  {
    return;
  }
  m_cloud->width = point_cloud_size;
  m_cloud->height=1;
  m_cloud->points.resize(m_cloud->width*m_cloud->height);
  for(int i=0;i<point_cloud_size;i++)
  {
    m_cloud->points[i].x = scene_cloud->points[index_list[i]].x;
    m_cloud->points[i].y = scene_cloud->points[index_list[i]].y;
    m_cloud->points[i].z = scene_cloud->points[index_list[i]].z;
    m_cloud->points[i].rgb = scene_cloud->points[index_list[i]].rgb;
  }
  label_image_pub.publish(label_image);
  write_pcd_2_rospack(m_cloud,"m_cloud.pcd",false);
}

bool ObjEstAction::segment()
{
  seg_srv.request.data = true;
  segment_client.call(seg_srv);
  L_Img = seg_srv.response.img;
  extract_cloud(L_Img);
  state = NADA;
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



void ObjEstAction::write_pcd_2_rospack(PCT::Ptr cloud, std::string f_name,bool breakup_with_ex){
  if(breakup_with_ex)
  {
    //Remove All PCD File in [package]/pcd_file/*.pcd      
    std::string sys_str;
    path = ros::package::getPath("object_pose_estimation");
    path.append("/pcd_file/");
    sys_str = "rm  " +  path + "*.pcd";
    std::cout << "[CMD] -> " << sys_str << std::endl;  
    system(sys_str.c_str());
  }

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