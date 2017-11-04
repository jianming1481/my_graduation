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

#ifdef SaveCloud
  write_pcd_2_rospack(m_cloud,"m_cloud.pcd",false);
#endif
  estimate_object_pose(m_cloud);
}

void ObjEstAction::estimate_object_pose(PCT::Ptr object_cloud)
{
  pcl::io::loadPCDFile<PointNT> ("/home/iclab-gtx1080/graduation_ws/src/my_graduation/object_pose_estimation/pcd_file/scene_cloud.pcd", *scene);
  pcl::io::loadPCDFile<PointNT> ("/home/iclab-gtx1080/graduation_ws/src/my_graduation/object_pose_estimation/pcd_file/m_cloud.pcd", *object);
  // pcl::copyPointCloud(*m_cloud, *object);
  // for(int i=0;i<object_cloud->size();i++)
  // {
  //   PointNT point;
    
  //   point.x = object_cloud->points[i].x;
  //   point.y = object_cloud->points[i].y;
  //   point.z = object_cloud->points[i].z;
  //   point.rgb = object_cloud->points[i].rgb;
  //   object->points.push_back(point);
  // }
  // object->width = object->points.size();
  // object->height = 1;

  // scene->points.resize(scene_cloud->size());
  // for(int i=0;i<scene_cloud->size();i++)
  // {
  //   scene->points[i].x = scene_cloud->points[i].x;
  //   scene->points[i].y = scene_cloud->points[i].y;
  //   scene->points[i].z = scene_cloud->points[i].z;
  //   scene->points[i].rgb = scene_cloud->points[i].rgb;
  // }
  std::cerr << "Scene before filtering: " << scene->width * scene->height << std::endl;
 
  
  pcl::removeNaNFromPointCloud(*object,*object, indices_obj);   
  pcl::removeNaNFromPointCloud(*scene,*scene, indices_sce);

  // Find the package to storage the pcd file
  path = ros::package::getPath("object_pose_estimation");
  path.append("/pcd_file/");
  path.append("m_cloud_removeNAN.pcd");

  std::cout << "Save PCD -> " << path << std::endl;
  pcl::PCDWriter writer;
  writer.write<PointNT> (path, *object, false);  

  // Downsample
  pcl::console::print_highlight ("Downsampling...\n");
  pcl::VoxelGrid<PointNT> grid;
  const float leaf = 0.005f;
  grid.setLeafSize (leaf, leaf, leaf);
  grid.setInputCloud (object);
  grid.filter (*object);
  grid.setInputCloud (scene);
  grid.filter (*scene);
  
  std::cerr << "Scene after filtering: " << scene->width * scene->height << std::endl;


  // Estimate normals for object
  pcl::console::print_highlight ("Estimating object normals...\n");
  pcl::NormalEstimationOMP<PointNT,PointNT> nest_obj;
  nest_obj.setRadiusSearch (0.01);
  nest_obj.setInputCloud (object);
  nest_obj.compute (*object);

  // Estimate normals for scene
  pcl::console::print_highlight ("Estimating scene normals...\n");
  pcl::NormalEstimationOMP<PointNT,PointNT> nest;
  nest.setRadiusSearch (0.01);
  nest.setInputCloud (scene);
  nest.compute (*scene);
  
  // Estimate features
  pcl::console::print_highlight ("Estimating features...\n");
  FeatureEstimationT fest;
  fest.setRadiusSearch (0.025);
  fest.setInputCloud (object);
  fest.setInputNormals (object);
  fest.compute (*object_features);
  fest.setInputCloud (scene);
  fest.setInputNormals (scene);
  fest.compute (*scene_features);
  
  // Perform alignment
  pcl::console::print_highlight ("Starting alignment...\n");
  pcl::SampleConsensusPrerejective<PointNT,PointNT,FeatureT> align;
  align.setInputSource (object);
  align.setSourceFeatures (object_features);
  align.setInputTarget (scene);
  align.setTargetFeatures (scene_features);
  align.setMaximumIterations (500000*10); // Number of RANSAC iterations
  align.setNumberOfSamples (5); // Number of points to sample for generating/prerejecting a pose
  align.setCorrespondenceRandomness (5); // Number of nearest features to use
  align.setSimilarityThreshold (0.9f); // Polygonal edge length similarity threshold
  align.setMaxCorrespondenceDistance (2.5f * leaf); // Inlier threshold
  align.setInlierFraction (0.25f); // Required inlier fraction for accepting a pose hypothesis
  {
    pcl::ScopeTime t("Alignment");
    align.align (*object_aligned);
  }
  
  if (align.hasConverged ())
  {
    // Print results
    printf ("\n");
    float roll,pitch,yaw;
    Eigen::Matrix4f transformation = align.getFinalTransformation ();
    pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (0,0), transformation (0,1), transformation (0,2));
    pcl::console::print_info ("R = | %6.3f %6.3f %6.3f | \n", transformation (1,0), transformation (1,1), transformation (1,2));
    pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (2,0), transformation (2,1), transformation (2,2));
    pcl::console::print_info ("\n");
    pcl::console::print_info ("t = < %0.3f, %0.3f, %0.3f >\n", transformation (0,3), transformation (1,3), transformation (2,3));

    Eigen::Vector4f centroid;
    pcl::compute3DCentroid (*scene, centroid);
    printf("center point = < %6.3f, %6.3f, %6.3f >\n", centroid(0)*100, centroid(1)*100, centroid(2)*100);

    Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
    transform_2 = transformation;
    pcl::getEulerAngles(transform_2,roll,pitch,yaw);
    std::cout << "Roll=" << roll << std::endl;
    std::cout << "Pitch=" << pitch << std::endl;
    std::cout << "Yaw=" << yaw << std::endl;
    pcl::console::print_info ("\n");
    pcl::console::print_info ("Inliers: %i/%i\n", align.getInliers ().size (), object->size ());
    
    // Show alignment
    pcl::visualization::PCLVisualizer visu("Alignment");
    visu.addCoordinateSystem (0.1, 0);
    visu.addPointCloud (scene, ColorHandlerT (scene, 0.0, 255.0, 0.0), "scene");
    visu.addPointCloud (object_aligned, ColorHandlerT (object_aligned, 255.0, 0.0, 0.0), "object_aligned");
    visu.spin ();
  }
  else
  {
    pcl::console::print_error ("Alignment failed!\n");
    return;
  }
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