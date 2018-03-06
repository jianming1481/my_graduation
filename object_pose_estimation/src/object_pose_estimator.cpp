#include "object_pose_estimator.hpp"
#include "ICP_alignment.hpp"

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
  feedback_.progress = 10;
  as_.publishFeedback(feedback_);
}

void ObjEstAction::preemptCB()
{
    ROS_INFO("%s: Preempted", action_name_.c_str());
    // set the action state to preempted
    as_.setPreempted();
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
  m_cloud->is_dense = false;
  m_cloud->points.resize(point_cloud_size);  
  
  for(int i=0;i<point_cloud_size;i++)
  {
    PT point;
    point.x = scene_cloud->points[index_list[i]].x;
    point.y = scene_cloud->points[index_list[i]].y;
    point.z = scene_cloud->points[index_list[i]].z;
    point.rgb = scene_cloud->points[index_list[i]].rgb;

    m_cloud->push_back(point);
  }

  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*m_cloud,*m_cloud, indices);  

  // Publish label image to show what we find
  label_image_pub.publish(label_image);

  // Create the filtering object
  // pcl::StatisticalOutlierRemoval<PT> sor;
  // sor.setInputCloud (m_cloud);
  // sor.setMeanK (50);
  // sor.setStddevMulThresh (2.0);
  // sor.filter (*m_cloud);

#ifdef SaveCloud
  write_pcd_2_rospack(m_cloud,"m_cloud.pcd",false);
#endif
  do_ICP(m_cloud);
  estimate_object_pose(m_cloud);
  //estimate_object_pose_CG(m_cloud);
}

void ObjEstAction::do_ICP(PCT::Ptr object_cloud){
  switch(goal_label){
    case 1:
      pcl::io::loadPCDFile<PointNT> ("/home/iclab-giga/graduate_ws/src/object_pose_estimation/pcd_file/model/Hand_Weight1.pcd", *object);
      break;
    case 2:
      pcl::io::loadPCDFile<PointNT> ("/home/iclab-giga/graduate_ws/src/object_pose_estimation/pcd_file/model/Crayons.pcd", *object);
      break;
    case 5:
      pcl::io::loadPCDFile<PointNT> ("/home/iclab-giga/graduate_ws/src/object_pose_estimation/pcd_file/model/Robots_Everywhere.pcd", *object);
      break;
  }
  Eigen::Matrix4f transformation_ICP;
  Eigen::Matrix4f transformation_matrix;
  pcl::console::print_info ("Doing ICP...\n");
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr model_PCD (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ> temp2;
  copyPointCloud(*m_cloud, *cloud_xyz);
  copyPointCloud(*object, *model_PCD);
  // writer.write<pcl::PointXYZ> ("/home/iclab-giga/Documents/hand_weight_scene/trans_out_scene.pcd", *cloud_xyz, false);  
  // writer.write<pcl::PointXYZ> ("/home/iclab-giga/Documents/hand_weight_scene/trans_out_object.pcd", *model_PCD, false);  
  ICP_alignment my_icp;
  my_icp.setSourceCloud(model_PCD);
  my_icp.setTargetCloud(cloud_xyz);
  my_icp.align(*model_PCD);
  printf("ICP align Score = %f\n",my_icp.getScore());
  transformation_ICP = my_icp.getMatrix ();
  printRotateMatrix(transformation_matrix);
  transfer_2_robot_frame(transformation_matrix);

  Eigen::Vector4f centroid;
  pcl::compute3DCentroid (*model_PCD, centroid);
  // printf("center point = < %6.3f, %6.3f, %6.3f >\n", centroid(0)*100, centroid(1)*100, centroid(2)*100);
  
  // Show alignment
  pcl::visualization::PCLVisualizer visu("Alignment");
  visu.addCoordinateSystem (0.1, 0);
  visu.addPointCloud (cloud_xyz, ColorHandlerT (cloud_xyz, 0.0, 255.0, 0.0), "scene");
  visu.addPointCloud (model_PCD, ColorHandlerT (model_PCD, 255.0, 0.0, 0.0), "object_aligned");
  visu.spin ();
}

void ObjEstAction::estimate_object_pose(PCT::Ptr object_cloud)
{
  // Load the PCD model to compare how much it rotate with model
  switch(goal_label){
    case 1:
      pcl::io::loadPCDFile<PointNT> ("/home/iclab-giga/graduate_ws/src/object_pose_estimation/pcd_file/model/Hand_Weight1.pcd", *object);
      break;
    case 2:
      pcl::io::loadPCDFile<PointNT> ("/home/iclab-giga/graduate_ws/src/object_pose_estimation/pcd_file/model/Crayons.pcd", *object);
      break;
    case 5:
      pcl::io::loadPCDFile<PointNT> ("/home/iclab-giga/graduate_ws/src/object_pose_estimation/pcd_file/model/Robots_Everywhere.pcd", *object);
      break;
  }
  // pcl::io::loadPCDFile<PointNT> ("/home/iclab-giga/graduate_ws/src/object_pose_estimation/pcd_file/model/Hand_Weight1.pcd", *object);

  // pcl::io::loadPCDFile<PointNT> ("/home/iclab-giga/graduate_ws/src/object_pose_estimation/pcd_file/m_cloud.pcd", *object);

  // Change Point cloud type from XYZRGBZ to XYZRGBANormal
  pcl::copyPointCloud(*m_cloud, *scene);

  std::cerr << "object before filtering: " << object->width * object->height << std::endl;
  pcl::PCDWriter writer;

  std::vector<int> indices_sce; 
  // pcl::removeNaNFromPointCloud(*scene,*scene, indices_sce);

  // // Find the package to storage the pcd file
  // path = ros::package::getPath("object_pose_estimation");
  // path.append("/pcd_file/");
  // path.append("m_cloud_removeNAN.pcd");

  // std::cout << "Save PCD -> " << path << std::endl;
  // writer.write<PointNT> (path, *object, false);  

  // Downsample
  pcl::console::print_highlight ("Downsampling...\n");
  pcl::VoxelGrid<PointNT> grid;
  const float leaf = 0.005f;
  grid.setLeafSize (leaf, leaf, leaf);
  grid.setInputCloud (object);
  grid.filter (*object);
  grid.setInputCloud (scene);
  grid.filter (*scene);
  
  std::cerr << "object after filtering: " << object->width * object->height << std::endl;
  std::cerr << "scene after filtering: " << scene->width * scene->height << std::endl;

  Eigen::Matrix4f transformation_ICP;
  Eigen::Matrix4f transformation_matrix;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr model_PCD (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ> temp2;

  pcl::console::print_info ("Doing ICP...\n");
  copyPointCloud(*object, *model_PCD);
  copyPointCloud(*scene, *cloud_xyz);

  // build the condition
  pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond (new pcl::ConditionAnd<pcl::PointXYZ> ());
  range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::GT, 0.0)));
  range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::LT, 0.8)));
  // build the filter
  pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
  condrem.setCondition (range_cond);
  condrem.setInputCloud (cloud_xyz);
  condrem.setKeepOrganized(true);
  // apply filter
  condrem.filter (*cloud_xyz);
  pcl::removeNaNFromPointCloud(*cloud_xyz,*cloud_xyz, indices_sce);

  Eigen::Vector4f centroid;
  pcl::compute3DCentroid (*cloud_xyz, centroid);
  printf("scene center point = < %6.3f, %6.3f, %6.3f >\n", centroid(0), centroid(1), centroid(2));
  // Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
  // float theta = 0; // The angle of rotation in radians
  // transform_1 (0,0) = cos (theta);
  // transform_1 (0,1) = -sin(theta);
  // transform_1 (1,0) = sin (theta);
  // transform_1 (1,1) = cos (theta);

  // // Define a translation of 2.5 meters on the x axis.
  // transform_1 (0,3) = centroid(0);
  // transform_1 (1,3) = centroid(1);
  // transform_1 (2,3) = centroid(2);
  // pcl::transformPointCloud (*model_PCD, *model_PCD, transform_1);
  // pcl::compute3DCentroid (*model_PCD, centroid);
  // printf("model center point = < %6.3f, %6.3f, %6.3f >\n", centroid(0), centroid(1), centroid(2));
  ICP_alignment my_icp;
  my_icp.setTargetCloud(cloud_xyz);
  my_icp.setSourceCloud(model_PCD);
  my_icp.align(temp2);

  
  printf("ICP align Score = %f\n",my_icp.getScore());
  transformation_ICP = my_icp.getMatrix ();
  // printRotateMatrix(transformation_ICP);
  // transformation_ICP = transfer_2_robot_frame(transformation_ICP,centroid);
  // transformPointCloud(temp2,temp2,transformation_ICP);

  // pcl::compute3DCentroid (temp2, centroid);
  // printf("center point = < %6.3f, %6.3f, %6.3f >\n", centroid(0), centroid(1), centroid(2));

  //seg_msg.header.frame_id = "robot_arm_base";
  pcl::toROSMsg(temp2, seg_msg);
  seg_msg.header.frame_id = "camera_rgb_optical_frame";
  align_pub_.publish(seg_msg);
  writer.write<pcl::PointXYZ> ("/home/iclab/Documents/hand_weight_scene/trans_out_model.pcd", temp2, false);  
  writer.write<pcl::PointXYZ> ("/home/iclab/Documents/hand_weight_scene/trans_out_scene_object.pcd", *cloud_xyz, false);  
  //----------------------- FOR FPFH and RANSAC------------------------------
  // // Estimate normals for object
  // pcl::console::print_highlight ("Estimating object normals...\n");
  // pcl::NormalEstimationOMP<PointNT,PointNT> nest_obj;
  // nest_obj.setRadiusSearch (0.01);
  // nest_obj.setInputCloud (object);
  // nest_obj.compute (*object);

  // // Estimate normals for scene
  // pcl::console::print_highlight ("Estimating scene normals...\n");
  // pcl::NormalEstimationOMP<PointNT,PointNT> nest;
  // nest.setRadiusSearch (0.01);
  // nest.setInputCloud (scene);
  // nest.compute (*scene);
  
  // // Estimate features
  // pcl::console::print_highlight ("Estimating features...\n");
  // FeatureEstimationT fest;
  // fest.setRadiusSearch (0.025);
  // fest.setInputCloud (object);
  // fest.setInputNormals (object);
  // fest.compute (*object_features);
  // fest.setInputCloud (scene);
  // fest.setInputNormals (scene);
  // fest.compute (*scene_features);
  
  // // Perform alignment
  // pcl::console::print_highlight ("Starting alignment...\n");
  // pcl::SampleConsensusPrerejective<PointNT,PointNT,FeatureT> align;
  // align.setInputSource (object);
  // align.setSourceFeatures (object_features);
  // align.setInputTarget (scene);
  // align.setTargetFeatures (scene_features);
  // align.setMaximumIterations (500000*10); // Number of RANSAC iterations
  // align.setNumberOfSamples (7); // Number of points to sample for generating/prerejecting a pose
  // align.setCorrespondenceRandomness (7); // Number of nearest features to use
  // align.setSimilarityThreshold (0.9f); // Polygonal edge length similarity threshold
  // align.setMaxCorrespondenceDistance (2.5f * leaf); // Inlier threshold
  // align.setInlierFraction (0.25f); // Required inlier fraction for accepting a pose hypothesis
  // {
  //   pcl::ScopeTime t("Alignment");
  //   align.align (*object_aligned);
  // }
  
  // if (align.hasConverged ())
  // {
  //   // Print results
  //   printf ("\n");
  //   float roll,pitch,yaw;
  //   Eigen::Matrix4f transformation_FPFH = align.getFinalTransformation ();

  //   // Show rotate_matrix from FPFH
  //   pcl::console::print_info ("Show rotate_matrix from FPFH\n");
  //   // pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation_FPFH (0,0), transformation_FPFH (0,1), transformation_FPFH (0,2));
  //   // pcl::console::print_info ("R = | %6.3f %6.3f %6.3f | \n", transformation_FPFH (1,0), transformation_FPFH (1,1), transformation_FPFH (1,2));
  //   // pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation_FPFH (2,0), transformation_FPFH (2,1), transformation_FPFH (2,2));
  //   pcl::console::print_info ("\n");
  //   pcl::console::print_info ("t = < %0.3f, %0.3f, %0.3f >\n", transformation_FPFH (0,3), transformation_FPFH (1,3), transformation_FPFH (2,3));
  //   Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
  //   transform_2 = transformation_FPFH;
  //   pcl::getEulerAngles(transform_2,roll,pitch,yaw);
  //   std::cout << "Roll=" << roll << std::endl;
  //   std::cout << "Pitch=" << pitch << std::endl;
  //   std::cout << "Yaw=" << yaw << std::endl;
  //   pcl::console::print_info ("\n");
  //   pcl::console::print_info ("Inliers: %i/%i\n", align.getInliers ().size (), object->size ());

  //   // Do ICP
  //   pcl::console::print_info ("Doing ICP...\n");
  //   copyPointCloud(*scene, *cloud_xyz);
  //   copyPointCloud(*object_aligned, *model_PCD);
  //   writer.write<pcl::PointXYZ> ("/home/iclab/Documents/hand_weight_scene/trans_out_scene.pcd", *cloud_xyz, false);  
  //   writer.write<pcl::PointXYZ> ("/home/iclab/Documents/hand_weight_scene/trans_out_object.pcd", *model_PCD, false);  
  //   ICP_alignment my_icp;
  //   my_icp.setSourceCloud(cloud_xyz);
  //   my_icp.setTargetCloud(model_PCD);
  //   my_icp.align(*cloud_xyz);
  //   pcl::toROSMsg(*model_PCD, seg_msg);
  //   seg_msg.header.frame_id = "camera_rgb_optical_frame";
  //   align_pub_.publish(seg_msg);
  //   printf("ICP align Score = %f\n",my_icp.getScore());
  //   transformation_ICP = my_icp.getMatrix ();
  //   transformation_matrix = transformation_FPFH*transformation_ICP;
  //   printRotateMatrix(transformation_matrix);
  //   // transformation_matrix = transformation_FPFH;
  //   transfer_2_robot_frame(transformation_matrix);

  //   Eigen::Vector4f centroid;
  //   pcl::compute3DCentroid (*model_PCD, centroid);
  //   // printf("center point = < %6.3f, %6.3f, %6.3f >\n", centroid(0)*100, centroid(1)*100, centroid(2)*100);
    
  //   // Send back result
  //   if (!as_.isActive())
  //     return;
  //   obj_pose.linear.x = 0;
  //   result_.object_pose = obj_pose;
  //   as_.setSucceeded(result_);
  //   // Show alignment
  //   // pcl::visualization::PCLVisualizer visu("Alignment");
  //   // visu.addCoordinateSystem (0.1, 0);
  //   // visu.addPointCloud (cloud_xyz, ColorHandlerT (cloud_xyz, 0.0, 255.0, 0.0), "scene");
  //   // visu.addPointCloud (model_PCD, ColorHandlerT (model_PCD, 255.0, 0.0, 0.0), "object_aligned");
  //   // visu.spin ();
  // }
  // else
  // {
  //   pcl::console::print_error ("Alignment failed!\n");
  //   pcl::console::print_info ("Doing ICP...\n");
  //   copyPointCloud(*scene, *cloud_xyz);
  //   copyPointCloud(*object, *model_PCD);
  //   ICP_alignment my_icp;
  //   my_icp.setTargetCloud(model_PCD);
  //   my_icp.setSourceCloud(cloud_xyz);
  //   my_icp.align(*model_PCD);
    
  //   printf("ICP align Score = %f\n",my_icp.getScore());
  //   transformation_ICP = my_icp.getMatrix ();
  //   printRotateMatrix(transformation_ICP);
  //   transformation_ICP = transfer_2_robot_frame(transformation_ICP);
  //   transformPointCloud(*model_PCD,*model_PCD,transformation_ICP);
  //   pcl::toROSMsg(*model_PCD, seg_msg);
  //   Eigen::Vector4f centroid;
  //   pcl::compute3DCentroid (*model_PCD, centroid);
  //   printf("center point = < %6.3f, %6.3f, %6.3f >\n", centroid(0)*100, centroid(1)*100, centroid(2)*100);
  //   writer.write<pcl::PointXYZ> ("/home/iclab/Documents/hand_weight_scene/trans_out_model.pcd", *model_PCD, false);  
  //   writer.write<pcl::PointXYZ> ("/home/iclab/Documents/hand_weight_scene/trans_out_scene_object.pcd", *cloud_xyz, false);  
  //   seg_msg.header.frame_id = "robot_arm_base";
  //   align_pub_.publish(seg_msg);
  //   return;
  // }
}
void ObjEstAction::printRotateMatrix (const Eigen::Matrix4f & matrix)
{
    float roll,pitch,yaw;
    printf ("================= After ICP ================= \n");
    printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
    printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
    printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
    printf ("Translation vector :\n");
    printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));
    Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
    transform_2 = matrix;
    pcl::getEulerAngles(transform_2,roll,pitch,yaw);
    std::cout << "Roll=" << roll << std::endl;
    std::cout << "Pitch=" << pitch << std::endl;
    std::cout << "Yaw=" << yaw << std::endl;
    pcl::console::print_info ("\n");
}

Eigen::Quaterniond ObjEstAction::euler2Quaternion( double roll,
                                                   double pitch,
                                                   double yaw )
{
    // roll = roll/180*3.14159;
    // pitch = pitch/180*3.14159;
    // yaw = yaw/180*3.14159;
    
    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());

    Eigen::Quaterniond q = rollAngle * pitchAngle * yawAngle;
    Eigen::Matrix3d matrix = q.matrix();
    // printf ("Rotation matrix :\n");
    // printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
    // printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
    // printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
    // std::cout << "Eigen = " << q.x() << std::endl << q.y() << std::endl << q.z() << std::endl << q.w() << std::endl;
    return q;
}

void ObjEstAction::joint_state_CB(const sensor_msgs::JointState::ConstPtr& joint)
{
  Eigen::Quaterniond tmp_q;
  tmp_q = euler2Quaternion(0, 0, 0);
  Eigen::Matrix3d tmp = tmp_q.matrix();
  Eigen::Matrix4f tmp_mat;
  Eigen::Matrix4f tmp_rot_mat;
  Eigen::Matrix4f tmp_eef_mat;
  Eigen::Affine3f transformatoin;
  float roll,pitch,yaw;

  // Define robot_arm_base
  tmp_mat << tmp(0,0), tmp(0,1), tmp(0,2), 0,
             tmp(1,0), tmp(1,1), tmp(1,2), 0,
             tmp(2,0), tmp(2,1), tmp(2,2), 0,
                    0,        0,        0, 1;
  // robot_arm_base to joint1
  tmp_q = euler2Quaternion(0, 0, joint->position[0]);
  tmp = tmp_q.matrix();
  tmp_rot_mat << tmp(0,0), tmp(0,1), tmp(0,2),     0,
                 tmp(1,0), tmp(1,1), tmp(1,2),     0,
                 tmp(2,0), tmp(2,1), tmp(2,2), 0.125,
                        0,        0,        0,     1;
  tmp_eef_mat = tmp_mat*tmp_rot_mat;

  // Joint1 to Joint2
  tmp_q = euler2Quaternion(0, joint->position[1], 0);
  tmp = tmp_q.matrix();
  tmp_rot_mat << tmp(0,0), tmp(0,1), tmp(0,2),     0,
                 tmp(1,0), tmp(1,1), tmp(1,2),     0,
                 tmp(2,0), tmp(2,1), tmp(2,2),  0.03,
                        0,        0,        0,     1;
  tmp_eef_mat = tmp_eef_mat*tmp_rot_mat;

  // Joint2 to Joint3
  tmp_q = euler2Quaternion(0, 0, joint->position[2]);
  tmp = tmp_q.matrix();
  tmp_rot_mat << tmp(0,0), tmp(0,1), tmp(0,2),     0,
                 tmp(1,0), tmp(1,1), tmp(1,2),     0,
                 tmp(2,0), tmp(2,1), tmp(2,2),  0.23,
                        0,        0,        0,     1;
  tmp_eef_mat = tmp_eef_mat*tmp_rot_mat;

  // Joint3 to Joint4
  tmp_q = euler2Quaternion(0, joint->position[3], 0);
  tmp = tmp_q.matrix();
  tmp_rot_mat << tmp(0,0), tmp(0,1), tmp(0,2),  0.03,
                 tmp(1,0), tmp(1,1), tmp(1,2),     0,
                 tmp(2,0), tmp(2,1), tmp(2,2),  0.06,
                        0,        0,        0,     1;
  tmp_eef_mat = tmp_eef_mat*tmp_rot_mat;

  // Joint4 to Joint5
  tmp_q = euler2Quaternion(0, 0, joint->position[4]);
  tmp = tmp_q.matrix();
  tmp_rot_mat << tmp(0,0), tmp(0,1), tmp(0,2),  -0.03,
                 tmp(1,0), tmp(1,1), tmp(1,2),      0,
                 tmp(2,0), tmp(2,1), tmp(2,2),   0.23,
                        0,        0,        0,     1;
  tmp_eef_mat = tmp_eef_mat*tmp_rot_mat;

  // Joint5 to Joint6
  tmp_q = euler2Quaternion(0, joint->position[5], 0);
  tmp = tmp_q.matrix();
  tmp_rot_mat << tmp(0,0), tmp(0,1), tmp(0,2),     0,
                 tmp(1,0), tmp(1,1), tmp(1,2),     0,
                 tmp(2,0), tmp(2,1), tmp(2,2),  0.03,
                        0,        0,        0,     1;
  tmp_eef_mat = tmp_eef_mat*tmp_rot_mat;
  
  // Joint6 to Joint7
  tmp_q = euler2Quaternion(0, 0, joint->position[6]);
  tmp = tmp_q.matrix();
  tmp_rot_mat << tmp(0,0), tmp(0,1), tmp(0,2),      0,
                 tmp(1,0), tmp(1,1), tmp(1,2),      0,
                 tmp(2,0), tmp(2,1), tmp(2,2),  0.135,
                        0,        0,        0,      1;
  tmp_eef_mat = tmp_eef_mat*tmp_rot_mat;

  transformatoin.matrix() = tmp_eef_mat;
  pcl::getEulerAngles(transformatoin,roll,pitch,yaw);

  // pcl::console::print_info ("======================= End Effector =======================\n");
  // pcl::console::print_info ("t = < %0.3f, %0.3f, %0.3f >\n", tmp_eef_mat (0,3), tmp_eef_mat (1,3), tmp_eef_mat (2,3));
  // pcl::console::print_info ("roll, pitch, yaw = < %0.3f, %0.3f, %0.3f >\n", roll, pitch, yaw);

  // Joint7 to camera_link
  tmp_q = euler2Quaternion(0, 0, 0);
  tmp = tmp_q.matrix();
  tmp_rot_mat << tmp(0,0), tmp(0,1), tmp(0,2),   0.03,
                 tmp(1,0), tmp(1,1), tmp(1,2),      0,
                 tmp(2,0), tmp(2,1), tmp(2,2),   0.09,
                        0,        0,        0,      1;
  tmp_eef_mat = tmp_eef_mat*tmp_rot_mat;

  // camera_link to camera_rgb_optical
  tmp_q = euler2Quaternion(-3.14159/2, 3.14159/2, 0);
  tmp = tmp_q.matrix();
  tmp_rot_mat << tmp(0,0), tmp(0,1), tmp(0,2), 0.0,
                  tmp(1,0), tmp(1,1), tmp(1,2), 0.0,
                  tmp(2,0), tmp(2,1), tmp(2,2), 0.0,
                        0,        0,        0,    1;
  camera_rgb_optical_frame = tmp_eef_mat*tmp_rot_mat;
}

Eigen::Matrix4f ObjEstAction::transfer_2_robot_frame(Eigen::Matrix4f relative_transform, Eigen::Vector4f centroid)
{
  Eigen::Matrix4f item_frame;
  Eigen::Matrix4f tmp_frame;
  Eigen::Affine3f transformatoin;
  float roll,pitch,yaw;
  // Transpose
  tmp_frame(0,0) = relative_transform(0,0);
  tmp_frame(0,1) = relative_transform(1,0);
  tmp_frame(0,2) = relative_transform(2,0);
  tmp_frame(1,0) = relative_transform(0,1);
  tmp_frame(1,1) = relative_transform(1,1);
  tmp_frame(1,2) = relative_transform(2,1);
  tmp_frame(2,0) = relative_transform(0,2);
  tmp_frame(2,1) = relative_transform(1,2);
  tmp_frame(2,2) = relative_transform(2,2);
  
  // Translation
  tmp_frame(0,3) = relative_transform(0,3)*-1.0;
  tmp_frame(1,3) = relative_transform(1,3)*-1.0;
  tmp_frame(2,3) = relative_transform(2,3)*-1.0;

  tmp_frame(3,0) = 0;
  tmp_frame(3,1) = 0;
  tmp_frame(3,2) = 0;
  tmp_frame(3,3) = 1;

  item_frame = camera_rgb_optical_frame*tmp_frame;
  transformatoin.matrix() = item_frame;
  pcl::getEulerAngles(transformatoin,roll,pitch,yaw);

  pcl::console::print_info ("======================= Item =======================\n");
  pcl::console::print_info ("t = < %0.3f, %0.3f, %0.3f >\n", item_frame (0,3), item_frame (1,3), item_frame (2,3));
  pcl::console::print_info ("roll, pitch, yaw = < %0.3f, %0.3f, %0.3f >\n", roll/3.1415926*180, pitch/3.1415926*180, yaw/3.1415926*180);
  pcl::console::print_info ("roll, pitch, yaw = < %0.3f, %0.3f, %0.3f >\n", roll, pitch, yaw);

  return item_frame;
}

void ObjEstAction::estimate_object_pose_CG(PCT::Ptr object_cloud)
{
  pcl::PointCloud<PT>::Ptr model (new pcl::PointCloud<PT> ());
  pcl::PointCloud<PT>::Ptr model_keypoints (new pcl::PointCloud<PT> ());
  pcl::PointCloud<PT>::Ptr scene (new pcl::PointCloud<PT> ());
  pcl::PointCloud<PT>::Ptr scene_keypoints (new pcl::PointCloud<PT> ());
  pcl::PointCloud<NormalType>::Ptr model_normals (new pcl::PointCloud<NormalType> ());
  pcl::PointCloud<NormalType>::Ptr scene_normals (new pcl::PointCloud<NormalType> ());
  pcl::PointCloud<DescriptorType>::Ptr model_descriptors (new pcl::PointCloud<DescriptorType> ());
  pcl::PointCloud<DescriptorType>::Ptr scene_descriptors (new pcl::PointCloud<DescriptorType> ());

  // Load the PCD model to compare how much it rotate with model
  pcl::io::loadPCDFile<PT> ("/home/iclab/graduation_ws/src/my_graduation/object_pose_estimation/pcd_file/model/hand_weight_model.pcd", *scene);
  // pcl::io::loadPCDFile<PointNT> ("/home/iclab-giga/graduate_ws/src/object_pose_estimation/pcd_file/m_cloud.pcd", *object);

  // Change Point cloud type from XYZRGBZ to XYZRGBANormal
  pcl::copyPointCloud(*m_cloud, *object);

  std::cerr << "object before filtering: " << object->width * object->height << std::endl;
  
  std::vector<int> indices_sce; 
  pcl::removeNaNFromPointCloud(*scene,*scene, indices_sce);

  // Find the package to storage the pcd file
  path = ros::package::getPath("object_pose_estimation");
  path.append("/pcd_file/");
  path.append("m_cloud_removeNAN.pcd");

  std::cout << "Save PCD -> " << path << std::endl;
  pcl::PCDWriter writer;
  writer.write<PointNT> (path, *object, false);  

  /**
   * Compute Normals
   */
  pcl::NormalEstimationOMP<PT, NormalType> norm_est;
  norm_est.setKSearch (10);
  norm_est.setInputCloud (m_cloud);
  norm_est.compute (*model_normals);

  norm_est.setInputCloud (scene);
  norm_est.compute (*scene_normals);

  /**
   *  Downsample Clouds to Extract keypoints
   */
  pcl::UniformSampling<PT> uniform_sampling;
  uniform_sampling.setInputCloud (m_cloud);
  uniform_sampling.setRadiusSearch (model_ss_);
  // uniform_sampling.filter (*model_keypoints);
  pcl::PointCloud<int> keypointIndices1; 
  uniform_sampling.compute(keypointIndices1); 
  pcl::copyPointCloud(*m_cloud, keypointIndices1.points, *model_keypoints); 
  std::cout << "Model total points: " << m_cloud->size () << "; Selected Keypoints: " << model_keypoints->size () << std::endl;

  uniform_sampling.setInputCloud (scene);
  uniform_sampling.setRadiusSearch (scene_ss_);
  // uniform_sampling.filter (*scene_keypoints);
  pcl::PointCloud<int> keypointIndices2; 
  uniform_sampling.compute(keypointIndices2); 
  pcl::copyPointCloud(*scene, keypointIndices2.points, *scene_keypoints); 
  std::cout << "Scene total points: " << scene->size () << "; Selected Keypoints: " << scene_keypoints->size () << std::endl;

  /**
   *  Compute Descriptor for keypoints
   */
  pcl::SHOTEstimationOMP<PT, NormalType, DescriptorType> descr_est;
  descr_est.setRadiusSearch (descr_rad_);

  descr_est.setInputCloud (model_keypoints);
  descr_est.setInputNormals (model_normals);
  descr_est.setSearchSurface (m_cloud);
  descr_est.compute (*model_descriptors);

  descr_est.setInputCloud (scene_keypoints);
  descr_est.setInputNormals (scene_normals);
  descr_est.setSearchSurface (scene);
  descr_est.compute (*scene_descriptors);

  /**
   *  Find Model-Scene Correspondences with KdTree
   */
  pcl::CorrespondencesPtr model_scene_corrs (new pcl::Correspondences ());
  pcl::KdTreeFLANN<DescriptorType> match_search;
  match_search.setInputCloud (model_descriptors);
  std::vector<int> model_good_keypoints_indices;
  std::vector<int> scene_good_keypoints_indices;

  for (size_t i = 0; i < scene_descriptors->size (); ++i)
  {
    std::vector<int> neigh_indices (1);
    std::vector<float> neigh_sqr_dists (1);
    if (!pcl_isfinite (scene_descriptors->at (i).descriptor[0]))  //skipping NaNs
    {
      continue;
    }
    int found_neighs = match_search.nearestKSearch (scene_descriptors->at (i), 1, neigh_indices, neigh_sqr_dists);
    if (found_neighs == 1 && neigh_sqr_dists[0] < 0.25f)
    {
      pcl::Correspondence corr (neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
      model_scene_corrs->push_back (corr);
      model_good_keypoints_indices.push_back (corr.index_query);
      scene_good_keypoints_indices.push_back (corr.index_match);
    }
  }
  pcl::PointCloud<PT>::Ptr model_good_kp (new pcl::PointCloud<PT> ());
  pcl::PointCloud<PT>::Ptr scene_good_kp (new pcl::PointCloud<PT> ());
  pcl::copyPointCloud (*model_keypoints, model_good_keypoints_indices, *model_good_kp);
  pcl::copyPointCloud (*scene_keypoints, scene_good_keypoints_indices, *scene_good_kp);

  std::cout << "Correspondences found: " << model_scene_corrs->size () << std::endl;

  /**
   *  Clustering
   */
  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
  std::vector < pcl::Correspondences > clustered_corrs;

  if (use_hough_)
  {
    pcl::PointCloud<RFType>::Ptr model_rf (new pcl::PointCloud<RFType> ());
    pcl::PointCloud<RFType>::Ptr scene_rf (new pcl::PointCloud<RFType> ());

    pcl::BOARDLocalReferenceFrameEstimation<PT, NormalType, RFType> rf_est;
    rf_est.setFindHoles (true);
    rf_est.setRadiusSearch (rf_rad_);

    rf_est.setInputCloud (model_keypoints);
    rf_est.setInputNormals (model_normals);
    rf_est.setSearchSurface (m_cloud);
    rf_est.compute (*model_rf);

    rf_est.setInputCloud (scene_keypoints);
    rf_est.setInputNormals (scene_normals);
    rf_est.setSearchSurface (scene);
    rf_est.compute (*scene_rf);

    //  Clustering
    pcl::Hough3DGrouping<PT, PT, RFType, RFType> clusterer;
    clusterer.setHoughBinSize (cg_size_);
    clusterer.setHoughThreshold (cg_thresh_);
    clusterer.setUseInterpolation (true);
    clusterer.setUseDistanceWeight (false);

    clusterer.setInputCloud (model_keypoints);
    clusterer.setInputRf (model_rf);
    clusterer.setSceneCloud (scene_keypoints);
    clusterer.setSceneRf (scene_rf);
    clusterer.setModelSceneCorrespondences (model_scene_corrs);

    clusterer.recognize (rototranslations, clustered_corrs);
  }
  else
  {
    pcl::GeometricConsistencyGrouping<PT, PT> gc_clusterer;
    gc_clusterer.setGCSize (cg_size_);
    gc_clusterer.setGCThreshold (cg_thresh_);

    gc_clusterer.setInputCloud (model_keypoints);
    gc_clusterer.setSceneCloud (scene_keypoints);
    gc_clusterer.setModelSceneCorrespondences (model_scene_corrs);

    gc_clusterer.recognize (rototranslations, clustered_corrs);
  }

  /**
   * Stop if no instances
   */
  if (rototranslations.size () <= 0)
  {
    cout << "*** No instances found! ***" << endl;
    return ;
  }
  else
  {
    cout << "Recognized Instances: " << rototranslations.size () << endl << endl;
  }

  /**
   * Generates clouds for each instances found 
   */
  std::vector<pcl::PointCloud<PT>::ConstPtr> instances;

  for (size_t i = 0; i < rototranslations.size (); ++i)
  {
    pcl::PointCloud<PT>::Ptr rotated_model (new pcl::PointCloud<PT> ());
    pcl::transformPointCloud (*m_cloud, *rotated_model, rototranslations[i]);
    instances.push_back (rotated_model);
  }

  /**
   * ICP
   */
  std::vector<pcl::PointCloud<PT>::ConstPtr> registered_instances;
  if (true)
  {
    cout << "--- ICP ---------" << endl;

    for (size_t i = 0; i < rototranslations.size (); ++i)
    {
      pcl::IterativeClosestPoint<PT, PT> icp;
      icp.setMaximumIterations (icp_max_iter_);
      icp.setMaxCorrespondenceDistance (icp_corr_distance_);
      icp.setInputTarget (scene);
      icp.setInputSource (instances[i]);
      pcl::PointCloud<PT>::Ptr registered (new pcl::PointCloud<PT>);
      icp.align (*registered);
      registered_instances.push_back (registered);
      cout << "Instance " << i << " ";
      if (icp.hasConverged ())
      {
        cout << "Aligned!" << endl;
      }
      else
      {
        cout << "Not Aligned!" << endl;
      }
    }

    cout << "-----------------" << endl << endl;
  }

  /**
   * Hypothesis Verification
   */
  cout << "--- Hypotheses Verification ---" << endl;
  std::vector<bool> hypotheses_mask;  // Mask Vector to identify positive hypotheses

  pcl::GlobalHypothesesVerification<PT, PT> GoHv;

  GoHv.setSceneCloud (scene);  // Scene Cloud
  GoHv.addModels (registered_instances, true);  //Models to verify

  GoHv.setInlierThreshold (hv_inlier_th_);
  GoHv.setOcclusionThreshold (hv_occlusion_th_);
  GoHv.setRegularizer (hv_regularizer_);
  GoHv.setRadiusClutter (hv_rad_clutter_);
  GoHv.setClutterRegularizer (hv_clutter_reg_);
  GoHv.setDetectClutter (hv_detect_clutter_);
  GoHv.setRadiusNormals (hv_rad_normals_);

  GoHv.verify ();
  GoHv.getMask (hypotheses_mask);  // i-element TRUE if hvModels[i] verifies hypotheses

  for (int i = 0; i < hypotheses_mask.size (); i++)
  {
    if (hypotheses_mask[i])
    {
      cout << "Instance " << i << " is GOOD! <---" << endl;
    }
    else
    {
      cout << "Instance " << i << " is bad!" << endl;
    }
  }
  cout << "-------------------------------" << endl;

  /**
   *  Visualization
   */
  pcl::visualization::PCLVisualizer viewer ("Hypotheses Verification");
  viewer.addPointCloud (scene, "scene_cloud");

  pcl::PointCloud<PT>::Ptr off_scene_model (new pcl::PointCloud<PT> ());
  pcl::PointCloud<PT>::Ptr off_scene_model_keypoints (new pcl::PointCloud<PT> ());

  pcl::PointCloud<PT>::Ptr off_model_good_kp (new pcl::PointCloud<PT> ());
  pcl::transformPointCloud (*m_cloud, *off_scene_model, Eigen::Vector3f (-1, 0, 0), Eigen::Quaternionf (1, 0, 0, 0));
  pcl::transformPointCloud (*model_keypoints, *off_scene_model_keypoints, Eigen::Vector3f (-1, 0, 0), Eigen::Quaternionf (1, 0, 0, 0));
  pcl::transformPointCloud (*model_good_kp, *off_model_good_kp, Eigen::Vector3f (-1, 0, 0), Eigen::Quaternionf (1, 0, 0, 0));

  if (show_keypoints_)
  {
    CloudStyle modelStyle = style_white;
    pcl::visualization::PointCloudColorHandlerCustom<PT> off_scene_model_color_handler (off_scene_model, modelStyle.r, modelStyle.g, modelStyle.b);
    viewer.addPointCloud (off_scene_model, off_scene_model_color_handler, "off_scene_model");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, modelStyle.size, "off_scene_model");
  }

  if (show_keypoints_)
  {
    CloudStyle goodKeypointStyle = style_violet;
    pcl::visualization::PointCloudColorHandlerCustom<PT> model_good_keypoints_color_handler (off_model_good_kp, goodKeypointStyle.r, goodKeypointStyle.g,
                                                                                                    goodKeypointStyle.b);
    viewer.addPointCloud (off_model_good_kp, model_good_keypoints_color_handler, "model_good_keypoints");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, goodKeypointStyle.size, "model_good_keypoints");

    pcl::visualization::PointCloudColorHandlerCustom<PT> scene_good_keypoints_color_handler (scene_good_kp, goodKeypointStyle.r, goodKeypointStyle.g,
                                                                                                    goodKeypointStyle.b);
    viewer.addPointCloud (scene_good_kp, scene_good_keypoints_color_handler, "scene_good_keypoints");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, goodKeypointStyle.size, "scene_good_keypoints");
  }

  for (size_t i = 0; i < instances.size (); ++i)
  {
    std::stringstream ss_instance;
    ss_instance << "instance_" << i;

    CloudStyle clusterStyle = style_red;
    pcl::visualization::PointCloudColorHandlerCustom<PT> instance_color_handler (instances[i], clusterStyle.r, clusterStyle.g, clusterStyle.b);
    viewer.addPointCloud (instances[i], instance_color_handler, ss_instance.str ());
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, clusterStyle.size, ss_instance.str ());

    CloudStyle registeredStyles = hypotheses_mask[i] ? style_green : style_cyan;
    ss_instance << "_registered" << endl;
    pcl::visualization::PointCloudColorHandlerCustom<PT> registered_instance_color_handler (registered_instances[i], registeredStyles.r,
                                                                                                   registeredStyles.g, registeredStyles.b);
    viewer.addPointCloud (registered_instances[i], registered_instance_color_handler, ss_instance.str ());
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, registeredStyles.size, ss_instance.str ());
  }

  while (!viewer.wasStopped ())
  {
    viewer.spinOnce ();
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
