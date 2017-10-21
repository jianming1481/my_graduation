#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/filters/passthrough.h>


// This function displays the help
void
showHelp(char * program_name)
{
  std::cout << std::endl;
  std::cout << "Usage: " << program_name << " cloud_filename.[pcd|ply]" << std::endl;
  std::cout << "-h:  Show this help." << std::endl;
}

// This is the main function
int
main (int argc, char** argv)
{

  // Show help
  if (pcl::console::find_switch (argc, argv, "-h") || pcl::console::find_switch (argc, argv, "--help")) {
    showHelp (argv[0]);
    return 0;
  }

  // Fetch point cloud filename in arguments | Works with PCD and PLY files
  std::vector<int> filenames;

  filenames = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");

  // Load file | Works with PCD
  pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud (new pcl::PointCloud<pcl::PointXYZ> ());

  if (pcl::io::loadPCDFile (argv[filenames[0]], *source_cloud) < 0)  {
    std::cout << "Error loading point cloud " << argv[filenames[0]] << std::endl << std::endl;
    showHelp (argv[0]);
    return -1;
  }else{
      std::cout << "Loading point cloud " << argv[filenames[0]] << std::endl << std::endl;
  }

  //------------------------------------------------------------------------------//
  //                                  Method 1                                    //
  //------------------------------------------------------------------------------//
  /* Reminder: how transformation matrices work :

           |-------> This column is the translation
    | 1 0 0 x |  \
    | 0 1 0 y |   }-> The identity 3x3 matrix (no rotation) on the left
    | 0 0 1 z |  /
    | 0 0 0 1 |    -> We do not use this line (and it has to stay 0,0,0,1)

    METHOD #1: Using a Matrix4f
    This is the "manual" method, perfect to understand but error prone !
  */
  //Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();

  // Define a rotation matrix (see https://en.wikipedia.org/wiki/Rotation_matrix)
  //float theta = M_PI/2; // The angle of rotation in radians
  //transform_1 (0,0) = cos (theta);
  //transform_1 (0,1) = -sin(theta);
  //transform_1 (1,0) = sin (theta);
  //transform_1 (1,1) = cos (theta);
  //    (row, column)

  // Define a translation of 2.5 meters on the x axis.
  //transform_1 (0,3) = 1;

  // Print the transformation
  //printf ("Method #1: using a Matrix4f\n");
  //std::cout << transform_1 << std::endl;

  //------------------------------------------------------------------------------//
  //                                  Method 2                                    //
  //------------------------------------------------------------------------------//
  /*  METHOD #2: Using a Affine3f
    This method is easier and less error prone
  */
  Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();

  // Define a translation of 2.5 meters on the x axis.
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);


  // The same rotation matrix as before; theta radians arround Z axis
  float roll=0;
  float pitch=0;
  float yaw=0;
  if (pcl::console::find_switch (argc, argv, "-x"))
  {
    pcl::console::parse (argc, argv, "-x", roll);
    transform_2.rotate (Eigen::AngleAxisf (roll, Eigen::Vector3f::UnitX()));
    // You can either apply transform_1 or transform_2; they are the same
    pcl::transformPointCloud (*source_cloud, *transformed_cloud, transform_2);
  }else{
    roll = 0.0;
  }
  if (pcl::console::find_switch (argc, argv, "-y"))
  {
    pcl::console::parse (argc, argv, "-y", pitch);
    transform_2.rotate (Eigen::AngleAxisf (pitch, Eigen::Vector3f::UnitY()));
    // You can either apply transform_1 or transform_2; they are the same
    pcl::transformPointCloud (*source_cloud, *transformed_cloud, transform_2);
  }else{
    pitch = 0.0;
  }
  if (pcl::console::find_switch (argc, argv, "-z"))
  {
    pcl::console::parse (argc, argv, "-z", yaw);
    transform_2.rotate (Eigen::AngleAxisf (yaw, Eigen::Vector3f::UnitZ()));
    // You can either apply transform_1 or transform_2; they are the same
    pcl::transformPointCloud (*source_cloud, *transformed_cloud, transform_2);
  }else{
    yaw = 0.0;
  }
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid (*transformed_cloud, centroid);
  transform_2.translation() << -1*centroid(0), -1*centroid(1), -1*centroid(2);
  // You can either apply transform_1 or transform_2; they are the same
  pcl::transformPointCloud (*source_cloud, *transformed_cloud, transform_2);

  float theta = 0; // The angle of rotation in radians

  // Print the transformation
  printf ("\nMethod #2: using an Affine3f\n");
  std::cout << transform_2.matrix() << std::endl;

  pcl::compute3DCentroid (*source_cloud, centroid);
  printf("center point = < %6.3f, %6.3f, %6.3f >\n", centroid(0)*100, centroid(1)*100, centroid(2)*100);

  if (pcl::console::find_switch (argc, argv, "-cut"))
  {
    // Create the filtering object
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (transformed_cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (-1.0, 0);
    //pass.setFilterLimitsNegative (true);
    pass.filter (*transformed_cloud);
  }


  float roll_,pitch_,yaw_;
  pcl::getEulerAngles(transform_2,roll_,pitch_,yaw_);
  std::cout << "roll_=" << roll_ << std::endl;
  std::cout << "pitch_=" << pitch_ << std::endl;
  std::cout << "yaw_=" << yaw_ << std::endl;

  // Executing the transformation


  std::cout << "Cloud Height = " << source_cloud->height << std::endl;
  std::cout << "Cloud Width = "  << source_cloud->width << std::endl;


  // Visualization
  printf(  "\nPoint cloud colors :  white  = original point cloud\n"
      "                        red  = transformed point cloud\n");
  pcl::visualization::PCLVisualizer viewer ("Matrix transformation example");

   // Define R,G,B colors for the point cloud
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler (source_cloud, 255, 255, 255);
  // We add the point cloud to the viewer and pass the color handler
  viewer.addPointCloud (source_cloud, source_cloud_color_handler, "original_cloud");

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_cloud_color_handler (transformed_cloud, 230, 20, 20); // Red
  viewer.addPointCloud (transformed_cloud, transformed_cloud_color_handler, "transformed_cloud");

  viewer.addCoordinateSystem (0.1, 0);
  viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed_cloud");
  //viewer.setPosition(800, 400); // Setting visualiser window position
  std::string out_file_name = argv[filenames[0]];
  out_file_name.insert(0,"trans_out_");
  pcl::io::savePCDFileASCII(out_file_name, *transformed_cloud); // saving the extracted colored cluster 
  while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
    viewer.spinOnce ();
  }

  return 0;
}
