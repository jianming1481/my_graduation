#include <iostream>
#include <string>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>

class ICP_alignment  
{  
    public:  
  
    ICP_alignment ()  
    {  
       // Intialize the parameters in the ICP algorithm  
       //icp.setMaxCorrespondenceDistance(0.01);  
       //icp.setTransformationEpsilon(1e-7);  
       //icp.setEuclideanFitnessEpsilon(1);  
       icp.setMaximumIterations(50);  
    }  
  
    ~ICP_alignment () {}  
  
    void print4x4Matrix (const Eigen::Matrix4f & matrix)
    {
        float roll,pitch,yaw;
        printf ("Show Rotation matrix from ICP\n");
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
    
     void setSourceCloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr source_cloud)  
    {  
        std::cout << "set source (model)!" << std::endl;
        icp.setInputCloud(source_cloud);  
    }  
  
    void setTargetCloud (pcl::PointCloud<pcl::PointXYZ>::ConstPtr target_cloud)  
    {  
        std::cout << "set target (scene)!" << std::endl;
        icp.setInputTarget(target_cloud);  
    }  
      
    // Align the given template cloud to the target specified by setTargetCloud ()  
    void align (pcl::PointCloud<pcl::PointXYZ> &temp)  
    {  
        
      pcl::PointCloud<pcl::PointXYZ> registration_output;  
      icp.align (temp);  
  
      fitness_score =  icp.getFitnessScore();
      final_transformation = Eigen::Matrix4f::Identity ();
      final_transformation = icp.getFinalTransformation ();  
      print4x4Matrix(final_transformation);
    }  
  
    float getScore()  
    {  
        return fitness_score;  
    }  
  
    Eigen::Matrix4f getMatrix()  
    {  
        return final_transformation;  
    }  
  private:  
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;  
    Eigen::Matrix4f final_transformation;  
    float fitness_score;  
};  
