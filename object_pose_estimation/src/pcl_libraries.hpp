#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/time.h>
#include <pcl/console/print.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl_ros/transforms.h>
#include <pcl/console/parse.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/surface/mls.h>
#include <pcl/kdtree/kdtree_flann.h>


// Types
typedef pcl::PointXYZRGBA PT;                   // Point Type with XYZ and color
typedef pcl::PointCloud<PT> PCT;                // Point Cloud Type
typedef pcl::PointXYZRGBNormal PointNT;               // Point Type with Normal
typedef pcl::PointCloud<PointNT> PointCloudT;   // Point Cloud Type with Normal

typedef pcl::FPFHSignature33 FeatureT;          // Feature type with FPFH Signature
typedef pcl::FPFHEstimationOMP<PointNT,PointNT,FeatureT> FeatureEstimationT;
typedef pcl::PointCloud<FeatureT> FeatureCloudT;// Feature Cloud Type with FPFH Signature
typedef pcl::visualization::PointCloudColorHandlerCustom<PointNT> ColorHandlerT;

// Define to Save Point Cloud Data
#define SaveCloud