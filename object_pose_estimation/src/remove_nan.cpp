#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/filter.h>
// #include <pcl/filters/uniform_sampling.h>
// //#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

std::string model_filename_;
std::string scene_filename_;
typedef pcl::PointXYZRGBNormal PointTypeN;
typedef pcl::PointXYZRGBA PointType;

int main (int argc, char *argv[])
{
    pcl::PointCloud<PointType>::Ptr model (new pcl::PointCloud<PointType> ());
    pcl::PointCloud<PointTypeN>::Ptr scene (new pcl::PointCloud<PointTypeN> ());
    
    std::vector<int> filenames;
    filenames = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");
    model_filename_ = argv[filenames[0]];

    if (pcl::io::loadPCDFile (model_filename_, *model) < 0)
    {
        std::cout << "Error loading model cloud." << std::endl;
        return (-1);
    }
    pcl::copyPointCloud(*model, *scene);
    
    std::cerr << "scene before filtering: " << scene->width * scene->height << std::endl;
    
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*scene,*scene, indices);
    std::cerr << "scene after filtering: " << scene->width * scene->height << std::endl;
    
    pcl::PCDWriter pcd_saver;
    std::stringstream ss;
    ss << model_filename_.c_str() << "_remove_NAN" << ".pcd";
    pcd_saver.write<PointTypeN> (ss.str (), *scene, false);
}
