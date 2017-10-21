#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

std::string model_filename_;
std::string scene_filename_;
typedef pcl::PointXYZRGBA PointType;

int main (int argc, char *argv[])
{
    pcl::PointCloud<PointType>::Ptr model (new pcl::PointCloud<PointType> ());
    std::vector<int> filenames;
    filenames = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");
    model_filename_ = argv[filenames[0]];

    if (pcl::io::loadPCDFile (model_filename_, *model) < 0)
    {
        std::cout << "Error loading model cloud." << std::endl;
        return (-1);
    }
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*model,*model, indices);

    pcl::PCDWriter pcd_saver;
    std::stringstream ss;
    ss << model_filename_.c_str() << "_remove_NAN" << ".pcd";
    pcd_saver.write<PointType> (ss.str (), *model, false);
}