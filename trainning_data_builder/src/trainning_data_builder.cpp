#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <boost/foreach.hpp>
#include <sensor_msgs/Image.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <iostream>
#include <iomanip>
#include <ctime>
#include <ros/package.h>
//pcl::toROSMsg
#include <pcl/io/pcd_io.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// Include rosservice
#include "trainning_data_builder/data.h"

//typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointXYZRGB PT;           //Point Type
typedef pcl::PointCloud<PT> PCT;


class PointCloud
{
public:

  PointCloud()
  {
    ROS_INFO("initialize...");

    cloud = PCT::Ptr (new PCT);

    point_sub_ = nh_.subscribe("/camera/depth_registered/points", 1, &PointCloud::PointCallback, this);
    image_sub_ = nh_.subscribe("/camera/rgb/image_color", 1, &PointCloud::ImageCallback, this);
    rotate_platfrom_client = nh_.serviceClient<trainning_data_builder::data>("/trainning_data_builder/save");

    tmpCount_ = 1;

    cv::namedWindow("view");
    ROS_INFO("OK");
  }

  ~PointCloud()
  {
    cv::destroyWindow("view");
  }

  void ImageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      
      cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
      cv::waitKey(1);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    image = cv_ptr->image;
  }

  void PointCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
  {
    if ((msg->width * msg->height) == 0)
      return; //return if the cloud is not dense!
    try
    {
      pcl::fromROSMsg(*msg, *cloud);
    }
    catch (std::runtime_error e)
    {
      ROS_ERROR_STREAM("Error in converting cloud to image message: "
                        << e.what());
    }
  }

  void save()
  {
    ROS_INFO("Save Pcd and Image : training_data_%05d", tmpCount_);
    std::stringstream s1;

    // Save Point Cloud 
    s1 << ros::package::getPath("trainning_data_builder") << "/pattern/pcd_file/training_data_"
      << std::setfill('0') << std::setw(5) << tmpCount_ << ".pcd";
    writer.write<PT> (s1.str(), *cloud, false);

    s1.str("");
    s1.clear();

    // Save Image
    s1 << ros::package::getPath("trainning_data_builder") << "/pattern/ori_img/training_data_"
      << std::setfill('0') << std::setw(5) << tmpCount_ << ".jpg";
    cv::imwrite(s1.str(), image);

    s1.str("");
    s1.clear();

    tmpCount_++;
  }

  void build_data(int label_number)
  {
    if ((cloud->width * cloud->height) == 0)
    {
      ROS_ERROR("Empty Cloud!");
      return; //return if the cloud is not dense!
    }
    std::vector<int> index;
    PCT::Ptr tmp_cloud(new PCT);
    int tmp_x;
    int tmp_y;
    int* data = new int[480*640];
    for(int i=0;i<480*640;i++)
    {
      data[i]=0;
    }
    cv::Mat tmp_data(480,640, CV_8U, data);
    pcl::removeNaNFromPointCloud(*cloud, *tmp_cloud, index);
    ROS_INFO("size of index = %d",index.size());
    for(int i=0;i<index.size();i++)
    {
      tmp_y = index[i]%cloud->width+1;
      tmp_x = index[i]/cloud->width+1;
      tmp_data.at<uchar>(tmp_x,tmp_y)=label_number;
    }

    ROS_INFO("Doing dilate!");
    dilate(tmp_data,tmp_data, cv::Mat());
    ROS_INFO("Doing erode!");
    erode(tmp_data,tmp_data, cv::Mat());
    erode(tmp_data,tmp_data, cv::Mat());
    erode(tmp_data,tmp_data, cv::Mat());
    dilate(tmp_data,tmp_data, cv::Mat());
    dilate(tmp_data,tmp_data, cv::Mat());

    ROS_INFO("Save trainning data : training_data_%05d", tmpCount_);

    std::stringstream s1;
    s1 << ros::package::getPath("trainning_data_builder") << "/pattern/label_img/training_data"
       << "_gt_" << std::setfill('0') << std::setw(5) << tmpCount_ << ".jpg";
    cv::imwrite( s1.str(), tmp_data );

    s1.str("");
    s1.clear();
    index.clear();
    tmp_data.release();
    delete [] data;
  }

  void call_server()
  {
    trainning_data_builder::data test;
    test.request.str = "test";
    rotate_platfrom_client.call(test);
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber point_sub_;
  ros::Subscriber image_sub_;
  ros::ServiceClient rotate_platfrom_client;

  // PCT cloud;
  PCT::Ptr cloud;
  pcl::PCDWriter writer;
  cv::Mat image;

  int tmpCount_;
};

int main(int argc, char** argv)
{
  int label_number = 50;
  ros::init(argc, argv, "trainning_data_builder");
  PointCloud pc;
  ros::Rate loop_rate(30);
  while(ros::ok())
  {
    if(cv::waitKey(30)==13)
    {
      for(int i=0;i<40;i++)
      {
        pc.save();
        pc.build_data(label_number);
        pc.call_server();
        ros::spinOnce();
        loop_rate.sleep();
      }
      label_number++;
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
