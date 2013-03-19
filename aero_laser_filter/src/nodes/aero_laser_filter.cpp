
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include<pcl/ros/conversions.h>
#include<pcl/filters/crop_box.h>
#include<boost/shared_ptr.hpp>

class My_Filter {
private:
  typedef pcl::PointXYZ            Point_t;
  typedef pcl::PointCloud<Point_t> PointCloud_t;
  typedef boost::shared_ptr<PointCloud_t> PointCloudPtr_t;
public:
  My_Filter(ros::NodeHandle& nh);
private:
  ros::NodeHandle node_;
  
  laser_geometry::LaserProjection projector_;
  
  tf::TransformListener tfListener_;
  
  ros::Publisher local_cloud_publisher_;
  ros::Publisher global_cloud_publisher_;
  ros::Subscriber scan_sub_;
  
  pcl::CropBox<Point_t> local_crop_box_;
  pcl::CropBox<Point_t> global_crop_box_;
  
  std::string local_frame_;
  int local_x_size_;
  int local_y_size_;
  int local_z_size_;
  std::string global_frame_;
  int global_x_size_;
  int global_y_size_;
  int global_z_size_;
  
  void loadParams();
  
  void setUpCrops();
	
  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
	
  void filterGlobal(const PointCloudPtr_t& in, PointCloudPtr_t& out);
  
  void filterLocal(const PointCloudPtr_t& in, PointCloudPtr_t& out);
};

My_Filter::My_Filter(ros::NodeHandle& nh):
   node_(nh)
{
  this->loadParams();
  this->setUpCrops();
  this->scan_sub_               = node_.subscribe<sensor_msgs::LaserScan> ("/scan", 2, &My_Filter::scanCallback, this);
  this->local_cloud_publisher_  = node_.advertise<sensor_msgs::PointCloud2> ("aero/local/laser", 2, false);
  this->global_cloud_publisher_ = node_.advertise<sensor_msgs::PointCloud2> ("aero/global/laser", 2, false);
//        tfListener_.setExtrapolationLimit(ros::Duration(0.1));
}

void My_Filter::loadParams()
{
  local_x_size_ = 10;
  local_y_size_ = 10;
  local_z_size_ = 0;
  
  global_x_size_ = 50;
  global_y_size_ = 50;
  global_z_size_ = 0;
}

void My_Filter::setUpCrops()
{
  Point_t local_start;
  local_start.x = 0;
  local_start.y = 0;
  local_start.z = 0;
  Point_t local_end;
  local_end.x   = this->local_x_size_;
  local_end.y   = this->local_y_size_;
  local_end.z   = this->local_z_size_;
  this->local_crop_box_.setMin(local_start.getVector4fMap());
  this->local_crop_box_.setMax(local_end.getVector4fMap());
  
  Point_t global_start;
  global_start.x = 0;
  global_start.y = 0;
  global_start.z = 0;
  Point_t global_end;
  global_end.x   = this->global_x_size_;
  global_end.y   = this->global_y_size_;
  global_end.z   = this->global_z_size_;
  this->global_crop_box_.setMin(global_start.getVector4fMap());
  this->global_crop_box_.setMax(global_end.getVector4fMap());
}

void My_Filter::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan){
    sensor_msgs::PointCloud2 raw_cloud_msg;
    sensor_msgs::PointCloud2 local_cloud_msg;
    sensor_msgs::PointCloud2 global_cloud_msg;
    PointCloudPtr_t scale_cloud(new PointCloud_t());
    PointCloudPtr_t local_cloud(new PointCloud_t());
    PointCloudPtr_t global_cloud(new PointCloud_t());
    projector_.projectLaser(*scan, raw_cloud_msg);
    pcl::fromROSMsg(raw_cloud_msg, *scale_cloud);
    
    for(int i=0; i<scale_cloud->size(); i++)
    {
      pcl::PointXYZ& point = scale_cloud->at(i);
      point.x = point.x;
      point.y = point.y;
      point.z = std::floor(point.z);
    }
    
    this->filterGlobal(scale_cloud, global_cloud);
    this->filterLocal(scale_cloud,  local_cloud);
    
    pcl::toROSMsg(*global_cloud, global_cloud_msg);
    pcl::toROSMsg(*local_cloud,  local_cloud_msg);
    
    this->local_cloud_publisher_.publish(local_cloud_msg);
    this->global_cloud_publisher_.publish(global_cloud_msg);
    
}

void My_Filter::filterGlobal(const PointCloudPtr_t& in, PointCloudPtr_t& out)
{
  this->global_crop_box_.setInputCloud(in);
  this->global_crop_box_.filter(*out);
}

void My_Filter::filterLocal(const PointCloudPtr_t& in, PointCloudPtr_t& out)
{
  this->local_crop_box_.setInputCloud(in);
  this->local_crop_box_.filter(*out);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "aero_laser_filter");
    ros::NodeHandle nh;
    My_Filter filter(nh);

    ros::spin();

    return 0;
}

