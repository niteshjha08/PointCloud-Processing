#include <iostream>
#include <math.h>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>

class PreprocessCloud : public rclcpp::Node
{
public:
  PreprocessCloud();

private:
  // subscriber of raw point cloud
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr _raw_cloud_sub;
  // publisher of filtered point cloud
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _filtered_cloud_pub;
  // filter callback function
  void filter_callback(sensor_msgs::msg::PointCloud2::SharedPtr);

  void range_based_filtering(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                             pcl::PointCloud<pcl::PointXYZI>::Ptr filter_cloud);

  void voxel_downsample(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr out);

  void setup_parameters();

  bool _range_based_filtering;
  double _range_limit;

  bool _angle_based_filtering;
  double _angle_range_limit;

  bool _voxel_downsampling;
  double _voxel_x, _voxel_y, _voxel_z;
};