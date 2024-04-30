#include <iostream>
#include <math.h>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/filters/extract_indices.h>
// #include <pcl_ros/transforms.h>

class FilterCloud : public rclcpp::Node
{
public:
  FilterCloud();

private:
  // subscriber of raw point cloud
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr _raw_cloud_sub;
  // publisher of filtered point cloud
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _filtered_cloud_pub;
  // filter callback function
  void filter_callback(sensor_msgs::msg::PointCloud2::SharedPtr);

  void range_based_filtering(pcl::PointCloud<pcl::PointXYZI>::Ptr);

  void setup_parameters();

  bool _range_based_filtering;

  bool _angle_based_filtering;

  double _range_limit;

  double _angle_range_limit;
};