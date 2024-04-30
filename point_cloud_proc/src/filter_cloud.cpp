#include <filter_cloud.hpp>
#include <string>
using std::placeholders::_1;
/*
1. Param file to load if range_filter is on, range_filter distance
2. Angle filter
*/
FilterCloud::FilterCloud() : rclcpp::Node("filter_cloud")
{
  setup_parameters();

  _raw_cloud_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/carla/ego_vehicle/lidar", 10, std::bind(&FilterCloud::filter_callback, this, _1));

  _filtered_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/filtered_pointcloud", 10);
}

void FilterCloud::setup_parameters()
{
  this->declare_parameter("range_based_filtering.active", true);
  this->get_parameter("range_based_filtering.active", _range_based_filtering);
  RCLCPP_INFO_STREAM(this->get_logger(),
                     "Using parameter: 'range_based_filtering.active' = " << _range_based_filtering);

  if (_range_based_filtering)
  {
    this->declare_parameter("range_based_filtering.range", 0.0);
    this->get_parameter("range_based_filtering.range", _range_limit);
    RCLCPP_INFO_STREAM(this->get_logger(), "Using parameter: 'range_based_filtering.range' = " << _range_limit);
  }

  this->declare_parameter("angle_based_filtering.active", true);
  this->get_parameter("angle_based_filtering.active", _angle_based_filtering);
  RCLCPP_INFO_STREAM(this->get_logger(),
                     "Using parameter: 'angle_based_filtering.active' = " << _angle_based_filtering);

  if (_angle_based_filtering)
  {
    this->declare_parameter("angle_based_filtering.range", 0.0);
    this->get_parameter("angle_based_filtering.range", _angle_range_limit);
    RCLCPP_INFO_STREAM(this->get_logger(), "Using parameter: 'angle_based_filtering.range' = " << _angle_range_limit);
  }
}

void FilterCloud::range_based_filtering(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>);

  for (unsigned int i = 0; i < cloud->points.size(); ++i)
  {
    auto point = cloud->points[i];
    if (point.x * point.x + point.y * point.y + point.z * point.z < _range_limit)
    {
      inliers->indices.push_back(i);
    }
  }
  pcl::ExtractIndices<pcl::PointXYZI> extract;
  extract.setInputCloud(cloud);

  extract.setIndices(inliers);

  extract.setNegative(false);
  extract.filter(*filtered_cloud);

  sensor_msgs::msg::PointCloud2 out_msg;

  pcl::toROSMsg(*filtered_cloud, out_msg);

  _filtered_cloud_pub->publish(out_msg);
}

void FilterCloud::filter_callback(sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  if (_range_based_filtering || _angle_based_filtering)
  {
    pcl::fromROSMsg(*msg, *cloud);
  }

  if (_range_based_filtering)
  {
    range_based_filtering(cloud);
  }
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FilterCloud>());
  rclcpp::shutdown();
  return 0;
}