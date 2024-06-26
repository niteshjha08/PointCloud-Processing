#include <preprocess_cloud.hpp>
#include <string>
using std::placeholders::_1;

PreprocessCloud::PreprocessCloud() : rclcpp::Node("preprocess_cloud")
{
  setup_parameters();

  _raw_cloud_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/carla/ego_vehicle/lidar", 10, std::bind(&PreprocessCloud::filter_callback, this, _1));

  _filtered_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/filtered_pointcloud", 10);
}

void PreprocessCloud::setup_parameters()
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

  this->declare_parameter("voxel_downsampling.active", true);
  this->get_parameter("voxel_downsampling.active", _voxel_downsampling);
  RCLCPP_INFO_STREAM(this->get_logger(), "Using parameter: 'voxel_downsampling.active' = " << _voxel_downsampling);
  if (_voxel_downsampling)
  {
    this->declare_parameter("voxel_downsampling.voxel_x", 0.05);
    this->get_parameter("voxel_downsampling.voxel_x", _voxel_x);

    this->declare_parameter("voxel_downsampling.voxel_y", 0.05);
    this->get_parameter("voxel_downsampling.voxel_y", _voxel_y);

    this->declare_parameter("voxel_downsampling.voxel_z", 0.05);
    this->get_parameter("voxel_downsampling.voxel_z", _voxel_z);

    RCLCPP_INFO_STREAM(this->get_logger(), "Using parameter: 'voxel_downsampling.voxel_x' = " << _voxel_x);
    RCLCPP_INFO_STREAM(this->get_logger(), "Using parameter: 'voxel_downsampling.voxel_y' = " << _voxel_y);
    RCLCPP_INFO_STREAM(this->get_logger(), "Using parameter: 'voxel_downsampling.voxel_z' = " << _voxel_z);
  }
}

void PreprocessCloud::range_based_filtering(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                                            pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud)
{
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

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
}

void PreprocessCloud::voxel_downsample(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                                       pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::VoxelGrid<pcl::PointXYZI> voxel_grid;
  voxel_grid.setInputCloud(cloud);
  voxel_grid.setLeafSize(_voxel_x, _voxel_y, _voxel_z);
  voxel_grid.filter(*filtered_cloud);
}

void PreprocessCloud::filter_callback(sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>);

  if (_range_based_filtering || _angle_based_filtering)
  {
    pcl::fromROSMsg(*msg, *cloud);
  }

  if (_range_based_filtering)
  {
    range_based_filtering(cloud, filtered_cloud);
    cloud = filtered_cloud;
  }

  // TODO:angle_based_filtering

  if (_voxel_downsampling)
  {
    voxel_downsample(filtered_cloud, cloud);
    cloud = filtered_cloud;
  }

  // Publish the filtered cloud
  sensor_msgs::msg::PointCloud2 out_msg;

  pcl::toROSMsg(*filtered_cloud, out_msg);

  _filtered_cloud_pub->publish(out_msg);
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PreprocessCloud>());
  rclcpp::shutdown();
  return 0;
}