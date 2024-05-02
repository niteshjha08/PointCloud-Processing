#include <iostream>
#include <plane_segmentation.hpp>
using std::placeholders::_1;

PlaneSegmentation::PlaneSegmentation() : rclcpp::Node("plane_segmentation")
{
  setup_parameters();
  if (_plane_segmentation)
  {
    _pointcloud_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/filtered_pointcloud", 10, std::bind(&PlaneSegmentation::plane_segment, this, _1));

    _plane_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/plane_cloud", 10);
    _nonplane_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/nonplane_cloud", 10);
  }

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  // Assuming vehicle -> LiDAR is static, lookup only once
  try
  {
    _lidar_to_body_transform = tf_buffer_->lookupTransform(_vehicle_body_frame, _lidar_frame, tf2::TimePointZero,
                                                           std::chrono::milliseconds(2000));
  }
  catch (const tf2::TransformException& ex)
  {
    RCLCPP_INFO_STREAM(this->get_logger(),
                       "Could not transform " << _vehicle_body_frame << " to " << _lidar_frame << ex.what());
    RCLCPP_INFO_STREAM(this->get_logger(), "Turning off groung clipping ");
    _clip_ground = false;
  }
  RCLCPP_INFO_STREAM(this->get_logger(), "Initialized node");
}
void PlaneSegmentation::setup_parameters()
{
  this->declare_parameter("plane_segmentation.active", true);
  this->get_parameter("plane_segmentation.active", _plane_segmentation);
  RCLCPP_INFO_STREAM(this->get_logger(), "Using parameter: 'plane_segmentation.active' = " << _plane_segmentation);

  if (_plane_segmentation)
  {
    this->declare_parameter("plane_segmentation.distance_threshold", 0.05);
    this->get_parameter("plane_segmentation.distance_threshold", _plane_dist_thresh);
    RCLCPP_INFO_STREAM(this->get_logger(),
                       "Using parameter: 'plane_segmentation.distance_threshold' = " << _plane_dist_thresh);
  }
  this->declare_parameter("clip_ground.active", true);
  this->get_parameter("clip_ground.active", _clip_ground);
  RCLCPP_INFO_STREAM(this->get_logger(), "Using parameter: 'clip_ground.active' = " << _clip_ground);

  if (_clip_ground)
  {
    this->declare_parameter("clip_ground.height", 0.05);
    this->get_parameter("clip_ground.height", _clip_height);
    RCLCPP_INFO_STREAM(this->get_logger(), "Using parameter: 'clip_ground.height' = " << _clip_height);
  }
  this->declare_parameter("vehicle_body_frame", "");
  this->get_parameter("vehicle_body_frame", _vehicle_body_frame);
  RCLCPP_INFO_STREAM(this->get_logger(), "Using parameter: 'vehicle_body_frame' = " << _vehicle_body_frame);

  this->declare_parameter("lidar_frame", "");
  this->get_parameter("lidar_frame", _lidar_frame);
  RCLCPP_INFO_STREAM(this->get_logger(), "Using parameter: 'lidar_frame' = " << _lidar_frame);
}

void PlaneSegmentation::clip_upto_height(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                                         pcl::PointCloud<pcl::PointXYZI>::Ptr clipped_cloud)
{
  pcl::PointIndices::Ptr clip_inliers(new pcl::PointIndices());

  for (unsigned int i = 0; i < cloud->points.size(); ++i)
  {
    auto point = cloud->points[i];
    geometry_msgs::msg::Point input_pt, output_pt;
    input_pt.x = point.x;
    input_pt.y = point.y;
    input_pt.z = point.z;

    tf2::doTransform(input_pt, output_pt, _lidar_to_body_transform);
    // transform point to body frame
    if (output_pt.z < _clip_height)
    {
      clip_inliers->indices.push_back(i);
    }
  }
  pcl::ExtractIndices<pcl::PointXYZI> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(clip_inliers);

  extract.setNegative(true);
  extract.filter(*clipped_cloud);
}

void PlaneSegmentation::plane_segment(sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*cloud_msg, *cloud);
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
  pcl::SACSegmentation<pcl::PointXYZI> seg;

  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(_plane_dist_thresh);

  seg.setInputCloud(cloud);
  seg.segment(*inliers, *coefficients);

  pcl::ExtractIndices<pcl::PointXYZI> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(inliers);

  extract.setNegative(false);
  pcl::PointCloud<pcl::PointXYZI>::Ptr plane_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  extract.filter(*plane_cloud);

  sensor_msgs::msg::PointCloud2 plane_cloud_msg;
  pcl::toROSMsg(*plane_cloud, plane_cloud_msg);

  _plane_cloud_pub->publish(plane_cloud_msg);

  extract.setNegative(true);
  pcl::PointCloud<pcl::PointXYZI>::Ptr nonplane_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  extract.filter(*nonplane_cloud);
  if (_clip_ground)
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr clipped_nonplane_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    clip_upto_height(nonplane_cloud, clipped_nonplane_cloud);
    nonplane_cloud = clipped_nonplane_cloud;
  }

  sensor_msgs::msg::PointCloud2 nonplane_cloud_msg;
  pcl::toROSMsg(*nonplane_cloud, nonplane_cloud_msg);
  _nonplane_cloud_pub->publish(nonplane_cloud_msg);
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlaneSegmentation>());
  rclcpp::shutdown();
  return 0;
}