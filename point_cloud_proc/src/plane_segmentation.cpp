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

  if (_plane_segmentation)
  {
    this->declare_parameter("clip_ground.height", 0.05);
    this->get_parameter("clip_ground.height", _clip_height);
    RCLCPP_INFO_STREAM(this->get_logger(), "Using parameter: 'clip_ground.height' = " << _clip_height);
  }
}

void PlaneSegmentation::clip_upto_height(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                                         pcl::PointCloud<pcl::PointXYZI>::Ptr clipped_cloud)
{
  pcl::PointIndices::Ptr clip_inliers(new pcl::PointIndices());

  for (unsigned int i = 0; i < cloud->points.size(); ++i)
  {
    // This clipping is w.r.t lidar frame which is on top of vehicle. 
    // TODO: Find the transform and then clip
    auto point = cloud->points[i];
    if (point.z < _clip_height)
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
    RCLCPP_INFO_STREAM(this->get_logger(), "Model coefficients = " << coefficients->values.size());


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