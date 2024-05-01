#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/extract_indices.h>

class PlaneSegmentation : public rclcpp::Node
{
public:
  PlaneSegmentation();

private:
  void setup_parameters();

  void plane_segment(sensor_msgs::msg::PointCloud2::SharedPtr cloud);

  void clip_upto_height(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                                           pcl::PointCloud<pcl::PointXYZI>::Ptr clipped_cloud);

      rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr _pointcloud_sub;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _plane_cloud_pub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _nonplane_cloud_pub;

  bool _plane_segmentation;
  double _plane_dist_thresh;

  bool _clip_ground;
  double _clip_height;
};