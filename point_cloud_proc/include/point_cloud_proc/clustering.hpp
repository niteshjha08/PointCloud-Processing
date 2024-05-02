#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <vector>
#include <pcl/segmentation/extract_clusters.h>
// #include "point_cloud_proc/ArrayPointCloud.msg"
#include "point_cloud_proc/msg/array_point_cloud.hpp"
// #include <msg/ArrayPointCloud.msg>
class Clustering : public rclcpp::Node
{
public:
  Clustering();

private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr _cloud_sub;

  rclcpp::Publisher<point_cloud_proc::msg::ArrayPointCloud>::SharedPtr _cluster_clouds_pub;

  void cluster_callback(sensor_msgs::msg::PointCloud2::SharedPtr);

  void euclidean_clustering(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                            std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>& clusters);
};