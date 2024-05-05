#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <vector>
#include <pcl/segmentation/extract_clusters.h>
// #include "point_cloud_proc/ArrayPointCloud.msg"
#include "point_cloud_proc/msg/array_point_cloud.hpp"
#include <string>
#include <Eigen/Dense>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
// #include <msg/ArrayPointCloud.msg>
class Clustering : public rclcpp::Node
{
public:
  Clustering();

private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr _cloud_sub;

  rclcpp::Subscription<point_cloud_proc::msg::ArrayPointCloud>::SharedPtr _clusters_visualization_sub, _bounding_box_sub;

  rclcpp::Publisher<point_cloud_proc::msg::ArrayPointCloud>::SharedPtr _cluster_clouds_pub;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr _bounding_box_pub;


  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _cluster_visualization_pub;

  void cluster_callback(sensor_msgs::msg::PointCloud2::SharedPtr);

  void euclidean_clustering(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                            std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>& clusters);

  void visualize_clusters(point_cloud_proc::msg::ArrayPointCloud::SharedPtr clusters_msg);

  void publish_bounding_boxes(point_cloud_proc::msg::ArrayPointCloud::SharedPtr clusters_msg);

  void setup_parameters();

  std::string _cluster_method;
  double _cluster_tolerance;
  int _min_cluster_size, _max_cluster_size;
  bool _visualize_clusters;

  struct Bbox;

  Bbox get_cluster_bounding_box(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster);
};