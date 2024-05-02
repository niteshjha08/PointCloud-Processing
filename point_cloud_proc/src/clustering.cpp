#include <clustering.hpp>
using std::placeholders::_1;

Clustering::Clustering() : rclcpp::Node("clustering")
{
  _cloud_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/nonplane_cloud", 10, std::bind(&Clustering::cluster_callback, this, _1));

  _cluster_clouds_pub = this->create_publisher<point_cloud_proc::msg::ArrayPointCloud>("/cluster_clouds", 10);
}

void Clustering::cluster_callback(sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg)
{ 
  RCLCPP_INFO_STREAM(this->get_logger(), "starting clustering");
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*cloud_msg, *cloud);

  //   pcl::PointCloud<pcl::PointXYZI>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloud_clusters;
  euclidean_clustering(cloud, cloud_clusters);

  // 1. Form array of pointcloud msg
  point_cloud_proc::msg::ArrayPointCloud cluster_array;
  for (auto cluster : cloud_clusters)
  {
    sensor_msgs::msg::PointCloud2 cluster_msg;
    pcl::toROSMsg(*cluster, cluster_msg);
    // 2. Load clusters into it
    cluster_array.clouds.push_back(cluster_msg);
  }
  _cluster_clouds_pub->publish(cluster_array);
  RCLCPP_INFO_STREAM(this->get_logger(), "publishing ");

  // 3. Publish
}

void Clustering::euclidean_clustering(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                                      std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>& clusters)
{
  pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
  tree->setInputCloud(cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
  ec.setClusterTolerance(0.02);  // 2cm
  ec.setMinClusterSize(100);
  ec.setMaxClusterSize(25000);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud);
  ec.extract(cluster_indices);

  for (const auto& cluster : cluster_indices)
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZI>);
    for (const auto& idx : cluster.indices)
    {
      cloud_cluster->push_back((*cloud)[idx]);
    }
    cloud_cluster->width = cloud_cluster->size();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;
    cloud_cluster->header.frame_id="";
    clusters.push_back(cloud_cluster);
  }
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Clustering>());
  rclcpp::shutdown();
  return 0;
}