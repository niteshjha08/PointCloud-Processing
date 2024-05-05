#include <clustering.hpp>
#include <utils.cpp>
using std::placeholders::_1;
struct Clustering::Bbox
{
  double x_min;
  double y_min;
  double z_min;
  double x_max;
  double y_max;
  double z_max;
};

Clustering::Clustering() : rclcpp::Node("clustering")
{
  _cloud_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/nonplane_cloud", 10, std::bind(&Clustering::cluster_callback, this, _1));

  _cluster_clouds_pub = this->create_publisher<point_cloud_proc::msg::ArrayPointCloud>("/cluster_clouds", 10);

  _clusters_visualization_sub = this->create_subscription<point_cloud_proc::msg::ArrayPointCloud>(
      "/cluster_clouds", 10, std::bind(&Clustering::visualize_clusters, this, _1));

  _cluster_visualization_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cluster_visualization", 10);

  _bounding_box_sub = this->create_subscription<point_cloud_proc::msg::ArrayPointCloud>(
      "/cluster_clouds", 10, std::bind(&Clustering::publish_bounding_boxes, this, _1));

  _bounding_box_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("/cluster_bounding_boxes", 10);

  setup_parameters();
}

void Clustering::cluster_callback(sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg)
{
  // RCLCPP_INFO_STREAM(this->get_logger(), "starting clustering");
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*cloud_msg, *cloud);

  // TODO: remove intermediate datatype, make cloud clusters of ArrayPointCloud directly and pass.
  // In euclidean_clustering, convert clusters to ROS type and append
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloud_clusters;
  if (_cluster_method == "euclidean")
  {
    euclidean_clustering(cloud, cloud_clusters);
  }

  // 1. Form array of pointcloud msg
  point_cloud_proc::msg::ArrayPointCloud cluster_array;
  for (auto cluster : cloud_clusters)
  {
    sensor_msgs::msg::PointCloud2 cluster_msg;
    pcl::toROSMsg(*cluster, cluster_msg);
    // 2. Load clusters into it
    cluster_array.clouds.push_back(cluster_msg);
  }
  cluster_array.header = cloud_msg->header;
  _cluster_clouds_pub->publish(cluster_array);
}

void Clustering::euclidean_clustering(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                                      std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>& clusters)
{
  pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
  RCLCPP_INFO_STREAM(this->get_logger(), "Length of cloud input: " << cloud->points.size());

  tree->setInputCloud(cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
  ec.setClusterTolerance(_cluster_tolerance);
  ec.setMinClusterSize(_min_cluster_size);
  ec.setMaxClusterSize(_max_cluster_size);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud);
  ec.extract(cluster_indices);
  RCLCPP_INFO_STREAM(this->get_logger(), "Number of clusters: " << cluster_indices.size());

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
    cloud_cluster->header = cloud->header;
    clusters.push_back(cloud_cluster);
  }
}

void Clustering::publish_bounding_boxes(point_cloud_proc::msg::ArrayPointCloud::SharedPtr clusters_msg)
{
  int i = 0;
  auto marker_array = visualization_msgs::msg::MarkerArray();
  for (auto cluster : clusters_msg->clouds)
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(cluster, *cloud);
    Bbox bbox = get_cluster_bounding_box(cloud);
    auto marker = visualization_msgs::msg::Marker();
    marker.id = i;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    // Set the position and size of the cube (bounding box)
    marker.pose.position.x = (bbox.x_min + bbox.x_max) / 2.0;
    marker.pose.position.y = (bbox.y_min + bbox.y_max) / 2.0;
    marker.pose.position.z = (bbox.z_min + bbox.z_max) / 2.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = bbox.x_max - bbox.x_min;
    marker.scale.y = bbox.y_max - bbox.y_min;
    marker.scale.z = bbox.z_max - bbox.z_min;

    // Set the color (green)
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0f;
    marker.header = clusters_msg->header;
    marker_array.markers.push_back(marker);
    i += 1;
  }
  _bounding_box_pub->publish(marker_array);
}

Clustering::Bbox Clustering::get_cluster_bounding_box(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster)
{
  Eigen::Matrix<double, Eigen::Dynamic, 2> cluster_points(cluster->points.size(), 2);
  Eigen::Vector<double, Eigen::Dynamic> z_coordinates(cluster->points.size());

  for (int i = 0; i < cluster->points.size(); ++i)
  {
    cluster_points.row(i) = Eigen::RowVector2d(cluster->points[i].x, cluster->points[i].y);
    z_coordinates(i) = cluster->points[i].z;
  }

  Eigen::MatrixXd centered = cluster_points.colwise() - cluster_points.rowwise().mean();

  Eigen::MatrixXd cov = (centered * centered.adjoint()) / double(cluster_points.cols() - 1);

  Eigen::EigenSolver<Eigen::MatrixXd> es(cov);

  Eigen::MatrixXd aligned_coords = es.eigenvectors().real().transpose() * centered;

  Eigen::Vector2d min_xy = aligned_coords.colwise().minCoeff();

  Eigen::Vector2d max_xy = aligned_coords.colwise().maxCoeff();
  Eigen::Vector2d z_extremes;

  z_extremes << z_coordinates.minCoeff(), z_coordinates.maxCoeff();

  Bbox bbox;
  bbox.x_max = max_xy[0];
  bbox.x_min = min_xy[0];
  bbox.y_max = max_xy[1];
  bbox.y_min = min_xy[1];
  bbox.z_max = z_extremes[1];
  bbox.z_min = z_extremes[0];
  RCLCPP_INFO_STREAM(this->get_logger(), "Done: ");

  return bbox;
}

void Clustering::visualize_clusters(point_cloud_proc::msg::ArrayPointCloud::SharedPtr clusters_msg)
{
  pcl::PointCloud<pcl::PointXYZRGB> aggregate_cloud;
  int num_clouds = clusters_msg->clouds.size();
  RCLCPP_INFO_STREAM(this->get_logger(), "Number of clusters for visualization: " << num_clouds);
  if (num_clouds == 0)
    return;
  std::vector<std::vector<int>> cluster_colors = get_cluster_colors(num_clouds);
  int cluster_idx = 0;
  for (auto cluster_ros : clusters_msg->clouds)
  {
    pcl::PointCloud<pcl::PointXYZI> cluster;
    pcl::fromROSMsg(cluster_ros, cluster);
    std::vector<int> colors = cluster_colors[cluster_idx];
    RCLCPP_INFO_STREAM(this->get_logger(),
                       "Cluster: " << cluster_idx << " color: " << colors[0] << "," << colors[1] << "," << colors[2]);

    for (auto point : cluster)
    {
      pcl::PointXYZRGB colored_point;
      colored_point.x = point.x;
      colored_point.y = point.y;
      colored_point.z = point.z;
      colored_point.r = colors[0];
      colored_point.g = colors[1];
      colored_point.b = colors[2];
      aggregate_cloud.points.push_back(colored_point);
    }
    cluster_idx += 1;
  }

  sensor_msgs::msg::PointCloud2 agg_cluster_msg;
  pcl::toROSMsg(aggregate_cloud, agg_cluster_msg);
  agg_cluster_msg.header.frame_id = clusters_msg->header.frame_id;
  agg_cluster_msg.header.stamp = clusters_msg->header.stamp;
  _cluster_visualization_pub->publish(agg_cluster_msg);
}

void Clustering::setup_parameters()
{
  this->declare_parameter("cluster_method", "None");
  this->get_parameter("cluster_method", _cluster_method);
  RCLCPP_INFO_STREAM(this->get_logger(), "Using parameter: 'cluster_method' = " << _cluster_method);

  if (_cluster_method == "euclidean")
  {
    this->declare_parameter("euclidean_clustering.cluster_tolerance", 0.1);
    this->get_parameter("euclidean_clustering.cluster_tolerance", _cluster_tolerance);
    RCLCPP_INFO_STREAM(this->get_logger(),
                       "Using parameter: 'euclidean_clustering.cluster_tolerance' = " << _cluster_tolerance);

    this->declare_parameter("euclidean_clustering.min_cluster_size", 5);
    this->get_parameter("euclidean_clustering.min_cluster_size", _min_cluster_size);
    RCLCPP_INFO_STREAM(this->get_logger(),
                       "Using parameter: 'euclidean_clustering.min_cluster_size' = " << _min_cluster_size);

    this->declare_parameter("euclidean_clustering.max_cluster_size", 50);
    this->get_parameter("euclidean_clustering.max_cluster_size", _max_cluster_size);
    RCLCPP_INFO_STREAM(this->get_logger(),
                       "Using parameter: 'euclidean_clustering.max_cluster_size' = " << _max_cluster_size);
  }

  this->declare_parameter("visualize_clusters", true);
  this->get_parameter("visualize_clusters", _visualize_clusters);
  RCLCPP_INFO_STREAM(this->get_logger(), "Using parameter: 'visualize_clusters' = " << _visualize_clusters);
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Clustering>());
  rclcpp::shutdown();
  return 0;
}