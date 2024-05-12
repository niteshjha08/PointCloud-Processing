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

  _visualize_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions options1;
  options1.callback_group = _visualize_callback_group;
  // _bounding_box_sub = this->create_subscription<point_cloud_proc::msg::ArrayPointCloud>(
  //     "/cluster_clouds", 10, std::bind(&Clustering::publish_bounding_boxes, this, _1), options1);

  _clusters_visualization_sub = this->create_subscription<point_cloud_proc::msg::ArrayPointCloud>(
      "/cluster_clouds", 10, std::bind(&Clustering::visualize_clusters, this, _1));

  _cluster_visualization_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cluster_visualization", 10);

  _bounding_box_calc_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  rclcpp::SubscriptionOptions options;
  options.callback_group = _bounding_box_calc_callback_group;
  _bounding_box_sub = this->create_subscription<point_cloud_proc::msg::ArrayPointCloud>(
      "/cluster_clouds", 10, std::bind(&Clustering::publish_bounding_boxes, this, _1), options1);

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
    Eigen::MatrixXd bbox = get_cluster_bounding_box(cloud);

    auto marker = get_bounding_box_marker(bbox);
    marker.id = i;
    marker.header = clusters_msg->header;
    marker_array.markers.push_back(marker);
    i += 1;
  }
  _bounding_box_pub->publish(marker_array);
}

Eigen::MatrixXd Clustering::get_cluster_bounding_box(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster)
{
  Eigen::Matrix<double, Eigen::Dynamic, 2> cluster_points(cluster->points.size(), 2);
  Eigen::Vector<double, Eigen::Dynamic> z_coordinates(cluster->points.size());

  for (int i = 0; i < cluster->points.size(); ++i)
  {
    cluster_points.row(i) = Eigen::RowVector2d(cluster->points[i].x, cluster->points[i].y);
    z_coordinates(i) = cluster->points[i].z;
  }

  Eigen::MatrixXd centered = cluster_points.rowwise() - cluster_points.colwise().mean();

  Eigen::MatrixXd cov = (centered.transpose() * centered.transpose().adjoint()) / double(cluster_points.rows());

  Eigen::Matrix2d eigvecs = find_eigenvectors_2d(cov);

  Eigen::MatrixXd aligned_coords = centered * eigvecs;

  Eigen::Vector2d min_xy = aligned_coords.colwise().minCoeff();

  Eigen::Vector2d max_xy = aligned_coords.colwise().maxCoeff();
  Eigen::Vector2d z_extremes;
  z_extremes << z_coordinates.minCoeff(), z_coordinates.maxCoeff();

  Eigen::MatrixXd bounding_box_coords(8, 2);

  bounding_box_coords.row(0) = Eigen::RowVector2d(min_xy[0], min_xy[1]);
  bounding_box_coords.row(1) = Eigen::RowVector2d(min_xy[0], max_xy[1]);
  bounding_box_coords.row(2) = Eigen::RowVector2d(max_xy[0], max_xy[1]);
  bounding_box_coords.row(3) = Eigen::RowVector2d(max_xy[0], min_xy[1]);
  bounding_box_coords.row(4) = Eigen::RowVector2d(min_xy[0], min_xy[1]);
  bounding_box_coords.row(5) = Eigen::RowVector2d(min_xy[0], max_xy[1]);
  bounding_box_coords.row(6) = Eigen::RowVector2d(max_xy[0], max_xy[1]);
  bounding_box_coords.row(7) = Eigen::RowVector2d(max_xy[0], min_xy[1]);
  Eigen::MatrixXd bounding_box_coords_with_z(8, 3);
  Eigen::VectorXd z_coords(8);
  z_coords << z_extremes[0], z_extremes[0], z_extremes[0], z_extremes[0], z_extremes[1], z_extremes[1], z_extremes[1],
      z_extremes[1];

  bounding_box_coords = (eigvecs * bounding_box_coords.transpose()).transpose();

  bounding_box_coords = bounding_box_coords.rowwise() + cluster_points.colwise().mean();
  bounding_box_coords_with_z.block(0, 0, 8, 2) = bounding_box_coords;
  bounding_box_coords_with_z.col(2) = z_coords;

  return bounding_box_coords_with_z;
}

void Clustering::visualize_clusters(point_cloud_proc::msg::ArrayPointCloud::SharedPtr clusters_msg)
{
  pcl::PointCloud<pcl::PointXYZRGB> aggregate_cloud;
  int num_clouds = clusters_msg->clouds.size();
  if (num_clouds == 0)
  {
    return;
  }
  std::vector<std::vector<int>> cluster_colors = get_cluster_colors(num_clouds);
  int cluster_idx = 0;
  for (auto cluster_ros : clusters_msg->clouds)
  {
    pcl::PointCloud<pcl::PointXYZI> cluster;
    pcl::fromROSMsg(cluster_ros, cluster);
    std::vector<int> colors = cluster_colors[cluster_idx];

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