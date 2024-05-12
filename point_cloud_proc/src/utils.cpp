/*
def gen_colors(num_clusters):
    colors = []
    color_delta = 768//num_clusters
    for i in range(num_clusters):
        total_color_val = (i+1)*color_delta
        r, g, b = 0, 0, 0
        while(total_color_val>0):
            delta_r = min(255-r, min(total_color_val,color_delta))
            total_color_val -= delta_r
            r+=delta_r
            if r >= 255:
                break
        while(total_color_val>0):
            delta_g = min(255-g, min(total_color_val,color_delta))
            total_color_val -= delta_g
            g+=delta_g
            if g >= 255:
                break
        while(total_color_val>0):
            delta_b = min(255-b, min(total_color_val,color_delta))
            total_color_val -= delta_b
            b+=delta_b
            if b >= 255:
                break
        colors.append([r,g,b])
    return colors
*/
#ifndef UTILS_H
#include <vector>
std::vector<std::vector<int>> get_cluster_colors(int num_clusters)
{
  std::vector<std::vector<int>> colors;
  int color_delta = 768 / num_clusters;
  int total_color_val;
  for (int i = 0; i < num_clusters; ++i)
  {
    total_color_val = (i + 1) * color_delta;
    int r = 0, g = 0, b = 0;
    int delta_r, delta_g, delta_b;
    while (total_color_val > 0)
    {
      delta_r = std::min(255 - r, std::min(total_color_val, color_delta));
      total_color_val -= delta_r;
      r += delta_r;
      if (r >= 255)
      {
        break;
      }
    }

    while (total_color_val > 0)
    {
      delta_g = std::min(255 - g, std::min(total_color_val, color_delta));
      total_color_val -= delta_g;
      g += delta_g;
      if (g >= 255)
      {
        break;
      }
    }

    while (total_color_val > 0)
    {
      delta_b = std::min(255 - b, std::min(total_color_val, color_delta));
      total_color_val -= delta_b;
      b += delta_b;
      if (r >= 255)
      {
        break;
      }
    }

    std::vector<int> cluster_colors{ r, g, b };
    colors.push_back(cluster_colors);
  }
  return colors;
}

Eigen::Matrix2d find_eigenvectors_2d(Eigen::Matrix2d mat)
{
  double eps = 0.000001;
  double tr_2 = (mat(0, 0) + mat(1, 1)) * 0.5;
  double det = (mat(0, 0) * mat(1, 1)) - (mat(0, 1) * mat(1, 0));
  double disc = (tr_2 * tr_2) - det + eps;
  if (disc < 0.0)
  {
    // throw exception
  }
  disc = sqrt(disc);
  std::vector<double> eigenvalues{ tr_2 + disc, tr_2 - disc };
  std::vector<double> eigvec1, eigvec2;
  if (fabs(mat(0, 1) * mat(0, 1)) > eps)
  {
    eigvec1.push_back(mat(0, 1));
    eigvec1.push_back(eigenvalues[0] - mat(0, 0));

    eigvec2.push_back(mat(0, 1));
    eigvec2.push_back(eigenvalues[1] - mat(0, 0));
  }
  else
  {
    if (mat(0, 0) > mat(1, 1))
    {
      eigvec1.push_back(1);
      eigvec1.push_back(0);
      eigvec2.push_back(0);
      eigvec2.push_back(1);
    }
    else
    {
      eigvec1.push_back(0);
      eigvec1.push_back(1);
      eigvec2.push_back(1);
      eigvec2.push_back(0);
    }
  }
  double eigvec1_norm = sqrt((eigvec1[0] * eigvec1[0]) + (eigvec1[1] * eigvec1[1]));
  eigvec1[0] = eigvec1[0] / eigvec1_norm;
  eigvec1[1] = eigvec1[1] / eigvec1_norm;

  double eigvec2_norm = sqrt((eigvec2[0] * eigvec2[0]) + (eigvec2[1] * eigvec2[1]));
  eigvec2[0] = eigvec2[0] / eigvec2_norm;
  eigvec2[1] = eigvec2[1] / eigvec2_norm;
  Eigen::Matrix2d eigvecs;
  eigvecs << eigvec1[0], eigvec1[1], eigvec2[0], eigvec2[1];
  return eigvecs;
}

visualization_msgs::msg::Marker get_bounding_box_marker(Eigen::MatrixXd bbox)
{
  auto marker = visualization_msgs::msg::Marker();

  marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = 0.01;  // width of line

  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;
  geometry_msgs::msg::Point p1, p2, p3, p4, p5, p6, p7, p8;
  p1.x = bbox(0, 0);
  p1.y = bbox(0, 1);
  p1.z = bbox(0, 2);

  p2.x = bbox(1, 0);
  p2.y = bbox(1, 1);
  p2.z = bbox(1, 2);

  p3.x = bbox(2, 0);
  p3.y = bbox(2, 1);
  p3.z = bbox(2, 2);

  p4.x = bbox(3, 0);
  p4.y = bbox(3, 1);
  p4.z = bbox(3, 2);

  p5.x = bbox(4, 0);
  p5.y = bbox(4, 1);
  p5.z = bbox(4, 2);

  p6.x = bbox(5, 0);
  p6.y = bbox(5, 1);
  p6.z = bbox(5, 2);

  p7.x = bbox(6, 0);
  p7.y = bbox(6, 1);
  p7.z = bbox(6, 2);

  p8.x = bbox(7, 0);
  p8.y = bbox(7, 1);
  p8.z = bbox(7, 2);
  // bottom face
  marker.points.push_back(p1);
  marker.points.push_back(p2);
  marker.points.push_back(p2);
  marker.points.push_back(p3);
  marker.points.push_back(p3);
  marker.points.push_back(p4);
  marker.points.push_back(p4);
  marker.points.push_back(p1);
  // top face
  marker.points.push_back(p5);
  marker.points.push_back(p6);
  marker.points.push_back(p6);
  marker.points.push_back(p7);
  marker.points.push_back(p7);
  marker.points.push_back(p8);
  marker.points.push_back(p8);
  marker.points.push_back(p5);
  // vertical edges
  marker.points.push_back(p1);
  marker.points.push_back(p5);
  marker.points.push_back(p4);
  marker.points.push_back(p8);
  marker.points.push_back(p3);
  marker.points.push_back(p7);
  marker.points.push_back(p2);
  marker.points.push_back(p6);
  return marker;
}

#endif