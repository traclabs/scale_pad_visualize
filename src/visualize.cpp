#include <chrono>

#include <ament_index_cpp/get_package_prefix.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

using namespace std::chrono_literals;

visualization_msgs::msg::Marker getEdgesMarker(const shapes::Mesh* mesh, int marker_id)
{
  visualization_msgs::msg::Marker marker_edges;
  shapes::constructMarkerFromShape(mesh, marker_edges, false);
  marker_edges.header.frame_id = "world";
  marker_edges.id = marker_id;
  marker_edges.color.r = 0.0;
  marker_edges.color.g = 0.0;
  marker_edges.color.b = 0.0;
  marker_edges.color.a = 1.0;
  marker_edges.scale.x = 0.005;
  return marker_edges;
}

visualization_msgs::msg::Marker getFacesMarker(const shapes::Mesh* mesh, int marker_id)
{
  visualization_msgs::msg::Marker marker_faces;
  shapes::constructMarkerFromShape(mesh, marker_faces, true);
  marker_faces.header.frame_id = "world";
  marker_faces.id = marker_id;
  marker_faces.color.r = 0.7;
  marker_faces.color.g = 0.7;
  marker_faces.color.b = 0.7;
  marker_faces.color.a = 1.0;
  return marker_faces;
}


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("visualize");
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).transient_local();
  auto markers_pub = node->create_publisher<visualization_msgs::msg::MarkerArray>("marker", qos);
  visualization_msgs::msg::MarkerArray markers;

  // Declare parameters
  node->declare_parameter<double>("scale", 1.0);
  node->declare_parameter<double>("padding", 0.0);
  std::string package_share_directory;
  try {
    package_share_directory = ament_index_cpp::get_package_share_directory("scale_pad_visualize");
  } catch (const ament_index_cpp::PackageNotFoundError &e) {
    throw std::runtime_error(e.what());
  }
  node->declare_parameter<std::string>("mesh", package_share_directory + "/stl/ros2.stl");

  while (rclcpp::ok())
  {
    // Get parameters
    double scale, padding;
    std::string mesh_file;
    node->get_parameter<double>("scale", scale);
    node->get_parameter<double>("padding", padding);
    node->get_parameter<std::string>("mesh", mesh_file);

    unsigned marker_id = 0;

    // Add markers for original mesh
    // markers.markers.push_back(getFacesMarker(mesh, marker_id++));
    // markers.markers.push_back(getEdgesMarker(mesh, marker_id++));

    // Add markers for scale/padded mesh
    auto mesh = shapes::createMeshFromResource("file://" + mesh_file);
    mesh->scaleAndPadd(scale, scale, scale, padding, padding, padding);
    auto faces_marker = getFacesMarker(mesh, marker_id++);
    markers.markers.push_back(faces_marker);
    auto edges_marker = getEdgesMarker(mesh, marker_id++);
    markers.markers.push_back(edges_marker);
    markers_pub->publish(markers);

    rclcpp::sleep_for(200ms);
    rclcpp::spin_some(node);
  }
}
