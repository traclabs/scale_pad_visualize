#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>

visualization_msgs::Marker getEdgesMarker(const shapes::Mesh* mesh, int marker_id)
{
  visualization_msgs::Marker marker_edges;
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

visualization_msgs::Marker getFacesMarker(const shapes::Mesh* mesh, int marker_id)
{
  visualization_msgs::Marker marker_faces;
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
  ros::init(argc, argv, "visualize_covariance_intersection");
  ros::NodeHandle nh;
  auto markers_pub = nh.advertise<visualization_msgs::MarkerArray>("marker", 10, true);
  visualization_msgs::MarkerArray markers;

  while (ros::ok())
  {
    // Load parameters
    double scale, padding;
    std::string mesh_file;
    ros::param::get("~scale", scale);
    ros::param::get("~padding", padding);
    ros::param::get("~mesh", mesh_file);

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
    markers_pub.publish(markers);

    ros::Duration(0.2).sleep();
    ros::spinOnce();
  }

  ros::spin();
}
