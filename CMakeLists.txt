cmake_minimum_required(VERSION 3.16.3)
project(scale_pad_visualize)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)

# Build
add_executable(visualize src/visualize.cpp)
target_include_directories(visualize
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

# Linking to dependencies
target_link_libraries(visualize
  PRIVATE
    ${visualization_msgs_TARGETS}
    ament_index_cpp::ament_index_cpp
    geometric_shapes::geometric_shapes
    rclcpp::rclcpp
)

# Install
install(
  TARGETS visualize
  DESTINATION lib/${PROJECT_NAME})
install(
  DIRECTORY
    launch
    rviz
    stl
  DESTINATION share/${PROJECT_NAME})

ament_package()
