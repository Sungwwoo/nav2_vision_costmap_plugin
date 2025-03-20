cmake_minimum_required(VERSION 3.8)
project(nav2_vision_costmap_plugin)

set(lib_name ${PROJECT_NAME}_core)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)


set (dep_pkgs
    rclcpp
    nav2_costmap_2d
    pluginlib
    cv_bridge
    geometry_msgs
    sensor_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    tf2_sensor_msgs
    OpenCV)

  add_executable(image_processor src/image_processor.cpp)
  ament_target_dependencies(image_processor cv_bridge rclcpp sensor_msgs OpenCV message_filters)
  

add_library(${lib_name} SHARED
            src/vision_layer.cpp)
include_directories(include)

install(TARGETS ${lib_name}
  DESTINATION lib)

install(TARGETS
  image_processor
  DESTINATION lib/${PROJECT_NAME})

pluginlib_export_plugin_description_file(nav2_costmap_2d vision_layer.xml)
ament_target_dependencies(${lib_name} ${dep_pkgs})
ament_package()
