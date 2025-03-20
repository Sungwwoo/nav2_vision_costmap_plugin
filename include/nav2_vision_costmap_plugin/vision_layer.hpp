#ifndef NAV2_VISION_LAYER_HPP_
#define NAV2_VISION_LAYER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav2_costmap_2d/layer.hpp>
#include <nav2_costmap_2d/costmap_layer.hpp>
#include <opencv4/opencv2/opencv.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <rclcpp/wait_for_message.hpp>
#include <string>
#include <vector>
#include <array>
#include <mutex>
#include <algorithm>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include "cv_bridge/cv_bridge.h"

namespace nav2_vision_costmap_plugin{
class VisionLayer: public nav2_costmap_2d::CostmapLayer{
public:
    VisionLayer();

    virtual void onInitialize();
    virtual void updateBounds(
        double robot_x, double robot_y, double robot_yaw, double * min_x,
        double * min_y,
        double * max_x,
        double * max_y);
    virtual void updateCosts(
        nav2_costmap_2d::Costmap2D & master_grid,
        int min_i, int min_j, int max_i, int max_j);
    
    virtual void reset()    {
        return;
    }

    virtual void onFootprintChanged();

    virtual bool isClearable() {return false;}

private:
    double last_min_x_, last_min_y_, last_max_x_, last_max_y_;
    void cb_pointcloud2(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pc2_;
    sensor_msgs::msg::PointCloud2::SharedPtr latest_cloud_;
    std::mutex obstacle_mutex_;
    bool cloud_received_;
    std::string global_frame_;
    std::string camera_frame_;            
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
}; 
}

#endif