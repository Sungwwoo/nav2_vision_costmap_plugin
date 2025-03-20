#include <rclcpp/rclcpp.hpp>
#include <nav2_costmap_2d/layer.hpp>
#include <nav2_costmap_2d/costmap_layer.hpp>
#include <opencv4/opencv2/opencv.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <rclcpp/wait_for_message.hpp>
#include "nav2_vision_costmap_plugin/vision_layer.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <rclcpp/qos.hpp>
#include <chrono>

using namespace std::chrono_literals;
namespace nav2_vision_costmap_plugin{

VisionLayer::VisionLayer() 
    : nav2_costmap_2d::CostmapLayer(), cloud_received_(false){}


void VisionLayer::onInitialize(){
    auto node = node_.lock();
    if (!node){
        throw std::runtime_error("VisionLayer: Failed to lock node");
    }    

    global_frame_ = layered_costmap_->getGlobalFrameID();
    RCLCPP_INFO(logger_, "VisionLayer: Using global frame %s", global_frame_.c_str());
    sub_pc2_ = node->create_subscription<sensor_msgs::msg::PointCloud2>("/vision_cloud", 10, std::bind(&VisionLayer::cb_pointcloud2, this, std::placeholders::_1));
    RCLCPP_INFO(logger_, "VisionLayer: Subscribing /vision_cloud");

    RCLCPP_INFO(logger_, "VisionLayer: Loading tf buffer");
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    current_ = true;
    enabled_ = true;
    RCLCPP_INFO(logger_, "VisionLayer Initialized");
}

void VisionLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double * min_x, double * min_y, double * max_x, double * max_y){
    if (!cloud_received_)
        return;    
    // Iterate through the point cloud to expand the update bounds.
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*latest_cloud_, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*latest_cloud_, "y");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y){
        double wx = *iter_x;
        double wy = *iter_y;
        *min_x = std::min(*min_x, wx);
        *min_y = std::min(*min_y, wy);
        *max_x = std::max(*max_x, wx);
        *max_y = std::max(*max_y, wy);
    }
}

void VisionLayer::updateCosts(nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i, int max_j){
    if (!cloud_received_)
        return;
    // For each point in the cloud, convert world coordinates to map indices
    // and mark the cell as an obstacle.
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*latest_cloud_, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*latest_cloud_, "y");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y){
        double wx = *iter_x;
        double wy = *iter_y;
        unsigned int mx, my;
        if (master_grid.worldToMap(wx, wy, mx, my)){
            master_grid.setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
        }
    }
}

void VisionLayer::onFootprintChanged(){

}

void VisionLayer::cb_pointcloud2(const sensor_msgs::msg::PointCloud2::SharedPtr msg){
    if (!cloud_received_){
        camera_frame_ = msg->header.frame_id;
        RCLCPP_INFO(logger_, "VisionLayer: Using camera frame %s", camera_frame_.c_str());
        cloud_received_ = true;
    }

    // Transform PointCloud2 from camera frame to global frame
    geometry_msgs::msg::TransformStamped transform;
    try {
        transform = tf_buffer_->lookupTransform(global_frame_, msg->header.frame_id, 
                                                msg->header.stamp, tf2::durationFromSec(0.1));
        sensor_msgs::msg::PointCloud2 transformed_cloud;
        tf2::doTransform(*msg, transformed_cloud, transform);
        latest_cloud_ = std::make_shared<sensor_msgs::msg::PointCloud2>(transformed_cloud);
    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(logger_, "Could not transform point cloud: %s", ex.what());
        cloud_received_ = false;
    }
}
}


// Export the plugin so that the costmap can load it
PLUGINLIB_EXPORT_CLASS(nav2_vision_costmap_plugin::VisionLayer, nav2_costmap_2d::Layer)
