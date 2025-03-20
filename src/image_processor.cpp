#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv4/opencv2/opencv.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <rclcpp/wait_for_message.hpp>
#include <vector>
#include "cv_bridge/cv_bridge.h"

using std::placeholders::_1;
using std::placeholders::_2;
class ImageProcessor: public rclcpp::Node{
public:
    ImageProcessor(): Node("image_processor"){
        
        this->set_parameter(rclcpp::Parameter("use_sim_time", true));
        pub_pc2_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("vision_cloud", 10);
        sub_rgb_.subscribe(this, "/base_camera/image_raw");
        sub_depth_.subscribe(this, "/base_camera/depth/image_raw");

        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10), sub_rgb_, sub_depth_);
        sync_->registerCallback(std::bind(&ImageProcessor::cb_image, this, _1, _2));
    }

    void get_camera_intrinsic(){
        auto camera_info = sensor_msgs::msg::CameraInfo();
        rclcpp::wait_for_message(camera_info, this->shared_from_this(), "/base_camera/camera_info");

        intrinsic_ = camera_info.k;
        RCLCPP_INFO(this->get_logger(), "Got intrinsic fx: %f, fy: %f, cx: %f, cy: %f", intrinsic_[0], intrinsic_[4], intrinsic_[2], intrinsic_[5]);

    }
private:
    void cb_image(const sensor_msgs::msg::Image::ConstSharedPtr& rgb, const sensor_msgs::msg::Image::ConstSharedPtr& depth) {
        
        cv_bridge::CvImagePtr p_rgb, p_depth;
        p_rgb = cv_bridge::toCvCopy(rgb, "bgr8");
        p_depth = cv_bridge::toCvCopy(depth, depth->encoding);
        cv::Scalar yellowlb(0, 200, 200), yellowub(40, 255, 255);
        cv::Scalar whitelb(220, 220, 220), whiteub(255, 255, 255);

        cv::Mat yellowMask, whiteMask;
        cv::inRange(p_rgb->image, yellowlb, yellowub, yellowMask);
        cv::inRange(p_rgb->image, whitelb, whiteub, whiteMask);
        
        cv::Mat mask;
        cv::bitwise_or(yellowMask, whiteMask, mask);

        cv::Mat detectedImage;
        cv::bitwise_and(p_rgb->image, p_rgb->image, detectedImage, mask);

        std::vector<cv::Point3f> cvPointCloud;
        for (int v=0; v<mask.rows; v++){
            for (int u=0; u<mask.cols; u++){
                if (mask.at<uchar>(v, u) > 0){
                    float depthVal = p_depth->image.at<float>(v, u);
                    if (depthVal > 0){
                        //     [fx  0 cx]
                        // K = [ 0 fy cy]
                        //     [ 0  0  1]
                        float z = depthVal;
                        float x = (u - intrinsic_[2]) * z / intrinsic_[0];
                        float y = (v - intrinsic_[5]) * z / intrinsic_[4];
                        cvPointCloud.push_back(cv::Point3f(x, y, z));
                    }
                }
            }
        }

        sensor_msgs::msg::PointCloud2 pointCloud;
        createPointCloudMsg(pointCloud, cvPointCloud);
        pub_pc2_->publish(pointCloud);
        
    }

    void createPointCloudMsg(sensor_msgs::msg::PointCloud2 &pc, const std::vector<cv::Point3f> &points){
        // Header
        pc.header.frame_id = "base_camera_color_optical_frame";
        pc.header.stamp = this->now();

        pc.height = 1;
        pc.width = points.size();
        pc.is_dense = false;

        sensor_msgs::PointCloud2Modifier modifier(pc);
        modifier.setPointCloud2FieldsByString(1, "xyz");
        modifier.resize(points.size());

        sensor_msgs::PointCloud2Iterator<float> iter_x(pc, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(pc, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(pc, "z");
        
        for (const auto& p:points){
            *iter_x = p.x;
            *iter_y = p.y;
            if (p.z < 0.0)
                *iter_z = 0.05;
            else
                *iter_z = p.z;
            ++iter_x; ++iter_y; ++iter_z;
            // RCLCPP_INFO(this->get_logger(), "Val: %f, %f, %f", p.x, p.y, p.z);
        }
    }

    using SyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>;
    message_filters::Subscriber<sensor_msgs::msg::Image> sub_rgb_;
    message_filters::Subscriber<sensor_msgs::msg::Image> sub_depth_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pc2_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
    std::array<double, 9> intrinsic_;
};


int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    std::shared_ptr<ImageProcessor> node = std::make_shared<ImageProcessor>();
    node->get_camera_intrinsic();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}