#ifndef CAMERA_FEATURE_EXTRACTION_NODE_HPP_
#define CAMERA_FEATURE_EXTRACTION_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "slam_toolbox/camera_utils.hpp"
#include "slam_toolbox/ORBextractor.h"


class CameraFeatureExtractionNode : public rclcpp::Node {
public:
    CameraFeatureExtractionNode(std::shared_ptr<camera_utils::KeyframeHolder> keyframe_holder);

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr orb_feature_pub_;

    std::shared_ptr<camera_utils::FeatureExtraction> feature_extractor_; 
    std::shared_ptr<camera_utils::KeyframeHolder> keyframe_holder_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::shared_ptr<sensor_msgs::msg::Image> last_image_msg_; 
    
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    void publishKeypoints(const std::vector<cv::KeyPoint>& keypoints);
    std::pair<std::vector<cv::KeyPoint>, cv::Mat> extractFeatures(const cv::Mat &image);
    bool isKeyframe(const std::vector<cv::KeyPoint>& keypoints, const cv::Mat& descriptors);
};


#endif  // CAMERA_FEATURE_EXTRACTION_NODE_HPP_

