#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "slam_toolbox/camera_feature_extraction_node.hpp"

using namespace std;
using namespace cv;
using namespace camera_utils;


CameraFeatureExtractionNode::CameraFeatureExtractionNode(
    std::shared_ptr<camera_utils::KeyframeHolder> keyframe_holder)
    : Node("camera_feature_extraction"), keyframe_holder_(keyframe_holder) {  

    RCLCPP_INFO(this->get_logger(), "Initializing Camera Feature Extraction Node...");

    try {
        feature_extractor_ = std::make_shared<camera_utils::FeatureExtraction>();
        if (!feature_extractor_) {
            throw std::runtime_error("Feature extractor failed to initialize.");
        }

        RCLCPP_INFO(this->get_logger(), "Feature extractor and image holder initialized.");
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Exception during initialization: %s", e.what());
        rclcpp::shutdown();
        return;
    }

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this);

    last_image_msg_ = nullptr;

    if (!tf_buffer_ || !tf_listener_) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize TF2 components.");
        rclcpp::shutdown();
        return;
    }

    // initializing image subscriber and feature publisher
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/camera/color/image_raw", rclcpp::QoS(1), 
        std::bind(&CameraFeatureExtractionNode::imageCallback, this, std::placeholders::_1)
    );
    
    orb_feature_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
        "/slam_toolbox/orb_features", 10
    );

}


void CameraFeatureExtractionNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    if (!msg || msg->data.empty()) {
        RCLCPP_ERROR(this->get_logger(), "ERROR: Received NULL or empty image message!");
        return;
    }

    // RCLCPP_INFO(this->get_logger(), "image encoding: %s", msg->encoding.c_str());
    last_image_msg_ = msg;

    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8); // change encoding if not BGR8

    // Convert to ORB-compatible format (CV_8UC1)
    cv::Mat gray_image;
    gray_image = cv::Mat(cv_ptr->image.size(), CV_8UC1);

    if (cv_ptr->image.type() == CV_8UC3) {
        cvtColor(cv_ptr->image, gray_image, cv::COLOR_BGR2GRAY);
    } else if (cv_ptr->image.type() == CV_16UC1) {
        cv_ptr->image.convertTo(gray_image, CV_8UC1, 1.0 / 256.0);
        RCLCPP_INFO(this->get_logger(), "type 16");
    } else if (cv_ptr->image.type() == CV_8UC1) {
        gray_image = cv_ptr->image.clone(); 
        RCLCPP_INFO(this->get_logger(), "Already Gray");
    } else {
        RCLCPP_ERROR(this->get_logger(), " ERROR: Unsupported image format: %d", cv_ptr->image.type());
        return;
    }

    auto [keypoints, descriptors] = extractFeatures(gray_image);

    publishKeypoints(keypoints, cv_ptr->image);

    // if (isKeyframe(keypoints, descriptors)) {
    //     Keyframe new_keyframe = {msg, keypoints, descriptors.clone()};  // Use Keyframe struct
    //     keyframe_holder_->addKeyframe(new_keyframe);                     // Pass the struct
    //     RCLCPP_INFO(this->get_logger(), "New keyframe added with %lu keypoints!", keypoints.size());
    // } else {
    //     // RCLCPP_INFO(this->get_logger(), "Ignore this keyframe!");
    // }
    
}

std::pair<std::vector<cv::KeyPoint>, cv::Mat> CameraFeatureExtractionNode::extractFeatures(const cv::Mat &image) {
    if (image.empty()) {
        RCLCPP_ERROR(this->get_logger(), "ERROR: Received an empty image for feature extraction!");
        return {{}, {}};
    }

    vector<KeyPoint> keypoints;
    Mat descriptors;
    feature_extractor_->extractFeatures(image, keypoints, descriptors);

    return {keypoints, descriptors};
}

bool CameraFeatureExtractionNode::isKeyframe(const std::vector<cv::KeyPoint>& keypoints, const cv::Mat& descriptors) {
    if (keyframe_holder_->size() == 0) {
        // First frame is always stored as a keyframe
        Keyframe new_keyframe{last_image_msg_, keypoints, descriptors.clone()};
        keyframe_holder_->addKeyframe(new_keyframe);
        RCLCPP_INFO(this->get_logger(), "First frame - Saving as keyframe.");
        return true;
    }

    try {
        if (!last_image_msg_) {
            RCLCPP_ERROR(this->get_logger(), "last_image_msg_ is NULL! Cannot save keyframe.");
            return false;
        }

        //// logic error?
        int best_match_count = 0;
        int step_size = std::max(1, static_cast<int>(keyframe_holder_->size() / 10)); 

        for (size_t i = 0; i < keyframe_holder_->size(); i += step_size) {
            const Keyframe& past_kf = keyframe_holder_->getKeyframe(i);
            vector<DMatch> matches = feature_extractor_->matchFeatures(descriptors, past_kf.descriptors);

            if (matches.size() > best_match_count) {
                best_match_count = matches.size();
            }
        }

        RCLCPP_INFO(this->get_logger(), "Best match count: %d", best_match_count);

        // If best match is LOW, we save as a new keyframe
        if (best_match_count < 300) {
            // RCLCPP_INFO(this->get_logger(), "Match count below threshold (%d < 300). Saving keyframe.", best_match_count);

            Keyframe new_keyframe{last_image_msg_, keypoints, descriptors.clone()};
            keyframe_holder_->addKeyframe(new_keyframe);

            // Confirm keyframe was actually added
            // RCLCPP_INFO(this->get_logger(), "Total keyframes stored: %zu", keyframe_holder_->size());
            return true;
        } else {
            RCLCPP_INFO(this->get_logger(), "Keyframe skipped - Too many matches (%d).", best_match_count);
        }

        return false;
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Exception in keyframe processing: %s", e.what());
        return false;
    }
}


void CameraFeatureExtractionNode::publishKeypoints(const vector<KeyPoint>& keypoints, const cv::Mat &image) {
    if (keypoints.empty()) {
        RCLCPP_WARN(this->get_logger(), "No keypoints detected, skipping publish.");
        return;
    }

    if (!tf_buffer_ || !tf_listener_) {
        RCLCPP_ERROR(this->get_logger(), "TF Buffer or Listener not initialized!");
        return;
    }

    // Create a copy of the image to draw on
    cv::Mat image_with_keypoints = image.clone();

    // Draw the keypoints on the image
    cv::drawKeypoints(image, keypoints, image_with_keypoints, cv::Scalar(0, 255, 0), cv::DrawMatchesFlags::DRAW_OVER_OUTIMG);

    // Now you have the image_with_keypoints, you can publish it.
    // Assuming you have a publisher called image_publisher_ of type sensor_msgs::msg::Image.
    // and a function cvMatToImageMsg(const cv::Mat& mat) to convert cv::Mat to sensor_msgs::msg::Image.

    // Convert the cv::Mat to sensor_msgs::msg::Image
    sensor_msgs::msg::Image image_msg = *(cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image_with_keypoints).toImageMsg());

    // Set the timestamp and frame id (if applicable)
    image_msg.header.stamp = this->now();
    //if you have a frame_id, set it.
    //image_msg.header.frame_id = "camera_frame";

    // Publish the image
    if (orb_feature_pub_) {
        orb_feature_pub_->publish(image_msg);
    } else {
        RCLCPP_ERROR(this->get_logger(),"Image publisher is not initialized.");
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    // Create a shared KeyframeHolder
    auto keyframe_holder = std::make_shared<camera_utils::KeyframeHolder>();

    // Pass the keyframe holder to CameraFeatureExtractionNode
    auto camera_node = std::make_shared<CameraFeatureExtractionNode>(keyframe_holder);
    rclcpp::spin(camera_node);
    rclcpp::shutdown();
    return 0;
}