#ifndef CAMERA_UTILS_HPP_
#define CAMERA_UTILS_HPP_


// Undefine any conflicting macros
#ifdef forEach
#undef forEach
#endif

// OpenCV includes
#include <opencv2/core/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <karto_sdk/Karto.h>

// ROS includes
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/transform_datatypes.h>
#include <cv_bridge/cv_bridge.h>
#include <slam_toolbox/ORBextractor.h>


namespace camera_utils {

struct Keyframe {
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
};

// // ======================== Camera Metadata ============================
// class CameraMetadata {
// public:
//     CameraMetadata();

//     cv::Mat getCameraMatrix() const { return camera_matrix_; }
//     cv::Mat getDistCoeffs() const { return dist_coeffs_; }
//     int getImageWidth() const { return image_width_; }
//     int getImageHeight() const { return image_height_; }

//     // Setters
//     void setCameraMatrix(const cv::Mat& matrix) { camera_matrix_ = matrix.clone(); }
//     void setDistCoeffs(const cv::Mat& coeffs) { dist_coeffs_ = coeffs.clone(); }
//     void setImageWidth(int width) { image_width_ = width; }
//     void setImageHeight(int height) { image_height_ = height; }

// private:
//     cv::Mat camera_matrix_, dist_coeffs_, R_, T_;
//     int image_width_, image_height_;
// };

// ======================== Image Holder ============================

class KeyframeHolder {
public:
    KeyframeHolder();
    ~KeyframeHolder();

    void addKeyframe(const Keyframe& keyframe);
    const Keyframe& getKeyframe(int id) const;
    size_t size() const { return keyframes_.size(); }
    void clear();

private:
    std::vector<Keyframe> keyframes_; // Store keyframes instead of plain images
};

// ======================== Feature Extraction ============================
class FeatureExtraction {
public:
    FeatureExtraction();
    void extractFeatures(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors);
    std::vector<cv::DMatch> matchFeatures(const cv::Mat &descriptors1, const cv::Mat &descriptors2);
    std::vector<cv::DMatch> matchFeaturesFLANN(const cv::Mat& descriptors1, const cv::Mat& descriptors2); 
    std::vector<cv::DMatch> filterMatchesWithFundamentalMatrix(const std::vector<cv::DMatch>& matches,
                                                                const std::vector<cv::KeyPoint>& keypoints1,
                                                                const std::vector<cv::KeyPoint>& keypoints2);
    std::vector<cv::DMatch> filterMatchesWithRANSAC(const std::vector<cv::DMatch>& matches, 
                                                        const std::vector<cv::KeyPoint>& keypoints1, 
                                                        const std::vector<cv::KeyPoint>& keypoints2);
    bool computeRelativePose(const std::vector<cv::DMatch>& matches, 
        const std::vector<cv::KeyPoint>& keypoints1, 
        const std::vector<cv::KeyPoint>& keypoints2, 
        karto::Pose2 &visualPose);

private:
    std::shared_ptr<orb::ORBextractor> orb_extractor_;  // orb namespace is defined in ORBextractor.h
};
}  // namespace camera_utils

#endif  // CAMERA_UTILS_HPP_
