/* camera_utils.cpp */

#include <cmath>
#include <string>
#include <vector>
#include <memory>

//#include <opencv2/opencv.hpp>
// #include <opencv2/core/core.hpp>
// #include <opencv2/highgui/highgui.hpp>
// #include <opencv2/features2d/features2d.hpp>
// #include <opencv2/imgproc/imgproc.hpp>

#include "slam_toolbox/camera_utils.hpp"
// #include "slam_toolbox/ORBextractor.h"

using namespace cv;
using namespace std;
using namespace orb;

namespace camera_utils {


KeyframeHolder::KeyframeHolder() 
{

}
KeyframeHolder::~KeyframeHolder() 
{

}

void KeyframeHolder::addKeyframe(const Keyframe& keyframe) {
    keyframes_.push_back(keyframe);
}

const Keyframe& KeyframeHolder::getKeyframe(int id) const {
    if (id < 0 || id >= (int)keyframes_.size()) {
        throw std::out_of_range("KeyframeHolder: Keyframe index out of range");
    }
    return keyframes_.at(id);
}

void KeyframeHolder::clear() {
    keyframes_.clear();
}


FeatureExtraction::FeatureExtraction() {
    try {
        int nFeatures = 100;
        float scaleFactor = 1.2f;
        int nLevels = 8;
        int iniThFAST = 30;
        int minThFAST = 10;

        orb_extractor_ = std::make_unique<orb::ORBextractor>(
            nFeatures, scaleFactor, nLevels, iniThFAST, minThFAST);

        if (!orb_extractor_) {
            throw std::runtime_error("ORBextractor failed to initialize.");
        }

    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("FeatureExtraction"), " Exception in ORBextractor constructor: %s", e.what());
    }
}
    

void FeatureExtraction::extractFeatures(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors) {
    if (image.empty()) {
        RCLCPP_ERROR(rclcpp::get_logger("FeatureExtraction"), "ERROR: Received empty image! Skipping feature extraction.");
        return;
    }

    //RCLCPP_INFO(rclcpp::get_logger("FeatureExtraction"), " Processing image - Size: %dx%d, Type: %d",
     //           image.cols, image.rows, image.type());

    std::vector<int> vLappingArea = {0, image.cols};
    // cv::Mat mask = cv::noArray();  // Empty mask

    if (!orb_extractor_) {
        RCLCPP_ERROR(rclcpp::get_logger("FeatureExtraction"), "ERROR: ORBextractor is NULL!");
        return;
    } 

    // RCLCPP_INFO(rclcpp::get_logger("FeatureExtraction"), "Running ORBextractor...");
    orb_extractor_->operator()(image, noArray(), keypoints, descriptors, vLappingArea);
    // RCLCPP_INFO(rclcpp::get_logger("FeatureExtraction"), "Extracted %lu keypoints.", keypoints.size());

} 

/** Match ORB Features */
vector<DMatch> FeatureExtraction::matchFeatures(
    // for binary string descriptors
    const Mat& descriptors1, const Mat& descriptors2) {

    BFMatcher matcher(NORM_HAMMING, true);  // Brute-force matcher
    vector<DMatch> matches;
    matcher.match(descriptors1, descriptors2, matches);
    return matches;
}

vector<DMatch> FeatureExtraction::matchFeaturesFLANN(const Mat& descriptors1, const Mat& descriptors2) {
    // for floating point descriptors such as SURF or SIFT
    if (descriptors1.empty() || descriptors2.empty()) {
        RCLCPP_WARN(rclcpp::get_logger("FeatureExtraction"), "FLANN Matching: One or both descriptor matrices are empty!");
        return {};  // Return empty vector
    }

    // Log descriptor types
    // RCLCPP_INFO(rclcpp::get_logger("FeatureExtraction"), 
    //             "Descriptor1 Type: %d, Descriptor2 Type: %d", descriptors1.type(), descriptors2.type());

    Mat desc1, desc2;
    descriptors1.convertTo(desc1, CV_32F);
    descriptors2.convertTo(desc2, CV_32F);

    // Log converted descriptor types
    // RCLCPP_INFO(rclcpp::get_logger("FeatureExtraction"), 
    //             "Converted Descriptor1 Type: %d, Converted Descriptor2 Type: %d", desc1.type(), desc2.type());

    if (desc1.cols != desc2.cols || desc1.cols != 32) {  // ORB descriptors are always 32 bytes
        RCLCPP_ERROR(rclcpp::get_logger("FeatureExtraction"), 
                        "FLANN Error: Descriptor column sizes do not match or are incorrect! Desc1: %d x %d, Desc2: %d x %d",
                        desc1.rows, desc1.cols, desc2.rows, desc2.cols);
        return {};
    }

    if (desc1.type() != CV_32F || desc2.type() != CV_32F) {
        RCLCPP_ERROR(rclcpp::get_logger("FeatureExtraction"), 
                        "FLANN Error: Descriptor types are not CV_32F after conversion! Desc1 Type: %d, Desc2 Type: %d",
                        desc1.type(), desc2.type());
        return {};
    }
    Ptr<flann::IndexParams> indexParams = makePtr<flann::LshIndexParams>(12, 20, 2);
    Ptr<flann::SearchParams> searchParams = makePtr<flann::SearchParams>(50);
    FlannBasedMatcher matcher(indexParams, searchParams);

    vector<vector<DMatch>> knnMatches;
    try {
        matcher.knnMatch(desc1, desc2, knnMatches, 2); // KNN with k=2
    } catch (const cv::Exception &e) {
        RCLCPP_ERROR(rclcpp::get_logger("FeatureExtraction"), 
                     "FLANN Matcher Exception: %s", e.what());
        return {};
    }

    // Apply Lowe's ratio test
    vector<DMatch> good_matches;
    for (const auto& match : knnMatches) {
        if (match.size() >= 2 && match[0].distance < 0.75 * match[1].distance) {
            good_matches.push_back(match[0]);
        }
    }

    RCLCPP_INFO(rclcpp::get_logger("FeatureExtraction"), "FLANN Matching found %lu good matches.", good_matches.size());
    return good_matches;

}

vector<DMatch> FeatureExtraction::filterMatchesWithFundamentalMatrix(const vector<DMatch>& matches,
                                                  const vector<KeyPoint>& keypoints1,
                                                  const vector<KeyPoint>& keypoints2) {
    vector<Point2f> points1, points2;
    for (const auto& match : matches) {
        points1.push_back(keypoints1[match.queryIdx].pt);
        points2.push_back(keypoints2[match.trainIdx].pt);
    }

    // Compute fundamental matrix using RANSAC
    vector<uchar> mask;
    Mat F = findFundamentalMat(points1, points2, FM_RANSAC, 3.0, 0.99, mask);

    vector<DMatch> filtered_matches;
    for (size_t i = 0; i < matches.size(); i++) {
        if (mask[i]) {
            filtered_matches.push_back(matches[i]);
        }
    }

    return filtered_matches;
}

/** Filter Matches Using RANSAC (PROSAC) */
vector<DMatch> FeatureExtraction::filterMatchesWithRANSAC(
    const vector<DMatch>& matches, 
    const vector<KeyPoint>& keypoints1, 
    const vector<KeyPoint>& keypoints2) {

    if (matches.empty()) return {}; // Return empty if no matches

    vector<Point2f> points1, points2;
    for (const auto& match : matches) {
        points1.push_back(keypoints1[match.queryIdx].pt);
        points2.push_back(keypoints2[match.trainIdx].pt);
    }

    Mat mask;
    findFundamentalMat(points1, points2, FM_RANSAC, 3, 0.99, mask);

    vector<DMatch> filtered_matches;
    for (size_t i = 0; i < matches.size(); i++) {
        if (mask.at<uchar>(i)) {
            filtered_matches.push_back(matches[i]);
        }
    }

    return filtered_matches;
}
}  // namespace camera_utils
