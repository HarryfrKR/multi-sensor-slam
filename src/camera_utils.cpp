/* camera_utils.cpp */

#include <cmath>
#include <string>
#include <vector>
#include <memory>


#include "slam_toolbox/camera_utils.hpp"


using namespace cv;
using namespace std;
using namespace orb;
using namespace karto;

namespace camera_utils {


KeyframeHolder::KeyframeHolder() 
{

}
KeyframeHolder::~KeyframeHolder() 
{

}
std::vector<Keyframe> KeyframeHolder::getAllKeyframes() const {
    return keyframes_;
}
void KeyframeHolder::addKeyframe(const Keyframe& keyframe) {
    keyframes_.push_back(keyframe);
}

void KeyframeHolder::addKeyframeImage(const cv::Mat& keyframe_image) {
    keyframe_images_.push_back(keyframe_image);
}

const cv::Mat& KeyframeHolder::getKeyframeImage(int keyframe_index) {
    if (keyframe_index < 0 || keyframe_index >= (int)keyframe_images_.size()) {
        throw std::out_of_range("KeyframeHolder: Keyframe index out of range");
    }
    return keyframe_images_.at(keyframe_index);
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
        int nFeatures = 200;
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

    std::vector<int> vLappingArea = {0, image.cols};
    // cv::Mat mask = cv::noArray();  // Empty mask

    if (!orb_extractor_) {
        RCLCPP_ERROR(rclcpp::get_logger("FeatureExtraction"), "ERROR: ORBextractor is NULL!");
        return;
    } 

    orb_extractor_->operator()(image, noArray(), keypoints, descriptors, vLappingArea);
    // RCLCPP_INFO(rclcpp::get_logger("FeatureExtraction"), "Extracted %lu keypoints.", keypoints.size());

} 

/** Match ORB Features */
vector<DMatch> FeatureExtraction::matchFeatures(
    // for binary string descriptors
    const Mat& descriptors1, const Mat& descriptors2) {
    // RCLCPP_INFO(rclcpp::get_logger("FeatureExtraction"), "Matching Features...");
    if (descriptors1.empty() || descriptors2.empty()) {
        RCLCPP_WARN(rclcpp::get_logger("FeatureExtraction"), "BFMatcher: One or both descriptor matrices are empty!");
        return {};  // Return empty vector
    }
    
    BFMatcher matcher(NORM_HAMMING, true);  // Brute-force matcher
    vector<DMatch> matches;
    matcher.match(descriptors1, descriptors2, matches);

    std::sort(matches.begin(), matches.end(), [](const DMatch &a, const DMatch &b) {
        return a.distance < b.distance;
    });

    // double hamming_threshold = 50.0; // Keep only good matches (adjust this)
    // vector<DMatch> good_matches;

    // for (const auto& match : matches) {
    //     if (match.distance < hamming_threshold) {
    //         good_matches.push_back(match);
    //     }
    // }

    size_t num_to_keep = matches.size() * 0.7; 
    vector<DMatch> best_matches(matches.begin(), matches.begin() + num_to_keep);

    // **Return the better-filtered list**
    return best_matches;
}

vector<DMatch> FeatureExtraction::matchFeaturesFLANN(const Mat& descriptors1, const Mat& descriptors2) {
    // for floating point descriptors such as SURF or SIFT
    if (descriptors1.empty() || descriptors2.empty()) {
        RCLCPP_WARN(rclcpp::get_logger("FeatureExtraction"), "FLANN Matching: One or both descriptor matrices are empty!");
        return {};  // Return empty vector
    }

    Mat desc1, desc2;
    descriptors1.convertTo(desc1, CV_32F);
    descriptors2.convertTo(desc2, CV_32F);

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

bool FeatureExtraction::computeRelativePose(const vector<DMatch>& matches, 
    const vector<KeyPoint>& keypoints1, 
    const vector<KeyPoint>& keypoints2, 
    Pose2 & visualPose) {
        
    vector<Point2f> points1, points2;
    for (const auto& match : matches) {
    points1.push_back(keypoints1[match.queryIdx].pt);
    points2.push_back(keypoints2[match.trainIdx].pt);
    }

    cv::Mat K = (cv::Mat_<double>(3, 3) <<  910.1874389648438,  0.0,                623.305908203125,  // fx,  0, cx
                        0.0,                910.4207763671875, 378.3083190917969,  //  0, fy, cy
                        0.0,                 0.0,                1.0);  //  0,  0,  1
    Mat E = findEssentialMat(points1, points2, K, RANSAC, 0.999, 1.0 ); // confidence, pixel error
    Mat R, t;
    recoverPose(E, points1, points2, R, t);

    double theta = atan2(R.at<double>(1, 0), R.at<double>(0, 0));
    visualPose = Pose2(t.at<double>(0, 0), t.at<double>(1, 0), theta);

    return true; // Success
    }
}  // namespace camera_utils
