#include <unordered_map>
#include <memory>
#include <slam_toolbox/camera_loop_closure_assistant.hpp>

// #include <slam_toolbox/camera_utils.hpp>
// #include <slam_toolbox/ORBextractor.h>
// #include <slam_toolbox/camera_utils.hpp>
// #include <std_srvs/srv/trigger.hpp>

using namespace std;
using namespace cv;
using namespace orb;
using namespace karto;

namespace loop_closure_assistant {

CameraLoopClosureAssistant::CameraLoopClosureAssistant(
    std::shared_ptr<rclcpp::Node> node, 
    karto::Mapper *mapper,
    std::shared_ptr<camera_utils::KeyframeHolder> keyframe_holder)
    : node_(node), mapper_(mapper), keyframe_holder_(keyframe_holder)  {
    
    RCLCPP_INFO(node_->get_logger(), "Initializing Camera Loop Closure Assistant...");

    ssLoopClosure_ = node_->create_service<std_srvs::srv::Trigger>(
        "slam_toolbox/manual_camera_loop_closure",
        std::bind(&CameraLoopClosureAssistant::manualLoopClosureCallback, this,
        std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));    

    // 🔹 Start automatic loop closure detection timer (Runs every 5 seconds)
    loop_closure_timer_ = node_->create_wall_timer(
        std::chrono::seconds(10),
        std::bind(&CameraLoopClosureAssistant::automaticLoopClosure, this)
    );
}

void CameraLoopClosureAssistant::automaticLoopClosure() {
    RCLCPP_INFO(node_->get_logger(), "Running automatic loop closure detection...");

    std_srvs::srv::Trigger::Request::SharedPtr req = std::make_shared<std_srvs::srv::Trigger::Request>();
    std_srvs::srv::Trigger::Response::SharedPtr resp = std::make_shared<std_srvs::srv::Trigger::Response>();

    if (manualLoopClosureCallback(nullptr, req, resp)) {
        RCLCPP_INFO(node_->get_logger(), "Loop closure successful: %s", resp->message.c_str());
    } else {
        RCLCPP_WARN(node_->get_logger(), "Loop closure failed: %s", resp->message.c_str());
    }
}

bool CameraLoopClosureAssistant::manualLoopClosureCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
    std::shared_ptr<std_srvs::srv::Trigger::Response> resp) {

    // Check the number of stored keyframes
    size_t num_keyframes = keyframe_holder_->size();
    RCLCPP_INFO(node_->get_logger(), "Loop Closure Assistant - Total keyframes: %zu", num_keyframes);

    if (num_keyframes < 2) {
        RCLCPP_WARN(node_->get_logger(), "Not enough keyframes for loop closure.");
        resp->success = false;
        resp->message = "Not enough keyframes.";
        return false;
    }

    // Fetch the latest keyframe
    const auto& current_keyframe = keyframe_holder_->getKeyframe(num_keyframes - 1);
    RCLCPP_INFO(node_->get_logger(), "Processing latest keyframe: %zu with %lu keypoints", num_keyframes - 1, current_keyframe.keypoints.size());

    const std::vector<KeyPoint>& keypoints1 = current_keyframe.keypoints;
    const Mat& descriptors1 = current_keyframe.descriptors;

    int best_match_index = -1;
    int max_matches = 0;
    vector<DMatch> best_matches;

    // Search for the best matching past keyframe
    for (size_t i = 0; i < num_keyframes - 1; i++) {
        const auto& candidate_keyframe = keyframe_holder_->getKeyframe(i);
        const std::vector<KeyPoint>& keypoints2 = candidate_keyframe.keypoints;
        const Mat& descriptors2 = candidate_keyframe.descriptors;

        // Log the comparison between keyframes
        RCLCPP_INFO(node_->get_logger(), "Comparing keyframe %zu (current) with keyframe %zu (past)", num_keyframes - 1, i);

        // Use BruteForce Matcher instead of FLANN
        vector<DMatch> matches = feature_extractor_.matchFeatures(descriptors1, descriptors2);

        RCLCPP_INFO(node_->get_logger(), " Keyframe %zu found %lu matches with keyframe %zu", num_keyframes - 1, matches.size(), i);

        if (matches.size() > max_matches) {
            max_matches = matches.size();
            best_match_index = i;
            best_matches = matches;
        }
    }

    // Ensure there are enough matches
    RCLCPP_INFO(node_->get_logger(), "Best match index: %d, Matches: %d", best_match_index, max_matches);

    if (best_match_index == -1 || max_matches < 30) {
        RCLCPP_WARN(node_->get_logger(), "No good keyframe match found for loop closure. (Best match: %d)", max_matches);
        resp->success = false;
        resp->message = "No loop closure detected.";
        return false;
    }

    // Retrieve best-matching keyframe
    const auto& best_match_keyframe = keyframe_holder_->getKeyframe(best_match_index);
    RCLCPP_INFO(node_->get_logger(), "🔹 Best match found: Keyframe %d with %lu keypoints", best_match_index, best_match_keyframe.keypoints.size());

    const std::vector<KeyPoint>& keypoints_best = best_match_keyframe.keypoints;
    const Mat& descriptors_best = best_match_keyframe.descriptors;

    // ✅ Filter matches using RANSAC
    vector<DMatch> filtered_matches = feature_extractor_.filterMatchesWithFundamentalMatrix(
        best_matches, keypoints1, keypoints_best);

    RCLCPP_INFO(node_->get_logger(), "Filtered matches count after RANSAC: %lu", filtered_matches.size());

    if (filtered_matches.size() < 20) {
        RCLCPP_WARN(node_->get_logger(), "Filtered matches too low for reliable loop closure.");
        resp->success = false;
        resp->message = "Too few filtered matches.";
        return false;
    }

    // Compute pose transformation
    Pose2 visualPose;
    if (computeRelativePose(filtered_matches, keypoints1, keypoints_best, visualPose)) {
        
        RCLCPP_INFO(node_->get_logger(), "Created visual constraint scan with pose: (%f, %f, %f)",
                    visualPose.GetX(), visualPose.GetY(), visualPose.GetHeading());

        // 🔹 Add visual constraint to pose graph
        karto::VisualConstraintScan* visualScan = new karto::VisualConstraintScan(karto::Name("camera"), visualPose);

        mapper_->GetGraph()->ProcessVisualConstraint(visualScan);
        mapper_->CorrectPoses();

        // delete visualScan;

        RCLCPP_INFO(node_->get_logger(), "Loop closure successfully executed!");

        resp->success = true;
        resp->message = "Camera loop closure executed successfully.";
        return true;
    }

    RCLCPP_ERROR(node_->get_logger(), "Failed to compute relative pose.");
    resp->success = false;
    resp->message = "Failed to compute relative pose.";
    return false;
}

bool CameraLoopClosureAssistant::computeRelativePose(const vector<DMatch>& matches, 
                            const vector<KeyPoint>& keypoints1, 
                            const vector<KeyPoint>& keypoints2, 
                            Pose2 & visualPose) {
    vector<Point2f> points1, points2;
    for (const auto& match : matches) {
        points1.push_back(keypoints1[match.queryIdx].pt);
        points2.push_back(keypoints2[match.trainIdx].pt);
    }

    Mat E = findEssentialMat(points1, points2, 1.0, Point2d(0, 0), RANSAC);
    Mat R, t;
    recoverPose(E, points1, points2, R, t);

    // Convert R, t to Pose2
    double theta = atan2(R.at<double>(1, 0), R.at<double>(0, 0));
    visualPose = Pose2(t.at<double>(0, 0), t.at<double>(1, 0), theta);

    return true; // Success
}


}  // namespace loop_closure_assistant
