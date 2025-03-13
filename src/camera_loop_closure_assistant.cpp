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
    : node_(node), mapper_(mapper), keyframe_holder_(keyframe_holder) {
    
    RCLCPP_INFO(node_->get_logger(), "Initializing Camera Loop Closure Assistant...");

    tfB_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);
    solver_ = mapper_->getScanSolver();
    
    auto camera_feature_extractor_= std::make_shared<CameraFeatureExtractionNode>(keyframe_holder_);
    ssLoopClosure_ = node_->create_service<std_srvs::srv::Trigger>(
        "slam_toolbox/manual_camera_loop_closure",
        std::bind(&CameraLoopClosureAssistant::manualLoopClosureCallback, this,
        std::placeholders::_1, std::placeholders::_2, std::placeholders::_3)); 
    
    threads_.push_back(std::make_unique<boost::thread>(
        [camera_feature_extractor_]() {
            rclcpp::spin(camera_feature_extractor_);
        }
    ));

    // Start automatic loop closure detection timer (Runs every 5 seconds)
    loop_closure_timer_ = node_->create_wall_timer(
        std::chrono::seconds(5),
        std::bind(&CameraLoopClosureAssistant::automaticLoopClosure, this)
    );

    map_frame_ = node->get_parameter("map_frame").as_string();
}


void CameraLoopClosureAssistant::setMapper(karto::Mapper * mapper)
{
  mapper_ = mapper;
}

bool CameraLoopClosureAssistant::transformPoseToLaserFrame(
  const Pose2& pose_in_camera, Pose2& pose_in_laser) {

  // Manually defined transform from camera to laser (from calculations)
  tf2::Transform tf_camera_to_laser_tf;
  tf_camera_to_laser_tf.setOrigin(tf2::Vector3(0.002, 0.0, 0.01));
  tf2::Quaternion q;
  q.setRPY(0, 0, 0);  // No rotation
  tf_camera_to_laser_tf.setRotation(q);

  // Convert pose from camera frame to laser frame
  tf2::Transform tf_pose_camera;
  tf_pose_camera.setOrigin(tf2::Vector3(pose_in_camera.GetX(), pose_in_camera.GetY(), 0.04));
  q.setRPY(0, 0, pose_in_camera.GetHeading());
  tf_pose_camera.setRotation(q);

  // RCLCPP_INFO(node_->get_logger(), "Manual TF used: pose in camera (%.3f, %.3f, %.3f)", 
  //             tf_pose_camera.getOrigin().x(), 
  //             tf_pose_camera.getOrigin().y(), 
  //             tf_pose_camera.getOrigin().z());

  // Compute final pose in laser frame
  tf2::Transform tf_pose_laser = tf_camera_to_laser_tf * tf_pose_camera;
  

  // Convert back to Pose2 format
  pose_in_laser = Pose2(tf_pose_laser.getOrigin().x(),
                         tf_pose_laser.getOrigin().y(),
                         tf2::getYaw(tf_pose_laser.getRotation()));

  // RCLCPP_INFO(node_->get_logger(), "Manual TF used: pose in laser (%.3f, %.3f, %.3f)", 
  //             tf_pose_laser.getOrigin().x(), 
  //             tf_pose_laser.getOrigin().y(), 
  //             tf_pose_laser.getOrigin().z());

  return true;

}

void CameraLoopClosureAssistant::automaticLoopClosure() {
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

    size_t num_keyframes = keyframe_holder_->size();
    if (num_keyframes == 0) { return false; }
    RCLCPP_INFO(node_->get_logger(), "Loop Closure Assistant - Total keyframes: %zu", num_keyframes);

    const auto& candidate_keyframe = keyframe_holder_->getKeyframe(num_keyframes - 1);

    if (candidate_keyframe.descriptors.empty()) {
        RCLCPP_ERROR(node_->get_logger(), "Candidate keyframe descriptors are empty!");
        resp->success = false;
        return false;
    }
    
    const std::vector<KeyPoint>& keypoints1 = candidate_keyframe.keypoints;
    const Mat& descriptors1 = candidate_keyframe.descriptors;

    int best_match_index = -1;
    int max_matches = 10;
    int matchThreshold = 40;
    vector<DMatch> best_matches;

    // Search for the best matching past keyframe
    for (size_t i = 0; i < num_keyframes - 1; i++) {

        if (i >= keyframe_holder_->size()) {
            RCLCPP_ERROR(node_->get_logger(), "Keyframe index out of bounds: %zu (size: %zu)", i, keyframe_holder_->size());
            resp->success = false;
            resp->message = "Keyframe index out of bounds.";
            return false;
        }
        const auto& past_keyframe = keyframe_holder_->getKeyframe(i);
        const std::vector<KeyPoint>& keypoints2 = past_keyframe.keypoints;
        const Mat& descriptors2 = past_keyframe.descriptors;

        // Use BruteForce Matcher since ORB binary string descriptors
        vector<DMatch> matches = feature_extractor_->matchFeatures(descriptors1, descriptors2);

        if (matches.size() > max_matches) {
            max_matches = matches.size();
            best_match_index = i;
            best_matches = matches;
        }
    }

    if (best_match_index == -1 || max_matches < matchThreshold) {
        RCLCPP_WARN(node_->get_logger(), "No good keyframe match found for loop closure. (Threshold %d : Best match: %d)", matchThreshold, max_matches);
        resp->success = false;
        resp->message = "No loop closure detected.";
        return false;
    }

    const auto& matched_keyframe = keyframe_holder_->getKeyframe(best_match_index);
    RCLCPP_INFO(node_->get_logger(), "Best match found: Current Keyframe %zu and Keyframe %d with %d keypoints", num_keyframes - 1, best_match_index, max_matches);

    const std::vector<KeyPoint>& keypoints_best = matched_keyframe.keypoints;
    const Mat& descriptors_best = matched_keyframe.descriptors;

    vector<DMatch> good_matches;
    double hamming_threshold = 50; 

    // if Hamming dist low, goot match
    for (const auto& match : best_matches) {
        if (match.distance < hamming_threshold) {
            good_matches.push_back(match);
        }
    }

    if (good_matches.size() < 8) {
        RCLCPP_WARN(node_->get_logger(), "Too few reliable matches after filtering : %zu good matches", good_matches.size());
        resp->success = false;
        resp->message = "Too few matches for loop closure.";
        return false;
    }

    Pose2 visualPose;
    if (feature_extractor_->computeRelativePose(good_matches, keypoints1, keypoints_best, visualPose)) {
        
        double translation_threshold = 0.2;  // cm 
        double rotation_threshold = 0.1;  // rad

        double translation_magnitude = sqrt(pow(visualPose.GetX(), 2) + pow(visualPose.GetY(), 2));
        double rotation_change = fabs(visualPose.GetHeading());

        if (translation_magnitude < translation_threshold || rotation_change < rotation_threshold) {
            RCLCPP_INFO(node_->get_logger(), "Loop closure verified! Translation: %.2fm, Rotation: %.2frad", 
                       translation_magnitude, rotation_change);
            Pose2 candidatePoseTransform, matchedPoseTransform;
            Pose2 candidatePose = candidate_keyframe.estimated_robot_pose;
            Pose2 matchedPose = matched_keyframe.estimated_robot_pose;
          
            transformPoseToLaserFrame(candidatePose, candidatePoseTransform);
            transformPoseToLaserFrame(matchedPose, matchedPoseTransform);

            Vertex<LocalizedRangeScan>* sourceVertex = mapper_->GetGraph()->FindNearByScan(karto::Name("Custom Described Lidar"), candidatePoseTransform);
            Vertex<LocalizedRangeScan>* targetVertex = mapper_->GetGraph()->FindNearByScan(karto::Name("Custom Described Lidar"), matchedPoseTransform);
            
            if (!sourceVertex || !targetVertex) {
                RCLCPP_ERROR(node_->get_logger(), "Failed to find corresponding scans for camera loop closure.");
                return false;
            }
            
            LocalizedRangeScan* sourceScan = sourceVertex->GetObject();
            LocalizedRangeScan* targetScan = targetVertex->GetObject();
            
            // Define a covariance matrix for uncertainty (identity for now)
            Matrix3 visualCovariance;
            visualCovariance.SetToIdentity();  
            
            mapper_->GetGraph()->ProcessLinkScans(sourceScan, targetScan, visualPose, visualCovariance, true);
            mapper_->GetGraph()->CorrectPoses();

            RCLCPP_INFO(node_->get_logger(), "Loop closure successfully executed!");

            resp->success = true;
            resp->message = "Loop closure detected and processed.";
            return true;
        }
    }

    resp->success = false;
    resp->message = "Loop closure rejected (geometric check failed).";
    return false;
}
}  // namespace loop_closure_assistant
