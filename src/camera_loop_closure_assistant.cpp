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

    // ssClear_manual_ = node_->create_service<slam_toolbox::srv::Clear>(
    //     "slam_toolbox/clear_changes", std::bind(&LoopClosureAssistant::clearChangesCallback, 
    //     this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    
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
        std::chrono::seconds(10),
        std::bind(&CameraLoopClosureAssistant::automaticLoopClosure, this)
    );

    map_frame_ = node->get_parameter("map_frame").as_string();
}


void CameraLoopClosureAssistant::setMapper(karto::Mapper * mapper)
{
  mapper_ = mapper;
}

/*****************************************************************************/
void CameraLoopClosureAssistant::publishGraph()
/*****************************************************************************/
{
  // interactive_server_->clear();
  auto graph = solver_->getGraph();

  if (graph->size() == 0) {
    return;
  }

  RCLCPP_DEBUG(node_->get_logger(), "Graph size: %zu", graph->size());
//   bool interactive_mode = false;
//   {
//     boost::mutex::scoped_lock lock(interactive_mutex_);
//     interactive_mode = interactive_mode_;
//   }

  const auto & vertices = mapper_->GetGraph()->GetVertices();
  const auto & edges = mapper_->GetGraph()->GetEdges();
  const auto & localization_vertices = mapper_->GetLocalizationVertices();

  int first_localization_id = std::numeric_limits<int>::max();
  if (!localization_vertices.empty()) {
    first_localization_id = localization_vertices.front().vertex->GetObject()->GetUniqueId();
  }

  visualization_msgs::msg::MarkerArray marray;

  // clear existing markers to account for any removed nodes
  visualization_msgs::msg::Marker clear;
  clear.header.stamp = node_->now();
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  marray.markers.push_back(clear);

  visualization_msgs::msg::Marker m = vis_utils::toMarker(map_frame_, "slam_toolbox", 0.1, node_);

  // add map nodes
  for (const auto & sensor_name : vertices) {
    for (const auto & vertex : sensor_name.second) {
      m.color.g = vertex.first < first_localization_id ? 0.0 : 1.0;
      const auto & pose = vertex.second->GetObject()->GetCorrectedPose();
      m.id = vertex.first;
      m.pose.position.x = pose.GetX();
      m.pose.position.y = pose.GetY();
      marray.markers.push_back(m);

    //   if (interactive_mode && enable_interactive_mode_) {
    //     visualization_msgs::msg::InteractiveMarker int_marker =
    //       vis_utils::toInteractiveMarker(m, 0.3, node_);
    //     interactive_server_->insert(int_marker,
    //       std::bind(
    //       &LoopClosureAssistant::processInteractiveFeedback,
    //       this, std::placeholders::_1));
    //   } else {
    //     marray.markers.push_back(m);
    //   }
    }
  }

  // add line markers for graph edges
  visualization_msgs::msg::Marker edges_marker;
  edges_marker.header.frame_id = map_frame_;
  edges_marker.header.stamp = node_->now();
  edges_marker.id = 0;
  edges_marker.ns = "slam_toolbox_edges";
  edges_marker.action = visualization_msgs::msg::Marker::ADD;
  edges_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  edges_marker.pose.orientation.w = 1;
  edges_marker.scale.x = 0.05;
  edges_marker.color.b = 1;
  edges_marker.color.a = 1;
  edges_marker.lifetime = rclcpp::Duration::from_seconds(0);
  edges_marker.points.reserve(edges.size() * 2);

  visualization_msgs::msg::Marker localization_edges_marker;
  localization_edges_marker.header.frame_id = map_frame_;
  localization_edges_marker.header.stamp = node_->now();
  localization_edges_marker.id = 1;
  localization_edges_marker.ns = "slam_toolbox_edges";
  localization_edges_marker.action = visualization_msgs::msg::Marker::ADD;
  localization_edges_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  localization_edges_marker.pose.orientation.w = 1;
  localization_edges_marker.scale.x = 0.05;
  localization_edges_marker.color.g = 1;
  localization_edges_marker.color.b = 1;
  localization_edges_marker.color.a = 1;
  localization_edges_marker.lifetime = rclcpp::Duration::from_seconds(0);
  localization_edges_marker.points.reserve(localization_vertices.size() * 3);

  for (const auto & edge : edges) {
    int source_id = edge->GetSource()->GetObject()->GetUniqueId();
    const auto & pose0 = edge->GetSource()->GetObject()->GetCorrectedPose();
    geometry_msgs::msg::Point p0;
    p0.x = pose0.GetX();
    p0.y = pose0.GetY();

    int target_id = edge->GetTarget()->GetObject()->GetUniqueId();
    const auto & pose1 = edge->GetTarget()->GetObject()->GetCorrectedPose();
    geometry_msgs::msg::Point p1;
    p1.x = pose1.GetX();
    p1.y = pose1.GetY();

    if (source_id >= first_localization_id || target_id >= first_localization_id) {
      localization_edges_marker.points.push_back(p0);
      localization_edges_marker.points.push_back(p1);
    } else {
      edges_marker.points.push_back(p0);
      edges_marker.points.push_back(p1);
    }
  }

  marray.markers.push_back(edges_marker);
  marray.markers.push_back(localization_edges_marker);

  // if disabled, clears out old markers
  // interactive_server_->applyChanges();
  marker_publisher_->publish(marray);
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
    if (num_keyframes == 0) { return false; }
    RCLCPP_INFO(node_->get_logger(), "Loop Closure Assistant - Total keyframes: %zu", num_keyframes);

    const auto& candidate_keyframe = keyframe_holder_->getKeyframe(num_keyframes - 1);
    // RCLCPP_INFO(node_->get_logger(), "Processing latest keyframe: %zu with %lu keypoints", num_keyframes - 1, candidate_keyframe.keypoints.size());

    if (candidate_keyframe.descriptors.empty()) {
        RCLCPP_ERROR(node_->get_logger(), "Candidate keyframe descriptors are empty!");
        resp->success = false;
        return false;
    }
    
    const std::vector<KeyPoint>& keypoints1 = candidate_keyframe.keypoints;
    const Mat& descriptors1 = candidate_keyframe.descriptors;

    int best_match_index = -1;
    int max_matches = 10;
    int matchThreshold = 80;
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

        RCLCPP_INFO(node_->get_logger(), "Comparing keyframe %zu (current) with keyframe %zu (past)", num_keyframes - 1, i);

        // Use BruteForce Matcher since ORB binary string descriptors
        vector<DMatch> matches = feature_extractor_->matchFeatures(descriptors1, descriptors2);

        // RCLCPP_INFO(node_->get_logger(), " Keyframe %zu found %lu matches with keyframe %zu", num_keyframes - 1, matches.size(), i);

        if (matches.size() > max_matches) {
            max_matches = matches.size();
            best_match_index = i;
            best_matches = matches;
        }
    }

    // RCLCPP_INFO(node_->get_logger(), "Best match index: %d, Matches: %d", best_match_index, max_matches);

    if (best_match_index == -1 || max_matches < matchThreshold) {
        RCLCPP_WARN(node_->get_logger(), "No good keyframe match found for loop closure. (Best match: %d)", max_matches);
        resp->success = false;
        resp->message = "No loop closure detected.";
        return false;
    }

    const auto& matched_keyframe = keyframe_holder_->getKeyframe(best_match_index);
    RCLCPP_INFO(node_->get_logger(), "Best match found: Current Keyframe %zu and Keyframe %d with %lu keypoints", num_keyframes - 1, best_match_index, matched_keyframe.keypoints.size());

    const std::vector<KeyPoint>& keypoints_best = matched_keyframe.keypoints;
    const Mat& descriptors_best = matched_keyframe.descriptors;

    vector<DMatch> good_matches;
    double hamming_threshold = 30; 

    // if Hamming dist low, goot match
    for (const auto& match : best_matches) {
        if (match.distance < hamming_threshold) {
            good_matches.push_back(match);
        }
    }

    if (good_matches.size() < 20) {
        RCLCPP_WARN(node_->get_logger(), "Too few reliable matches after filtering.");
        resp->success = false;
        resp->message = "Too few matches for loop closure.";
        return false;
    }

    Pose2 visualPose;
    if (feature_extractor_->computeRelativePose(good_matches, keypoints1, keypoints_best, visualPose)) {
        
        double translation_threshold = 0.2;  // 30 cm movement
        double rotation_threshold = 0.1;    // ~8.5 degrees

        double translation_magnitude = sqrt(pow(visualPose.GetX(), 2) + pow(visualPose.GetY(), 2));
        double rotation_change = fabs(visualPose.GetHeading());

        if (translation_magnitude > translation_threshold || rotation_change > rotation_threshold) {
            RCLCPP_INFO(node_->get_logger(), "Loop closure verified! Translation: %.2fm, Rotation: %.2frad", 
                       translation_magnitude, rotation_change);
            
            Pose2 candidatePose = candidate_keyframe.estimated_robot_pose;
            Pose2 matchedPose = matched_keyframe.estimated_robot_pose;
            Vertex<LocalizedRangeScan>* sourceVertex = mapper_->GetGraph()->FindNearByScan(karto::Name("laser"), candidatePose);
            Vertex<LocalizedRangeScan>* targetVertex = mapper_->GetGraph()->FindNearByScan(karto::Name("laser"), matchedPose);
            
            if (!sourceVertex || !targetVertex) {
                RCLCPP_ERROR(node_->get_logger(), "Failed to find corresponding scans for camera loop closure.");
                return false;
            }
            
            LocalizedRangeScan* sourceScan = sourceVertex->GetObject();
            LocalizedRangeScan* targetScan = targetVertex->GetObject();
            
            // Define a covariance matrix for uncertainty (identity for now)
            Matrix3 visualCovariance;
            visualCovariance.SetToIdentity();  
            
            // Link camera constraint in the pose graph
            // mapper_->GetGraph()->LinkScans(sourceScan, targetScan, visualPose, visualCovariance, true);

            RCLCPP_INFO(node_->get_logger(), "Loop closure successfully executed!");

            resp->success = true;
            resp->message = "Loop closure detected and processed.";
            return true;
        }
    }

    // If no valid loop closure was found
    // RCLCPP_WARN(node_->get_logger(), "No significant motion detected. Skipping loop closure.");
    resp->success = false;
    resp->message = "Loop closure rejected (geometric check failed).";
    return false;
}

}  // namespace loop_closure_assistant
