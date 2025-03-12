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
    // ssLoopClosure_ = node_->create_service<std_srvs::srv::Trigger>(
    //     "slam_toolbox/manual_camera_loop_closure",
    //     std::bind(&CameraLoopClosureAssistant::manualLoopClosureCallback, this,
    //     std::placeholders::_1, std::placeholders::_2, std::placeholders::_3)); 
    
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

// /*****************************************************************************/
// void CameraLoopClosureAssistant::publishGraph()
// /*****************************************************************************/
// {
//   // interactive_server_->clear();
//   auto graph = solver_->getGraph();

//   if (graph->size() == 0) {
//     return;
//   }

//   RCLCPP_DEBUG(node_->get_logger(), "Graph size: %zu", graph->size());
// //   bool interactive_mode = false;
// //   {
// //     boost::mutex::scoped_lock lock(interactive_mutex_);
// //     interactive_mode = interactive_mode_;
// //   }

//   const auto & vertices = mapper_->GetGraph()->GetVertices();
//   const auto & edges = mapper_->GetGraph()->GetEdges();
//   const auto & localization_vertices = mapper_->GetLocalizationVertices();

//   int first_localization_id = std::numeric_limits<int>::max();
//   if (!localization_vertices.empty()) {
//     first_localization_id = localization_vertices.front().vertex->GetObject()->GetUniqueId();
//   }

//   visualization_msgs::msg::MarkerArray marray;

//   // clear existing markers to account for any removed nodes
//   visualization_msgs::msg::Marker clear;
//   clear.header.stamp = node_->now();
//   clear.action = visualization_msgs::msg::Marker::DELETEALL;
//   marray.markers.push_back(clear);

//   visualization_msgs::msg::Marker m = vis_utils::toMarker(map_frame_, "slam_toolbox", 0.1, node_);

//   // add map nodes
//   for (const auto & sensor_name : vertices) {
//     for (const auto & vertex : sensor_name.second) {
//       m.color.g = vertex.first < first_localization_id ? 0.0 : 1.0;
//       const auto & pose = vertex.second->GetObject()->GetCorrectedPose();
//       m.id = vertex.first;
//       m.pose.position.x = pose.GetX();
//       m.pose.position.y = pose.GetY();
//       marray.markers.push_back(m);

//     //   if (interactive_mode && enable_interactive_mode_) {
//     //     visualization_msgs::msg::InteractiveMarker int_marker =
//     //       vis_utils::toInteractiveMarker(m, 0.3, node_);
//     //     interactive_server_->insert(int_marker,
//     //       std::bind(
//     //       &LoopClosureAssistant::processInteractiveFeedback,
//     //       this, std::placeholders::_1));
//     //   } else {
//     //     marray.markers.push_back(m);
//     //   }
//     }
//   }

//   // add line markers for graph edges
//   visualization_msgs::msg::Marker edges_marker;
//   edges_marker.header.frame_id = map_frame_;
//   edges_marker.header.stamp = node_->now();
//   edges_marker.id = 0;
//   edges_marker.ns = "slam_toolbox_edges";
//   edges_marker.action = visualization_msgs::msg::Marker::ADD;
//   edges_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
//   edges_marker.pose.orientation.w = 1;
//   edges_marker.scale.x = 0.05;
//   edges_marker.color.b = 1;
//   edges_marker.color.a = 1;
//   edges_marker.lifetime = rclcpp::Duration::from_seconds(0);
//   edges_marker.points.reserve(edges.size() * 2);

//   visualization_msgs::msg::Marker localization_edges_marker;
//   localization_edges_marker.header.frame_id = map_frame_;
//   localization_edges_marker.header.stamp = node_->now();
//   localization_edges_marker.id = 1;
//   localization_edges_marker.ns = "slam_toolbox_edges";
//   localization_edges_marker.action = visualization_msgs::msg::Marker::ADD;
//   localization_edges_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
//   localization_edges_marker.pose.orientation.w = 1;
//   localization_edges_marker.scale.x = 0.05;
//   localization_edges_marker.color.g = 1;
//   localization_edges_marker.color.b = 1;
//   localization_edges_marker.color.a = 1;
//   localization_edges_marker.lifetime = rclcpp::Duration::from_seconds(0);
//   localization_edges_marker.points.reserve(localization_vertices.size() * 3);

//   for (const auto & edge : edges) {
//     int source_id = edge->GetSource()->GetObject()->GetUniqueId();
//     const auto & pose0 = edge->GetSource()->GetObject()->GetCorrectedPose();
//     geometry_msgs::msg::Point p0;
//     p0.x = pose0.GetX();
//     p0.y = pose0.GetY();

//     int target_id = edge->GetTarget()->GetObject()->GetUniqueId();
//     const auto & pose1 = edge->GetTarget()->GetObject()->GetCorrectedPose();
//     geometry_msgs::msg::Point p1;
//     p1.x = pose1.GetX();
//     p1.y = pose1.GetY();

//     if (source_id >= first_localization_id || target_id >= first_localization_id) {
//       localization_edges_marker.points.push_back(p0);
//       localization_edges_marker.points.push_back(p1);
//     } else {
//       edges_marker.points.push_back(p0);
//       edges_marker.points.push_back(p1);
//     }
//   }

//   marray.markers.push_back(edges_marker);
//   marray.markers.push_back(localization_edges_marker);

//   // if disabled, clears out old markers
//   // interactive_server_->applyChanges();
//   marker_publisher_->publish(marray);
// }

void CameraLoopClosureAssistant::automaticLoopClosure() {
    RCLCPP_INFO(node_->get_logger(), "Running automatic loop closure detection...");

    std_srvs::srv::Trigger::Request::SharedPtr req = std::make_shared<std_srvs::srv::Trigger::Request>();
    std_srvs::srv::Trigger::Response::SharedPtr resp = std::make_shared<std_srvs::srv::Trigger::Response>();

    // if (manualLoopClosureCallback(nullptr, req, resp)) {
    //     RCLCPP_INFO(node_->get_logger(), "Loop closure successful: %s", resp->message.c_str());
    // } else {
    //     RCLCPP_WARN(node_->get_logger(), "Loop closure failed: %s", resp->message.c_str());
    // }
}

// bool CameraLoopClosureAssistant::manualLoopClosureCallback(
//     const std::shared_ptr<rmw_request_id_t> request_header,
//     const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
//     std::shared_ptr<std_srvs::srv::Trigger::Response> resp) {

//     // Check the number of stored keyframes
//     size_t num_keyframes = keyframe_holder_->size();
//     RCLCPP_INFO(node_->get_logger(), "Loop Closure Assistant - Total keyframes: %zu", num_keyframes);

//     if (num_keyframes < 2) {
//         RCLCPP_WARN(node_->get_logger(), "Not enough keyframes for loop closure.");
//         resp->success = false;
//         resp->message = "Not enough keyframes.";
//         return false;
//     }

//     // Fetch the latest keyframe
//     const auto& current_keyframe = keyframe_holder_->getKeyframe(num_keyframes - 1);
//     RCLCPP_INFO(node_->get_logger(), "Processing latest keyframe: %zu with %lu keypoints", num_keyframes - 1, current_keyframe.keypoints.size());

//     const std::vector<KeyPoint>& keypoints1 = current_keyframe.keypoints;
//     const Mat& descriptors1 = current_keyframe.descriptors;

//     int best_match_index = -1;
//     int max_matches = 0;
//     int matchThreshold = 300;
//     vector<DMatch> best_matches;

//     // Search for the best matching past keyframe
//     for (size_t i = 0; i < num_keyframes - 1; i++) {
//         const auto& candidate_keyframe = keyframe_holder_->getKeyframe(i);
//         const std::vector<KeyPoint>& keypoints2 = candidate_keyframe.keypoints;
//         const Mat& descriptors2 = candidate_keyframe.descriptors;

//         // Log the comparison between keyframes
//         RCLCPP_INFO(node_->get_logger(), "Comparing keyframe %zu (current) with keyframe %zu (past)", num_keyframes - 1, i);

//         // Use BruteForce Matcher instead of FLANN
//         vector<DMatch> matches = feature_extractor_.matchFeatures(descriptors1, descriptors2);

//         RCLCPP_INFO(node_->get_logger(), " Keyframe %zu found %lu matches with keyframe %zu", num_keyframes - 1, matches.size(), i);

//         if (matches.size() > max_matches) {
//             max_matches = matches.size();
//             best_match_index = i;
//             best_matches = matches;
//         }
//     }

//     // Ensure there are enough matches
//     RCLCPP_INFO(node_->get_logger(), "Best match index: %d, Matches: %d", best_match_index, max_matches);

//     if (best_match_index == -1 || max_matches < matchThreshold) {
//         RCLCPP_WARN(node_->get_logger(), "No good keyframe match found for loop closure. (Best match: %d)", max_matches);
//         resp->success = false;
//         resp->message = "No loop closure detected.";
//         return false;
//     }

//     // Retrieve best-matching keyframe
//     const auto& best_match_keyframe = keyframe_holder_->getKeyframe(best_match_index);
//     RCLCPP_INFO(node_->get_logger(), "🔹 Best match found: Keyframe %d with %lu keypoints", best_match_index, best_match_keyframe.keypoints.size());

//     const std::vector<KeyPoint>& keypoints_best = best_match_keyframe.keypoints;
//     const Mat& descriptors_best = best_match_keyframe.descriptors;

//     // Filter matches using RANSAC
//     vector<DMatch> filtered_matches = feature_extractor_.filterMatchesWithFundamentalMatrix(
//         best_matches, keypoints1, keypoints_best);

//     RCLCPP_INFO(node_->get_logger(), "Filtered matches count after RANSAC: %lu", filtered_matches.size());

//     if (filtered_matches.size() < 20) {
//         RCLCPP_WARN(node_->get_logger(), "Filtered matches too low for reliable loop closure.");
//         resp->success = false;
//         resp->message = "Too few filtered matches.";
//         return false;
//     }

//     // Compute pose transformation
//     Pose2 visualPose;
//     if (computeRelativePose(filtered_matches, keypoints1, keypoints_best, visualPose)) {
        
//         RCLCPP_INFO(node_->get_logger(), "Created visual constraint scan with pose: (%f, %f, %f)",
//                     visualPose.GetX(), visualPose.GetY(), visualPose.GetHeading());

//         karto::VisualConstraintScan* visualScan = new karto::VisualConstraintScan(
//             karto::Name("camera"), visualPose, best_match_index);
                    

//         mapper_->GetGraph()->ProcessVisualConstraint(visualScan);
//         mapper_->CorrectPoses();

//         delete visualScan;

//         RCLCPP_INFO(node_->get_logger(), "Loop closure successfully executed!");

//         resp->success = true;
//         resp->message = "Camera loop closure executed successfully.";

//         publishGraph();
//         // clearMovedNodes();
//         return true;
//     }

//     RCLCPP_ERROR(node_->get_logger(), "Failed to compute relative pose.");
//     resp->success = false;
//     resp->message = "Failed to compute relative pose.";
//     return false;
// }



}  // namespace loop_closure_assistant
