#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "slam_toolbox/toolbox_types.hpp"
// #include "karto_sdk/Karto.h"
// #include "karto_sdk/Mapper.h"
#include "slam_toolbox/camera_loop_closure_server.hpp"

namespace loop_closure_assistant
{
using namespace toolbox_types;  // NOLINT
using namespace karto;

CameraLoopClosureServer::CameraLoopClosureServer(
    std::shared_ptr<rclcpp::Node> node,
    karto::Mapper *mapper)
    : node_(node), mapper_(mapper){

    RCLCPP_INFO(node_->get_logger(), "Initializing Camera Loop Closure Server...");

    // tfB_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);
    // tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    // tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, node_);

    ssCameraLoopClosure_ = node_->create_service<slam_toolbox::srv::CameraLoopClosure>(
        "slam_toolbox/camera_loop_closure",
        std::bind(&CameraLoopClosureServer::CameraLoopClosureCallback, this,
            std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    map_frame_ = node->get_parameter("map_frame").as_string();
}

void CameraLoopClosureServer::setMapper(karto::Mapper * mapper)
{
  mapper_ = mapper;
}

bool CameraLoopClosureServer::CameraLoopClosureCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<slam_toolbox::srv::CameraLoopClosure::Request> req,
    std::shared_ptr<slam_toolbox::srv::CameraLoopClosure::Response> resp) {
    
    Pose2 currentPose(req->current_x, req->current_y, req->current_z);
    Pose2 matchedPose(req->target_x, req->target_y, req->target_z);
    Pose2 relativePose = matchedPose - currentPose;


    Vertex<LocalizedRangeScan>* sourceVertex = mapper_->GetGraph()->FindNearByScan(karto::Name("Custom Described Lidar"), currentPose);
    Vertex<LocalizedRangeScan>* targetVertex = mapper_->GetGraph()->FindNearByScan(karto::Name("Custom Described Lidar"), matchedPose);
    
    if (!sourceVertex || !targetVertex) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to find corresponding scans for camera loop closure.");
        return false;
    }
    
    LocalizedRangeScan* sourceScan = sourceVertex->GetObject();
    LocalizedRangeScan* targetScan = targetVertex->GetObject();
    
    // Define a covariance matrix for uncertainty (identity for now)
    Matrix3 visualCovariance;
    visualCovariance.SetToIdentity();  
    
    mapper_->GetGraph()->ProcessLinkScans(sourceScan, targetScan, relativePose, visualCovariance, true);
    mapper_->GetGraph()->CorrectPoses();

    //RCLCPP_INFO(node_->get_logger(), "Loop closure successfully executed between keyframe %zu and Keyframe %d!", num_keyframes - 1, best_match_index);
    
    // RCLCPP_INFO(node_->get_logger(), "Candidate Pose: (%.3f, %.3f, %.3f)", 
    // currentPose.GetX(), currentPose.GetY(), currentPose.GetHeading());

    // RCLCPP_INFO(node_->get_logger(), "Matched Pose: (%.3f, %.3f, %.3f)", 
    //             matchedPose.GetX(), matchedPose.GetY(), matchedPose.GetHeading());

    resp->success = true;
    return true;

}

}  // namespace loop_closure_assistant