

#ifndef SLAM_TOOLBOX__CAMERA_LOOP_CLOSURE_ASSISTANT_HPP_
#define SLAM_TOOLBOX__CAMERA_LOOP_CLOSURE_ASSISTANT_HPP_

#include <thread>
#include <string>
#include <functional>
#include <memory>
#include <map>

#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"
#include "rclcpp/rclcpp.hpp"
#include "karto_sdk/Mapper.h"
#include "karto_sdk/Karto.h"

#include "slam_toolbox/toolbox_types.hpp"
#include "slam_toolbox/visualization_utils.hpp"

namespace loop_closure_assistant
{

using namespace ::toolbox_types;  // NOLINT

class CameraLoopClosureServer
{
public:
  CameraLoopClosureServer(
    rclcpp::Node::SharedPtr node, karto::Mapper * mapper);
  void setMapper(karto::Mapper * mapper);
//   void ProcessLinkScans(karto::LocalizedRangeScan* pScan1, karto::LocalizedRangeScan* pScan2, 
//     const karto::Pose2& relativePose, const karto::Matrix3& covariance, bool is_from_camera) {
//         karto::Mapper::LinkScans(pScan1, pScan2, relativePose, covariance, is_from_camera);
//     }

private:
  bool CameraLoopClosureCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<slam_toolbox::srv::CameraLoopClosure::Request> req, 
    std::shared_ptr<slam_toolbox::srv::CameraLoopClosure::Response> resp);

  //std::unique_ptr<tf2_ros::TransformBroadcaster> tfB_;
  rclcpp::Service<slam_toolbox::srv::CameraLoopClosure>::SharedPtr ssCameraLoopClosure_;
  karto::Mapper * mapper_;
  rclcpp::Node::SharedPtr node_;
  std::string map_frame_;

};

}   // namespace loop_closure_assistant

#endif  // SLAM_TOOLBOX__CAMERA_LOOP_CLOSURE_ASSISTANT_HPP_
