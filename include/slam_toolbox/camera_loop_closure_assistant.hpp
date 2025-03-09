#ifndef CAMERA_LOOP_CLOSURE_ASSISTANT_HPP_
#define CAMERA_LOOP_CLOSURE_ASSISTANT_HPP_

#include <thread>
#include <string>
#include <functional>
#include <memory>
#include <map>
#include <vector>
#include <karto_sdk/Karto.h>

#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"
#include "rclcpp/rclcpp.hpp"
#include "interactive_markers/interactive_marker_server.hpp"
#include "interactive_markers/menu_handler.hpp"

#include "slam_toolbox/toolbox_types.hpp"
#include "slam_toolbox/visualization_utils.hpp"
#include <std_srvs/srv/trigger.hpp>
#include "slam_toolbox/camera_utils.hpp"
//#include "slam_toolbox/camera_feature_extraction_node.cpp"


namespace loop_closure_assistant {

using namespace ::toolbox_types;  // NOLINT

class CameraLoopClosureAssistant {
public:
    /**
     * Constructor for CameraLoopClosureAssistant
     * @param node Shared ROS node
     * @param mapper Pointer to the SLAM mapper
     * @param keyframe_holder Pointer to KeyframeHolder storing keyframes and features
     */
    CameraLoopClosureAssistant(rclcpp::Node::SharedPtr node, 
        karto::Mapper *mapper, 
        std::shared_ptr<camera_utils::KeyframeHolder> keyframe_holder);

    /**
     * Callback for manual loop closure detection (Service)
     */
    bool manualLoopClosureCallback(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
        std::shared_ptr<std_srvs::srv::Trigger::Response> resp);

private:
    /**
     * Runs loop closure detection every X seconds
     */
    void automaticLoopClosure();

    /**
     * Computes relative pose between two keyframes using feature matching
     */
    bool computeRelativePose(const std::vector<cv::DMatch>& matches, 
                             const std::vector<cv::KeyPoint>& keypoints1, 
                             const std::vector<cv::KeyPoint>& keypoints2, 
                             karto::Pose2 &visualPose);

    rclcpp::Node::SharedPtr node_;  // ROS Node
    karto::Mapper *mapper_;  // SLAM Mapper
    camera_utils::FeatureExtraction feature_extractor_;  // Feature extraction utility
    std::shared_ptr<camera_utils::KeyframeHolder> keyframe_holder_;  

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr ssLoopClosure_;  // Manual service for triggering loop closure
    rclcpp::TimerBase::SharedPtr loop_closure_timer_;  // Timer for automatic loop closure detection
};

}  // namespace loop_closure_assistant

#endif  // CAMERA_LOOP_CLOSURE_ASSISTANT_HPP_
