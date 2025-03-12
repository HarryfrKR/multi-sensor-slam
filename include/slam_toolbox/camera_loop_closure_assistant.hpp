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
#include "slam_toolbox/camera_feature_extraction_node.hpp"


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
    CameraLoopClosureAssistant(
        rclcpp::Node::SharedPtr node, 
        karto::Mapper *mapper, 
        std::shared_ptr<camera_utils::KeyframeHolder> keyframe_holder);

    void publishGraph();
    void setMapper(karto::Mapper * mapper);
    /**
     * Callback for manual loop closure detection (Service)
     */
    // bool manualLoopClosureCallback(
    //     const std::shared_ptr<rmw_request_id_t> request_header,
    //     const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
    //     std::shared_ptr<std_srvs::srv::Trigger::Response> resp);

private:
    void automaticLoopClosure();
    
    std::vector<std::unique_ptr<boost::thread>> threads_;
    rclcpp::Node::SharedPtr node_; 
    karto::Mapper *mapper_;
    karto::ScanSolver * solver_;
    std::shared_ptr<CameraFeatureExtractionNode> camera_feature_extractor_;
    // camera_utils::FeatureExtraction feature_extractor_; 
    std::shared_ptr<camera_utils::KeyframeHolder> keyframe_holder_; 
    std::unique_ptr<tf2_ros::TransformBroadcaster> tfB_; 
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr ssLoopClosure_;
    rclcpp::TimerBase::SharedPtr loop_closure_timer_; 

    std::string map_frame_;
};

}  // namespace loop_closure_assistant

#endif  // CAMERA_LOOP_CLOSURE_ASSISTANT_HPP_
