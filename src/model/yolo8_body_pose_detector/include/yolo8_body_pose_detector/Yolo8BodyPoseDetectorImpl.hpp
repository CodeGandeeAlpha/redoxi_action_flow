#include <chrono>
#include <map>

#include <tbb/concurrent_queue.h>
#include <tbb/task_group.h>

#include <yolo8_body_pose_detector/Yolo8BodyPoseDetectorNode.hpp>
#include <redoxi_common_cpp/ros_utils/StampedImagePub.hpp>
#include <redoxi_common_nodes/port_handlers/PullProcessSendHandler.hpp>
#include <redoxi_common_nodes/port_handlers/PullProcessReplyHandler.hpp>
#include <std_msgs/msg/string.hpp>

namespace redoxi_works::model_nodes
{

struct Yolo8BodyPoseDetectorNode::Impl {
    using Node_t = Yolo8BodyPoseDetectorNode;
    Impl() = default;

    // this limits the number of inference resources, like GPU, NPU
    // each task must first acquire a resource from this pool, then do inference
    tbb::concurrent_bounded_queue<InferenceResource_t> inference_resource_pool;

    // visualization publisher
    std::shared_ptr<StampedImagePub> pub_visualization;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_detection_done;

    // pull input, work on it and then send output
    using PullProcessSendHandler_t = redoxi_works::port_handlers::PullProcessSendHandler<Node_t::ByImageRequest::InputPort_t::MasterSpec_t,
                                                                                         Node_t::ByImageRequest::OutputPort_t::MasterSpec_t,
                                                                                         InferenceResource_t>;
    std::shared_ptr<PullProcessSendHandler_t> work_then_send_handler;

    // pull input, work on it and then reply
    using PullProcessReplyHandler_t = redoxi_works::port_handlers::PullProcessReplyHandler<Node_t::ByDetectionRequest::InputPort_t::MasterSpec_t,
                                                                                           InferenceResource_t>;
    std::shared_ptr<PullProcessReplyHandler_t> work_then_reply_handler;
};
} // namespace redoxi_works::model_nodes
