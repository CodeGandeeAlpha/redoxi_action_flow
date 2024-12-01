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
    Impl()
    {
        // fill the token queue with initial tokens
        {
            int capacity = num_detection_handler_tokens;
            detection_request_handler_tokens.set_capacity(capacity);
            for (int i = 0; i < capacity; ++i) {
                detection_request_handler_tokens.push(i);
            }
        }

        {
            int capacity = num_image_handler_tokens;
            image_request_handler_tokens.set_capacity(capacity);
            for (int i = 0; i < capacity; ++i) {
                image_request_handler_tokens.push(i);
            }
        }
    }
    tbb::task_group inference_task_group;

    // this limits the number of inference resources, like GPU, NPU
    // each task must first acquire a resource from this pool, then do inference
    tbb::concurrent_bounded_queue<InferenceResource_t> inference_resource_pool;

    // this limits the number of concurrent inference tasks
    // each task must first acquire a task id from this pool, then enqueued to wait for inference resource

    // detection request handler tokens, used for parallel task
    // detection request's order is maintained by client using goal handle
    // so it is safe to use parallel task here
    tbb::concurrent_bounded_queue<int> detection_request_handler_tokens;
    int num_detection_handler_tokens = 2;

    // image request handler tokens, used for sequential task
    // image response's order is undetermined, not safe to use parallel task here
    // so only one token is available, equivalent to sequential task
    tbb::concurrent_bounded_queue<int> image_request_handler_tokens;
    int num_image_handler_tokens = 1;

    // use parallel task to handle the detection and image request
    // so that in blocking mode, one port's missing data will not hang the other port
    bool use_parallel_task = true;

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
