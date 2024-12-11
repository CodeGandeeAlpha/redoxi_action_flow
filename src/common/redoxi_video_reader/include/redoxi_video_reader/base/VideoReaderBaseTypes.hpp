#pragma once

#include <redoxi_common_nodes/async_action_port/AsyncActionOutputPort.hpp>
#include <redoxi_common_nodes/image_ports/ImageOutputPortSpec.hpp>
#include <redoxi_common_nodes/base_nodes/OpenCloseNode.hpp>
#include <redoxi_common_cpp/redoxi_common_cpp.hpp>
#include <redoxi_common_cpp/redoxi_json_struct_conversion.hpp>
#include <redoxi_video_reader/visibility_control.h>
#include <sensor_msgs/image_encodings.hpp>
#include <json_struct/json_struct.h>

namespace redoxi_works
{
class RedoxiVideoReaderBase;

namespace video_reader_base
{

using OutputPortSpec = redoxi_works::image_ports::types::ImageActionOutputPortSpec;
using OutputPortType = AsyncActionOutputPort<OutputPortSpec>;

//! The delivery policy for making frame delivery request
using RequestPolicy = OutputPortSpec::DeliveryPolicy_t;

//! The init config for RedoxiVideoReaderBase or its subclass
struct InitConfig : public common_nodes::OpenCloseNode::InitConfig_t {
    virtual ~InitConfig() = default;
    InitConfig() = default;
    //! The downstream nodes, indexed by node name
    std::shared_ptr<OutputPortSpec::InitConfig_t> primary_output_spec = std::make_shared<OutputPortSpec::InitConfig_t>();

    //! create the debug publish topic for this video reader?
    bool create_debug_pub = true;
    // int debug_pub_queue_size = 10;
    std::string debug_pub_task_enqueue_name = "debug_port/task_enqueue";
    std::string debug_pub_task_drop_name = "debug_port/task_drop";

    //! parse from node, the node must be exactly RedoxiVideoReaderBase, not its subclass
    template <typename Node_t>
    requires std::is_same_v<Node_t, RedoxiVideoReaderBase>
    void from_node(const Node_t *node)
    {
        InitConfig::parse_from_node_parameters(this, node);
    }

    // json serialize
    JS_OBJECT_WITH_SUPER(JS_SUPER(common_nodes::OpenCloseNode::InitConfig_t),
                         JS_MEMBER(primary_output_spec),
                         JS_MEMBER(create_debug_pub),
                         JS_MEMBER(debug_pub_task_enqueue_name),
                         JS_MEMBER(debug_pub_task_drop_name));
};

//! The runtime config for RedoxiVideoReaderBase or its subclass
class RuntimeConfig : public common_nodes::OpenCloseNode::RuntimeConfig_t
{
  public:
    // IMPORTANT: default output is rgb8, if you use bgr8, you need to specify it in the config
    // inline static const std::string DefaultOutputImageEncoding = DefaultColorImageEncoding;
    inline static const TimeUnit_t DefaultRequestRetryInterval{std::chrono::milliseconds(5)};
    inline static const TimeUnit_t DefaultRequestRetryResponseTime{std::chrono::milliseconds(5)};
    inline static const int DefaultRequestRetryNumber{5};

    virtual ~RuntimeConfig() = default;
    RuntimeConfig()
    {
        auto &p = frame_enqueue_policy;

        // respect default settings defined in policy
        p.get_retry_policy().set_number_of_retry(DefaultRequestRetryNumber);
        p.get_retry_policy().set_wait_time_between_retry(DefaultRequestRetryInterval);
        p.get_retry_policy().set_wait_time_retry_response(DefaultRequestRetryResponseTime);
    }

    //! The frame interval in ms, 0 means as fast as possible
    TimeUnit_t frame_interval{0};

    //! The output image size. If input is different, resize to output_image_size.
    //! If output_image_size.width=-1 or output_image_size.height=-1, keep aspect ratio.
    cv::Size output_image_size = cv::Size(-1, -1);

    //! The encoding of the output image, see sensor_msgs::image_encodings for more details
    //! accepts rgb8, bgr8, mono8, mono16
    std::string output_image_encoding{DefaultColorImageEncoding.data()};

    //! publish in debug topic?
    bool publish_to_debug_topic = false;

    //! delivery policy for frame delivery request, after the frame is enqueued
    //! when this is set, it will override the individual downstream delivery policies in the output port
    std::optional<OutputPortSpec::DeliveryPolicy_t> frame_request_policy;

    //! delivery policy for frame enqueue request
    RequestPolicy frame_enqueue_policy;

    //! parse from node, the node must be exactly RedoxiVideoReaderBase, not its subclass
    template <typename Node_t>
    requires std::is_same_v<Node_t, RedoxiVideoReaderBase>
    void from_node(const Node_t *node)
    {
        RuntimeConfig::parse_from_node_parameters(this, node);
    }

    // json serialize
    JS_OBJECT_WITH_SUPER(JS_SUPER(common_nodes::OpenCloseNode::RuntimeConfig_t),
                         JS_MEMBER(frame_interval),
                         JS_MEMBER(output_image_size),
                         JS_MEMBER(output_image_encoding),
                         JS_MEMBER(publish_to_debug_topic),
                         JS_MEMBER(frame_request_policy),
                         JS_MEMBER(frame_enqueue_policy));
};

} // namespace video_reader_base

} // namespace redoxi_works
