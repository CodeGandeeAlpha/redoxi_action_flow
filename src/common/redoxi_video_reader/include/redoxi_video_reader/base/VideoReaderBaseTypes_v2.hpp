#pragma once

#include <redoxi_common_nodes/async_action_port/AsyncActionOutputPort.hpp>
#include <redoxi_common_nodes/async_action_port/ImageOutputPortSpec.hpp>
#include <redoxi_common_cpp/redoxi_common_cpp.hpp>
#include <redoxi_video_reader/visibility_control.h>
#include <json_struct/json_struct.h>

namespace redoxi_works
{
class RedoxiVideoReaderBase_v2;

namespace RedoxiVideoReaderBaseTypes_v2
{

using MainSpec = async_action_image_output_port::ImageOutputPortSpec;

//! The init config for RedoxiVideoReaderBase or its subclass
struct REDOXI_VIDEO_READER_PUBLIC InitConfig {
    virtual ~InitConfig() = default;

    //! The downstream nodes, indexed by node name
    std::map<std::string, std::shared_ptr<MainSpec::DownstreamSpec_t>> downstreams;

    //! create the debug publish topic for this video reader?
    bool create_debug_pub = true;
    int debug_pub_queue_size = 10;
    std::string debug_pub_task_enqueue_name = "debug_port/task_enqueue";
    std::string debug_pub_task_drop_name = "debug_port/task_drop";

    //! Load parameters from node, this will override empty existing parameters
    virtual void from_parameters(rclcpp::Node *){};

    // json serialize
    JS_OBJECT(JS_MEMBER(downstreams),
              JS_MEMBER(create_debug_pub),
              JS_MEMBER(debug_pub_queue_size),
              JS_MEMBER(debug_pub_task_enqueue_name),
              JS_MEMBER(debug_pub_task_drop_name));
};

//! The runtime config for RedoxiVideoReaderBase or its subclass
class REDOXI_VIDEO_READER_PUBLIC RuntimeConfig
{
  public:
    inline static const std::string DEFAULT_OUTPUT_IMAGE_ENCODING = "bgr8";
    inline static const MainSpec::TimeUnit_t DEFAULT_STEP_INTERVAL{10};

    virtual ~RuntimeConfig() = default;
    RuntimeConfig()
    {
        fallback_delivery_policy = std::make_shared<MainSpec::DeliveryPolicy_t>();
    }

    //! The step interval in ms
    MainSpec::TimeUnit_t step_interval{DEFAULT_STEP_INTERVAL};

    //! The frame interval in ms, 0 means as fast as possible
    MainSpec::TimeUnit_t frame_interval{0};

    //! The output image size. If input is different, resize to output_image_size.
    //! If output_image_size.width=-1 or output_image_size.height=-1, keep aspect ratio.
    cv::Size output_image_size = cv::Size(-1, -1);

    //! The encoding of the output image, see sensor_msgs::image_encodings for more details
    //! accepts rgb8, bgr8, mono8, mono16
    std::string output_image_encoding = DEFAULT_OUTPUT_IMAGE_ENCODING;

    //! publish in debug topic?
    bool publish_to_debug_topic = false;

    //! The frame delivery quality of service
    std::shared_ptr<MainSpec::DeliveryPolicy_t> fallback_delivery_policy;

    //! Load parameters from node, this will override empty existing parameters
    virtual void from_parameters(rclcpp::Node *){};

    // json serialize
    JS_OBJECT(JS_MEMBER(step_interval),
              JS_MEMBER(frame_interval),
              JS_MEMBER(output_image_size),
              JS_MEMBER(output_image_encoding),
              JS_MEMBER(publish_to_debug_topic),
              JS_MEMBER(fallback_delivery_policy));
};

} // namespace RedoxiVideoReaderBaseTypes_v2

} // namespace redoxi_works
