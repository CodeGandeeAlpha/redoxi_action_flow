#pragma once

#include <redoxi_common_nodes/async_action_port/AsyncActionOutputPort.hpp>
#include <redoxi_common_nodes/async_action_port/ImageOutputPortSpec.hpp>
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

using OutputPortSpec = async_action_image_output_port::ImageOutputPortSpec;
using OutputPortType = AsyncActionOutputPort<OutputPortSpec>;

//! The delivery policy for making frame delivery request
using RequestPolicy = OutputPortSpec::DeliveryPolicy_t;

//! The init config for RedoxiVideoReaderBase or its subclass
struct InitConfig {
    virtual ~InitConfig() = default;
    InitConfig()
    {
        // by default, only starts sending any data when some downstream is ready
        // skip if no downstream is ready
        primary_output_spec.set_fallback_delivery_precondition(DeliveryPrecondition::AnyDownstreamReady);
    }

    //! The time unit for the step interval, see redoxi_common_cpp::get_default_time_unit_name for more details
    //! @note: this is just for json serialization, intended as a comment, do not modify it
    std::optional<std::string> _time_unit = get_default_time_unit_name();

    //! The downstream nodes, indexed by node name
    OutputPortSpec::InitConfig_t primary_output_spec;

    //! create the debug publish topic for this video reader?
    bool create_debug_pub = true;
    int debug_pub_queue_size = 10;
    std::string debug_pub_task_enqueue_name = "debug_port/task_enqueue";
    std::string debug_pub_task_drop_name = "debug_port/task_drop";

    //! Load parameters from node, this will override empty existing parameters
    virtual void from_parameters(RedoxiVideoReaderBase *);

    // json serialize
    JS_OBJECT(JS_MEMBER(_time_unit),
              JS_MEMBER(primary_output_spec),
              JS_MEMBER(create_debug_pub),
              JS_MEMBER(debug_pub_queue_size),
              JS_MEMBER(debug_pub_task_enqueue_name),
              JS_MEMBER(debug_pub_task_drop_name));
};

//! The runtime config for RedoxiVideoReaderBase or its subclass
class RuntimeConfig
{
  public:
    // IMPORTANT: default output is rgb8, if you use bgr8, you need to specify it in the config
    inline static const std::string DEFAULT_OUTPUT_IMAGE_ENCODING = sensor_msgs::image_encodings::RGB8;
    inline static const DefaultTimeUnit_t DEFAULT_STEP_INTERVAL{std::chrono::milliseconds(5)};
    inline static const DefaultTimeUnit_t DEFAULT_REQUEST_RETRY_INTERVAL{std::chrono::milliseconds(5)};
    inline static const DefaultTimeUnit_t DEFAULT_REQUEST_RETRY_RESPONSE_TIME{std::chrono::milliseconds(5)};
    inline static const int DEFAULT_REQUEST_RETRY_NUMBER{5};

    virtual ~RuntimeConfig() = default;
    RuntimeConfig()
    {
        auto &p = frame_enqueue_policy;
        p.set_drop_strategy(DropStrategy::DropAsNeeded);
        p.set_precondition(DeliveryPrecondition::AnyDownstreamReady);
        p.get_retry_policy().set_number_of_retry(DEFAULT_REQUEST_RETRY_NUMBER);
        p.get_retry_policy().set_wait_time_between_retry(DEFAULT_REQUEST_RETRY_INTERVAL);
        p.get_retry_policy().set_wait_time_retry_response(DEFAULT_REQUEST_RETRY_RESPONSE_TIME);
    }

    //! The time unit for the step interval, see redoxi_common_cpp::get_default_time_unit_name for more details
    //! @note: this is just for json serialization, do not modify it
    std::optional<std::string> _time_unit = get_default_time_unit_name();

    //! The step interval in ms
    OutputPortSpec::TimeUnit_t step_interval{DEFAULT_STEP_INTERVAL};

    //! The frame interval in ms, 0 means as fast as possible
    OutputPortSpec::TimeUnit_t frame_interval{0};

    //! The output image size. If input is different, resize to output_image_size.
    //! If output_image_size.width=-1 or output_image_size.height=-1, keep aspect ratio.
    cv::Size output_image_size = cv::Size(-1, -1);

    //! The encoding of the output image, see sensor_msgs::image_encodings for more details
    //! accepts rgb8, bgr8, mono8, mono16
    std::string output_image_encoding = DEFAULT_OUTPUT_IMAGE_ENCODING;

    //! publish in debug topic?
    bool publish_to_debug_topic = false;

    //! delivery policy for frame delivery request, after the frame is enqueued
    //! when this is set, it will override the individual downstream delivery policies in the output port
    std::optional<OutputPortSpec::DeliveryPolicy_t> frame_request_policy;

    //! delivery policy for frame enqueue request
    RequestPolicy frame_enqueue_policy;

    //! Load parameters from node, this will override empty existing parameters
    virtual void from_parameters(RedoxiVideoReaderBase *);

    // json serialize
    JS_OBJECT(JS_MEMBER(_time_unit),
              JS_MEMBER(step_interval),
              JS_MEMBER(frame_interval),
              JS_MEMBER(output_image_size),
              JS_MEMBER(output_image_encoding),
              JS_MEMBER(publish_to_debug_topic),
              JS_MEMBER(frame_request_policy),
              JS_MEMBER(frame_enqueue_policy));
};

} // namespace video_reader_base

} // namespace redoxi_works
