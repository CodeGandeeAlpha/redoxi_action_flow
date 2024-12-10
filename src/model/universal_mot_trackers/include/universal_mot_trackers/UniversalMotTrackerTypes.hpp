#pragma once

#include <string_view>
#include <redoxi_common_cpp/redoxi_json_struct_conversion.hpp>
#include <universal_mot_trackers/visibility_control.h>
#include <universal_mot_trackers/tracking_ports/TrackingRequestInputPort.hpp>
#include <redoxi_common_nodes/base_nodes/OpenCloseNode.hpp>
#include <opencv2/core.hpp>
#include <json_struct/json_struct.h>

namespace redoxi_works::model_nodes::universal_mot_trackers::types
{
using namespace tracking_ports::request_response;
using namespace tracking_ports::request_response::types;

struct TrackerType {
    inline constexpr static std::string_view DeepSORT = "deepsort";
    inline constexpr static std::string_view BoTSort = "botsort";
};

struct MotionPredictionType {
    inline constexpr static std::string_view None = "none"; // no motion prediction
    inline constexpr static std::string_view KalmanFilter = "kalman_filter";
    inline constexpr static std::string_view OpticalFlow = "optical_flow";
    inline constexpr static std::string_view MixedOFKF = "mixed_ofkf"; // mixed optical flow and kalman filter
};

/**
 * @brief The initialization configuration for the Universal MOT Tracker node
 */
class InitConfig : public common_nodes::OpenCloseNode::InitConfig_t
{
  public:
    using InputPort_t = TrackingRequestInputPort;

    std::shared_ptr<InputPort_t::InitConfig_t> input_port_config = std::make_shared<InputPort_t::InitConfig_t>();
    std::string publish_visualization_topic = "debug/visualization";
    std::string publish_probe_topic = "debug/tracking_done";

    // preferred image size, if not set, the first input image size will be used
    cv::Size preferred_image_size;

    // tracker params
    std::string tracker_type = std::string(TrackerType::DeepSORT);
    std::string motion_prediction_type = std::string(MotionPredictionType::MixedOFKF);

    JS_OBJECT_WITH_SUPER(
        JS_SUPER(common_nodes::OpenCloseNode::InitConfig_t),
        JS_MEMBER(input_port_config),
        JS_MEMBER(publish_visualization_topic),
        JS_MEMBER(publish_probe_topic),
        JS_MEMBER(preferred_image_size));
};

/**
 * @brief The runtime configuration for the Universal MOT Tracker node
 */
class RuntimeConfig : public common_nodes::OpenCloseNode::RuntimeConfig_t
{
  public:
    using InputPort_t = TrackingRequestInputPort;

    bool enable_blocking_mode = false;
    bool enable_visualization = true;
    bool enable_performance_probe = true;

    JS_OBJECT_WITH_SUPER(
        JS_SUPER(common_nodes::OpenCloseNode::RuntimeConfig_t),
        JS_MEMBER(enable_blocking_mode),
        JS_MEMBER(enable_visualization),
        JS_MEMBER(enable_performance_probe));
};
} // namespace redoxi_works::model_nodes::universal_mot_trackers::types
