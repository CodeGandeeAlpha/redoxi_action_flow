#pragma once

#include <string_view>
#include <redoxi_common_cpp/redoxi_json_struct_conversion.hpp>
#include <universal_mot_trackers/visibility_control.h>
#include <redoxi_common_nodes/tracking_ports/TrackingRequestInputPort.hpp>
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

struct DeepSORTParams {
    float max_gating_distance = 0.3;

    float base_gating_threshold = 6.325 * 1e-3;

    float alpha_smooth_features = 0.9;
    float gating_dist_lambda = 0.98;
    float duplicate_iou_dist = 0.15;
    bool use_optical_before_track = true;

    JS_OBJECT(
        JS_MEMBER(max_gating_distance),
        JS_MEMBER(base_gating_threshold),
        JS_MEMBER(alpha_smooth_features),
        JS_MEMBER(gating_dist_lambda),
        JS_MEMBER(duplicate_iou_dist),
        JS_MEMBER(use_optical_before_track));
};

struct BoTSORTParams {
    float track_high_thresh = 0.6; // botsort nni 0.35  bytetrack0.5  botsort 0.6
    float track_low_thresh = 0.1;
    float new_track_thresh = 0.7; // botsort nni 0.5   bytetrack0.6  botsort 0.7
    int keep_track_buffer = 30;
    int max_time_lost = 30;
    float match_thresh = 0.8; // botsort nni 0.6   bytetrack0.8  botsort 0.8
    float aspect_ratio_thresh = 1.6;
    float min_box_area = 10.0;
    float proximity_thresh = 0.5;
    float appearance_thresh = 0.25; // botsort nni 0.5   botsort 0.25
    float alpha_smooth_features = 0.9;
    bool use_optical_before_track = false;
    bool fuse_score = false; // botsort/bytetrack false
    bool use_reid_feature = true;

    JS_OBJECT(
        JS_MEMBER(track_high_thresh),
        JS_MEMBER(track_low_thresh),
        JS_MEMBER(new_track_thresh),
        JS_MEMBER(keep_track_buffer),
        JS_MEMBER(max_time_lost),
        JS_MEMBER(match_thresh),
        JS_MEMBER(aspect_ratio_thresh),
        JS_MEMBER(min_box_area),
        JS_MEMBER(proximity_thresh),
        JS_MEMBER(appearance_thresh),
        JS_MEMBER(alpha_smooth_features),
        JS_MEMBER(use_optical_before_track),
        JS_MEMBER(fuse_score),
        JS_MEMBER(use_reid_feature));
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

    // DeepSORT params
    DeepSORTParams deep_sort_params;

    // BoTSORT params
    BoTSORTParams botsort_params;

    JS_OBJECT_WITH_SUPER(
        JS_SUPER(common_nodes::OpenCloseNode::InitConfig_t),
        JS_MEMBER(input_port_config),
        JS_MEMBER(publish_visualization_topic),
        JS_MEMBER(publish_probe_topic),
        JS_MEMBER(preferred_image_size),
        JS_MEMBER(tracker_type),
        JS_MEMBER(motion_prediction_type),
        JS_MEMBER(deep_sort_params),
        JS_MEMBER(botsort_params));
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
