#pragma once

#include <redoxi_common_nodes/redoxi_common_nodes.hpp>
#include <redoxi_common_nodes/base_nodes/StartStopNode.hpp>
#include <psg_document_sink/AsyncDocumentInputPort.hpp>
#include <psg_document_sink/DocumentInputSpec.hpp>
#include <psg_master_node/AsyncDocumentOutputPort.hpp>
#include <psg_master_node/DocumentOutputSpec.hpp>
#include <redoxi_common_cpp/redoxi_common_cpp.hpp>
#include <redoxi_common_cpp/redoxi_json_struct_conversion.hpp>
#include <json_struct/json_struct.h>
#include <PassengerFlow/PassengerFlow.h>

namespace redoxi_works
{

class PSGCounter;

class RegionInfo
{
  public:
    virtual ~RegionInfo()
    {
    }
    std::string m_name;
    PassengerFlow::RegionType m_region_type;
};
using RegionInfoPtr = std::shared_ptr<RegionInfo>;

class DoorInOutRegionInfo : public RegionInfo
{
  public:
    virtual ~DoorInOutRegionInfo()
    {
    }
    std::vector<PassengerFlow::POINT> m_door_line_pixel_points;
    PassengerFlow::POINT m_door_in_pixel_point; // pixel point in door
    double m_certain_region_size = 0.7;         // m
    double m_likely_region_size = 0.3;          // m
};

class DisappearRegionInfo : public RegionInfo
{
  public:
    virtual ~DisappearRegionInfo()
    {
    }
    std::vector<PassengerFlow::POINT> m_door_line_pixel_points;
    PassengerFlow::POINT m_door_in_pixel_point;
    double m_region_size = 0.3;
};

class PassingRegionInfo : public RegionInfo
{
  public:
    virtual ~PassingRegionInfo()
    {
    }
    std::vector<PassengerFlow::POINT> m_region_pixel_points;
};

struct SceneParameter {
    double m_camera_fx, m_camera_fy, m_camera_ux, m_camera_uy;
    PassengerFlow::MATRIX_4d m_camera_extrinsic; // Tcw
    int m_image_width;
    int m_image_height;
    PassengerFlow::MATRIX_4d m_ground_to_world; // Twg
    std::string m_video_path;
    std::vector<RegionInfoPtr> m_regions;
};


namespace psg_counter
{
using InputPortType = AsyncDocumentInputPort;
using OutputPortType = AsyncDocumentOutputPort;
using OutputPortSpec = OutputPortType::MasterSpec_t;

//! The delivery policy for making frame delivery request
using RequestPolicy = OutputPortSpec::DeliveryPolicy_t;

//! The init config for PSGCounter
struct InitConfig : public common_nodes::StartStopNode::InitConfig_t {
    virtual ~InitConfig() = default;
    InitConfig()
    {
        // by default, only starts sending any data when some downstream is ready
        // skip if no downstream is ready
        output_port_config->set_fallback_delivery_precondition(DeliveryPrecondition::AnyDownstreamReady);
    }

    std::shared_ptr<InputPortType::InitConfig_t>
        input_port_config = std::make_shared<InputPortType::InitConfig_t>();

    //! The downstream nodes, indexed by node name
    std::shared_ptr<OutputPortSpec::InitConfig_t> output_port_config =
        std::make_shared<OutputPortSpec::InitConfig_t>();

    //! passengerflow config path
    std::string passengerflow_config_path;

    //! create the debug publish topic for this node?
    bool create_debug_pub = true;
    int debug_pub_queue_size = 10;
    std::string debug_pub_task_enqueue_name = "debug_port/task_enqueue";
    std::string debug_pub_task_drop_name = "debug_port/task_drop";

    //! parse from node, the node must be exactly PSGCounter, not its subclass
    template <typename Node_t>
    requires std::is_same_v<Node_t, PSGCounter>
    void from_node(const Node_t *node)
    {
        InitConfig::parse_from_node_parameters(this, node);
    }

    // json serialize
    JS_OBJECT_WITH_SUPER(JS_SUPER(common_nodes::StartStopNode::InitConfig_t),
                         JS_MEMBER(input_port_config),
                         JS_MEMBER(output_port_config),
                         JS_MEMBER(create_debug_pub),
                         JS_MEMBER(debug_pub_queue_size),
                         JS_MEMBER(debug_pub_task_enqueue_name),
                         JS_MEMBER(debug_pub_task_drop_name),
                         JS_MEMBER(passengerflow_config_path));
};

//! The runtime config for PSGPersonGenerator
struct RuntimeConfig : public common_nodes::StartStopNode::RuntimeConfig_t {
  public:
    inline static const TimeUnit_t DefaultStepInterval{std::chrono::milliseconds(5)};
    inline static const TimeUnit_t DefaultRequestRetryInterval{std::chrono::milliseconds(5)};
    inline static const TimeUnit_t DefaultRequestRetryResponseTime{std::chrono::milliseconds(5)};
    inline static const int DefaultRequestRetryNumber{5};

    virtual ~RuntimeConfig() = default;
    RuntimeConfig()
    {
        auto &p = frame_enqueue_policy;
        p.set_drop_strategy(DropStrategy::DropAsNeeded);
        p.set_precondition(DeliveryPrecondition::AnyDownstreamReady);
        p.get_retry_policy().set_number_of_retry(DefaultRequestRetryNumber);
        p.get_retry_policy().set_wait_time_between_retry(DefaultRequestRetryInterval);
        p.get_retry_policy().set_wait_time_retry_response(DefaultRequestRetryResponseTime);
    }

    //! The document interval in ms, 0 means as fast as possible
    OutputPortSpec::TimeUnit_t document_interval{0};

    //! publish in debug topic?
    bool publish_to_debug_topic = false;

    //! delivery policy for frame delivery request, after the frame is enqueued
    //! when this is set, it will override the individual downstream delivery policies in the output port
    std::optional<OutputPortSpec::DeliveryPolicy_t> frame_request_policy;

    //! Use blocking mode for the reading input port
    bool enable_blocking_mode = false;

    //! delivery policy for frame enqueue request
    RequestPolicy frame_enqueue_policy;

    //! parse from node, the node must be exactly PSGCounter, not its subclass
    template <typename Node_t>
    requires std::is_same_v<Node_t, PSGCounter>
    void from_node(const Node_t *node)
    {
        RuntimeConfig::parse_from_node_parameters(this, node);
    }

    // json serialize
    JS_OBJECT_WITH_SUPER(JS_SUPER(common_nodes::StartStopNode::RuntimeConfig_t),
                         JS_MEMBER(document_interval),
                         JS_MEMBER(enable_blocking_mode),
                         JS_MEMBER(publish_to_debug_topic),
                         JS_MEMBER(frame_request_policy),
                         JS_MEMBER(frame_enqueue_policy));
};
} // namespace psg_counter

} // namespace redoxi_works
