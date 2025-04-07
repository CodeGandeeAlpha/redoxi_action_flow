#pragma once
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <redoxi_samples_nodes/redoxi_samples_nodes.hpp>
#include <redoxi_common_nodes/base_nodes/StartStopNode.hpp>
#include <redoxi_common_nodes/image_ports/AsyncImageInputPort.hpp>
#include <redoxi_common_cpp/ros_utils/StampedImagePub.hpp>
#include <redoxi_shared_memory/SharedMemoryClient.hpp>

namespace redoxi_works::samples
{
class FrameRelayNode;

struct FrameRelayNodeInitConfig : public common_nodes::StartStopNode::InitConfig_t {
    FrameRelayNodeInitConfig();
    virtual ~FrameRelayNodeInitConfig() = default;

    using InputPort_t = image_ports::AsyncImageInputPort;
    std::shared_ptr<InputPort_t::InitConfig_t>
        input_port_config = std::make_shared<InputPort_t::InitConfig_t>();

    //! The topic to publish the relayed frame
    std::string publish_topic = "out/relayed_frame";

    //! debug topics
    std::string debug_topic_frame_accepted = "debug_port/frame_accepted";
    std::string debug_topic_frame_rejected = "debug_port/frame_rejected";

    JS_OBJECT_WITH_SUPER(JS_SUPER(common_nodes::StartStopNode::InitConfig_t),
                         JS_MEMBER(input_port_config),
                         JS_MEMBER(publish_topic),
                         JS_MEMBER(debug_topic_frame_accepted),
                         JS_MEMBER(debug_topic_frame_rejected));
};

struct FrameRelayNodeRuntimeConfig : public common_nodes::StartStopNode::RuntimeConfig_t {
    bool enable_blocking_mode = false;
    bool enable_debug_topics = true;

    JS_OBJECT_WITH_SUPER(JS_SUPER(common_nodes::StartStopNode::RuntimeConfig_t),
                         JS_MEMBER(enable_blocking_mode),
                         JS_MEMBER(enable_debug_topics));
};

// start-stop node has no custom open/close operations, so you do not need to implement _open()/_close()
// but as lifecycle node, you still need to do configure()/activate()/deactivate()/cleanup()
class FrameRelayNode : public common_nodes::StartStopNode
{
  public: // useful types
    using InputPort_t = image_ports::AsyncImageInputPort;
    using SourceData_t = InputPort_t::SourceData_t;
    using ActionDataTrait_t = InputPort_t::ActionDataTrait_t;
    using InitConfig_t = FrameRelayNodeInitConfig;
    using RuntimeConfig_t = FrameRelayNodeRuntimeConfig;

    using BaseNode_t = common_nodes::StartStopNode;
    using BaseInitConfig_t = BaseNode_t::InitConfig_t;
    using BaseRuntimeConfig_t = BaseNode_t::RuntimeConfig_t;

  public:
    using BaseNode_t::BaseNode_t;

    //! Enable or disable debug topics
    void set_debug_topics_enabled(bool enable)
    {
        m_enable_debug_topics = enable;
    }

    //! Check if debug topics are enabled
    bool get_debug_topics_enabled() const
    {
        return m_enable_debug_topics;
    }

  protected:
    int _start() override;
    int _stop() override;
    void _step() override;
    int _update_init_config(std::shared_ptr<BaseInitConfig_t> init_config) override;
    int _update_runtime_config(std::shared_ptr<BaseRuntimeConfig_t> runtime_config) override;

    DEFAULT_CONFIG_LOADER_IMPL(InitConfig_t, RuntimeConfig_t);

  protected:
    virtual int _parse_frame(cv::Mat *output, const SourceData_t &source_data);

  protected:
    std::shared_ptr<InputPort_t> m_input_port;
    std::shared_ptr<InitConfig_t> m_init_config;
    std::atomic<bool> m_enable_debug_topics{false};

    // publishers
    StampedImagePub m_pub_relayed_frame;
    StampedImagePub m_pub_frame_accepted;
    StampedImagePub m_pub_frame_rejected;
};

} // namespace redoxi_works::samples
