#pragma once

#include "psg_document_sink/visibility_control.h"
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <nlohmann/json.hpp>
#include <redoxi_common_cpp/redoxi_common_cpp.hpp>
#include <psg_document_sink/AsyncDocumentInputPort.hpp>
#include <psg_master_node/StampedDocumentPub.hpp>
#include <redoxi_common_cpp/ros_utils/StampedImagePub.hpp>
namespace redoxi_works
{
class PSGDocumentSink;

struct PSGDocumentSinkInitConfig {
    PSGDocumentSinkInitConfig();
    virtual ~PSGDocumentSinkInitConfig() = default;
    virtual void from_parameters(const PSGDocumentSink *node);

    //! time unit name
    std::string _time_unit = get_default_time_unit_name();

    using InputPort_t = AsyncDocumentInputPort;
    std::shared_ptr<InputPort_t::InitConfig_t>
        input_port_config = std::make_shared<InputPort_t::InitConfig_t>();

    //! The interval between steps in microseconds
    DefaultTimeUnit_t step_interval{10000};

    //! Use blocking mode for the reading input port
    bool enable_blocking_mode = false;

    //! The topic to publish the relayed document
    std::string publish_topic = "out/relayed_document";
    std::string publish_topic_image = "out/relayed_image";

    //! debug topics
    bool enable_debug_topics = true;
    std::string debug_topic_document_accepted = "debug_port/document_accepted";
    std::string debug_topic_document_rejected = "debug_port/document_rejected";

    JS_OBJECT(JS_MEMBER(input_port_config),
              JS_MEMBER(_time_unit),
              JS_MEMBER(step_interval),
              JS_MEMBER(publish_topic),
              JS_MEMBER(enable_debug_topics),
              JS_MEMBER(debug_topic_document_accepted),
              JS_MEMBER(debug_topic_document_rejected),
              JS_MEMBER(enable_blocking_mode));
};

class PSGDocumentSink : public rclcpp::Node, public IStartStopProtocol
{
  public:
    PSGDocumentSink(const std::string &node_name, const rclcpp::NodeOptions &options);
    virtual ~PSGDocumentSink();

  public: // useful types
    using InputPort_t = AsyncDocumentInputPort;
    using SourceData_t = InputPort_t::SourceData_t;
    using ActionDataTrait_t = InputPort_t::ActionDataTrait_t;

    using InitConfig_t = PSGDocumentSinkInitConfig;

    virtual int init(std::shared_ptr<InitConfig_t> init_config);
    int start() override;
    int stop() override;

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

    //! Get the parameters
    const auto &get_json_parameters() const
    {
        return m_json_parameters;
    }

  protected:
    virtual void _step();

  protected:
    std::shared_ptr<InputPort_t> m_input_port;
    std::shared_ptr<InitConfig_t> m_init_config;
    nlohmann::json m_json_parameters;
    std::atomic<bool> m_enable_debug_topics{false};

    // publishers
    StampedDocumentPub m_pub_relayed_document;
    // StampedImagePub m_pub_relayed_image;
    StampedImagePub m_pub_debug_document_accepted;
    StampedImagePub m_pub_debug_document_rejected;
    std::shared_ptr<std::thread> m_step_thread;
    std::atomic<bool> m_running{false};
};

} // namespace redoxi_works
