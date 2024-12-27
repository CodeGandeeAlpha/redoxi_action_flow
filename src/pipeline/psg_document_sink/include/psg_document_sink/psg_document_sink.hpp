#pragma once

#include "psg_document_sink/visibility_control.h"
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <nlohmann/json.hpp>
#include <redoxi_common_cpp/redoxi_common_cpp.hpp>
#include <redoxi_common_nodes/base_nodes/StartStopNode.hpp>
#include <psg_document_sink/AsyncDocumentInputPort.hpp>
#include <psg_master_node/StampedDocumentPub.hpp>
#include <redoxi_common_cpp/ros_utils/StampedImagePub.hpp>
namespace redoxi_works
{
class PSGDocumentSink;

struct PSGDocumentSinkInitConfig : public common_nodes::StartStopNode::InitConfig_t {
    PSGDocumentSinkInitConfig();
    virtual ~PSGDocumentSinkInitConfig() = default;

    using InputPort_t = AsyncDocumentInputPort;
    std::shared_ptr<InputPort_t::InitConfig_t>
        input_port_config = std::make_shared<InputPort_t::InitConfig_t>();


    //! The topic to publish the relayed document
    std::string publish_topic = "out/relayed_document";
    std::string publish_topic_image = "out/relayed_image";

    //! debug topics
    std::string debug_topic_document_accepted = "debug_port/document_accepted";
    std::string debug_topic_document_rejected = "debug_port/document_rejected";

    //! save document for test
    std::string save_middle_result_dir_path = "";
    bool enable_save_middle_result = false;

    JS_OBJECT_WITH_SUPER(JS_SUPER(common_nodes::StartStopNode::InitConfig_t),
                         JS_MEMBER(input_port_config),
                         JS_MEMBER(publish_topic),
                         JS_MEMBER(debug_topic_document_accepted),
                         JS_MEMBER(debug_topic_document_rejected),
                         JS_MEMBER(save_middle_result_dir_path),
                         JS_MEMBER(enable_save_middle_result));
};

struct PSGDocumentSinkRuntimeConfig : public common_nodes::StartStopNode::RuntimeConfig_t {
    bool enable_blocking_mode = false;
    bool enable_debug_topics = true;

    JS_OBJECT_WITH_SUPER(JS_SUPER(common_nodes::StartStopNode::RuntimeConfig_t),
                         JS_MEMBER(enable_blocking_mode),
                         JS_MEMBER(enable_debug_topics));
};

class PSGDocumentSink : public common_nodes::StartStopNode
{
  public:
    PSGDocumentSink(const std::string &node_name, const rclcpp::NodeOptions &options);
    virtual ~PSGDocumentSink();

  public: // useful types
    using InputPort_t = AsyncDocumentInputPort;
    using SourceData_t = InputPort_t::SourceData_t;
    using ActionDataTrait_t = InputPort_t::ActionDataTrait_t;

    using InitConfig_t = PSGDocumentSinkInitConfig;
    using RuntimeConfig_t = PSGDocumentSinkRuntimeConfig;

    using BaseInitConfig_t = common_nodes::StartStopNode::InitConfig_t;
    using BaseRuntimeConfig_t = common_nodes::StartStopNode::RuntimeConfig_t;


  protected:
    int _start() override;
    int _stop() override;
    void _step() override;
    int _update_init_config(std::shared_ptr<BaseInitConfig_t> init_config) override;
    int _update_runtime_config(std::shared_ptr<BaseRuntimeConfig_t> runtime_config) override;

  protected:
    void _save_event_count(std::shared_ptr<InitConfig_t> init_config);
    void _save_event_data(std::shared_ptr<InitConfig_t> init_config,
                          const psg_private_msgs::msg::PsgDocument &raw_document);
    void _save_person_every_frame(std::shared_ptr<InitConfig_t> init_config,
                                  const psg_private_msgs::msg::PsgDocument &raw_document);
    void _save_trajectory_every_person(std::shared_ptr<InitConfig_t> init_config,
                                       const psg_private_msgs::msg::PsgDocument &raw_document);

  protected:
    std::shared_ptr<InputPort_t> m_input_port;

    // publishers
    StampedDocumentPub m_pub_relayed_document;
    StampedImagePub m_pub_relayed_image;
    StampedImagePub m_pub_debug_document_accepted;
    StampedImagePub m_pub_debug_document_rejected;

    // 中间结果相关
    std::map<std::string, std::map<std::string, int>> m_event_count;

    std::map<int, std::string> m_EventTyp2String = {
        {0, "None"},
        {1, "Disappear"},
        {2, "DoorIn"},
        {3, "DoorOut"},
        {4, "DoorIgnore"},
        {5, "DoorSpeedOut"},
        {6, "DoorSpeedIn"},
        {7, "PassingIn"},
        {8, "PassingOut"},
        {9, "PassingIgnore"}};
};

} // namespace redoxi_works
