#pragma once

#include <rclcpp/rclcpp.hpp>
#include <redoxi_common_nodes/image_ports/AsyncImageInputPort.hpp>
#include <redoxi_common_cpp/redoxi_common_cpp.hpp>
#include <psg_master_node/StampedDocumentPub.hpp>
#include <redoxi_common_nodes/async_action_port/AsyncActionOutputPort.hpp>
#include <psg_master_node/MasterNodeTypes.hpp>
#include <thread>
#include <nlohmann/json.hpp>

namespace redoxi_works
{

struct PSGMasterNodeImpl;

/**
 * @brief The class for PSG master node.
 * A frame is considered successfully sent if any downstream accepts it, in which case the frame is written into shared memory. Otherwise the frame is dropped.
 * @note: this is a stateful node, the status code is used to indicate the current state
 * state changes as following:
 * BEFORE_INIT -> [init()] -> CLOSED -> [open()] -> OPENED -> [start()] -> STARTED -> [stop()] -> STOPPED -> [close()] -> CLOSED
 * the action allowed at each state is shown in the comments of each function
 */
class PSGMasterNode : public common_nodes::StartStopNode
{
    friend struct PSGMasterNodeImpl;
    friend struct psg_master_node::InitConfig;
    friend struct psg_master_node::RuntimeConfig;

  public:
    using InputPort_t = image_ports::AsyncImageInputPort;
    using InputSourceData_t = InputPort_t::SourceData_t;
    using ActionDataTrait_t = InputPort_t::ActionDataTrait_t;

    //! Import all names from PSGMasterNodeInternalTypes
    //! @note: this is to allow subclass to override the type definitions
    // using OutputPortSpec = video_reader_base::OutputPortSpec;
    using OutputPort_t = psg_master_node::OutputPortType;

    using InitConfig_t = psg_master_node::InitConfig;
    using RuntimeConfig_t = psg_master_node::RuntimeConfig;

    using Downstream_t = OutputPort_t::Downstream_t;
    using DownstreamSpec_t = Downstream_t::DownstreamSpec_t;

    using DeliveryGoal_t = OutputPort_t::ActionType_t::Goal;
    using DeliveryRequest_t = OutputPort_t::DeliveryRequest_t;
    using DeliveryPolicy_t = DeliveryRequest_t::DeliveryPolicy_t;
    using SendFrameResult_t = OutputPort_t::SendResult_t;
    using OutputSourceData_t = OutputPort_t::SourceData_t;

    // init config and runtime config types of StartStopNode
    using BaseInitConfig_t = common_nodes::StartStopNode::InitConfig_t;
    using BaseRuntimeConfig_t = common_nodes::StartStopNode::RuntimeConfig_t;

  public:
    //! Constructor with node options and name
    explicit PSGMasterNode(const std::string &name, const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

    //! Destructor
    virtual ~PSGMasterNode();

    using common_nodes::StartStopNode::get_init_config;
    using common_nodes::StartStopNode::get_runtime_config;

  public:
    //! enable or disable document publishing
    virtual void set_publish_to_debug_topic(bool enable);
    virtual bool get_publish_to_debug_topic() const;

  protected: // from base class
    int _start() override;
    int _stop() override;
    void _step() override;
    int _update_init_config(std::shared_ptr<BaseInitConfig_t> config) override;
    int _update_runtime_config(std::shared_ptr<BaseRuntimeConfig_t> config) override;

  protected:
    /**
     * @brief Create a delivery request from source data
     * @param source_data the source data to be filled with the read frame
     * @return the delivery request
     */
    virtual DeliveryRequest_t _create_delivery_request(
        const OutputSourceData_t &source_data,
        std::optional<ControlSignalCode> control_signal_code = std::nullopt);

    //! create primary output port
    virtual std::shared_ptr<OutputPort_t> _create_primary_output_port(const InitConfig_t &init_config);

    //! create implementation details of this node
    //! @note this must be called before any other operations, so it cannot access any member variables
    virtual std::shared_ptr<PSGMasterNodeImpl> _create_impl();

    //! do periodic step operation
    virtual void _step2();

  protected:
    // input port
    std::shared_ptr<InputPort_t> m_input_port;
    // member of downstreams
    std::shared_ptr<OutputPort_t> m_primary_output_port;

    // publish to debug topic
    std::atomic<bool> m_publish_to_debug_topic{false};

    //! implementation details of this node
    std::shared_ptr<PSGMasterNodeImpl> m_impl;

    //! debug publishers
    StampedImagePub m_pub_task_enqueue;
    StampedImagePub m_pub_task_drop;
};

} // namespace redoxi_works
