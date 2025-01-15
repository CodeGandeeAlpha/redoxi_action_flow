#pragma once

#include <rclcpp/rclcpp.hpp>
#include <redoxi_common_cpp/redoxi_common_cpp.hpp>
#include <psg_master_node/StampedDocumentPub.hpp>
#include <psg_person_generator/PSGPersonGeneratorTypes.hpp>
#include <thread>
#include <nlohmann/json.hpp>

namespace redoxi_works
{

struct PSGPersonGeneratorImpl;

/**
 * @brief The class for PSG person generator.
 * A frame is considered successfully sent if any downstream accepts it, in which case the frame is written into shared memory. Otherwise the frame is dropped.
 * @note: this is a stateful node, the status code is used to indicate the current state
 * state changes as following:
 * BEFORE_INIT -> [init()] -> CLOSED -> [open()] -> OPENED -> [start()] -> STARTED -> [stop()] -> STOPPED -> [close()] -> CLOSED
 * the action allowed at each state is shown in the comments of each function
 */
class PSGPersonGenerator : public common_nodes::StartStopNode
{
    friend struct PSGPersonGeneratorImpl;
    friend struct psg_person_generator::InitConfig;
    friend struct psg_person_generator::RuntimeConfig;

  public:
    using InputPort_t = psg_person_generator::InputPortType;
    using InputSourceData_t = InputPort_t::SourceData_t;
    using ActionDataTrait_t = InputPort_t::ActionDataTrait_t;

    //! Import all names from PSGPersonGeneratorInternalTypes
    //! @note: this is to allow subclass to override the type definitions
    // using OutputPortSpec = video_reader_base::OutputPortSpec;
    using OutputPort_t = psg_person_generator::OutputPortType;

    using InitConfig_t = psg_person_generator::InitConfig;
    using RuntimeConfig_t = psg_person_generator::RuntimeConfig;

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
    using BaseNode_t = common_nodes::StartStopNode;
    using BaseNode_t::BaseNode_t;

    //! Destructor
    virtual ~PSGPersonGenerator();

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

    DEFAULT_CONFIG_LOADER_IMPL(InitConfig_t, RuntimeConfig_t);

  protected:
    /**
     * @brief Create a delivery request from source data
     * @param source_data the source data to be filled with the read frame
     * @return the delivery request
     */
    virtual DeliveryRequest_t _create_delivery_request(const OutputSourceData_t &source_data,
                                                       std::optional<ControlSignalCode> control_signal_code);

    //! create primary output port
    virtual std::shared_ptr<OutputPort_t> _create_primary_output_port(const InitConfig_t &init_config);

    //! create implementation details of this node
    //! @note this must be called before any other operations, so it cannot access any member variables
    virtual std::shared_ptr<PSGPersonGeneratorImpl> _create_impl();

    //! create document request handler
    virtual int _create_document_request_handler(const RuntimeConfig_t &runtime_config);

    //! process document request
    virtual int _process_document_request();

  protected:
    // input port
    std::shared_ptr<InputPort_t> m_input_port;
    // member of downstreams
    std::shared_ptr<OutputPort_t> m_primary_output_port;

    // publish to debug topic
    std::atomic<bool> m_publish_to_debug_topic{false};

    //! implementation details of this node
    std::shared_ptr<PSGPersonGeneratorImpl> m_impl;
};

} // namespace redoxi_works
