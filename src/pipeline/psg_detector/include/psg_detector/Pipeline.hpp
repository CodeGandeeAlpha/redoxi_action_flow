#pragma once

#include <rclcpp/rclcpp.hpp>
#include <redoxi_common_cpp/redoxi_common_cpp.hpp>
#include <psg_master_node/StampedDocumentPub.hpp>
#include <redoxi_common_nodes/async_action_port/AsyncActionOutputPort.hpp>
#include <psg_document_sink/AsyncDocumentInputPort.hpp>
#include <psg_detector/GetDetectionsOutputSpec.hpp>
#include <psg_detector/PipelineTypes.hpp>
#include <thread>
#include <nlohmann/json.hpp>

namespace redoxi_works
{

struct PSGDetectorImpl;

/**
 * @brief The class for PSG detector node.
 * A frame is considered successfully sent if any downstream accepts it, in which case the frame is written into shared memory. Otherwise the frame is dropped.
 * @note: this is a stateful node, the status code is used to indicate the current state
 * state changes as following:
 * BEFORE_INIT -> [init()] -> CLOSED -> [open()] -> OPENED -> [start()] -> STARTED -> [stop()] -> STOPPED -> [close()] -> CLOSED
 * the action allowed at each state is shown in the comments of each function
 */
class PSGDetectorNode : public common_nodes::StartStopNode
{
    friend struct PSGDetectorImpl;
    friend struct psg_detector::InitConfig;
    friend struct psg_detector::RuntimeConfig;

  public:
    using InputPort_t = AsyncDocumentInputPort;
    using InputSourceData_t = InputPort_t::SourceData_t;
    using ActionDataTrait_t = InputPort_t::ActionDataTrait_t;

    //! Import all names from PSGMasterNodeInternalTypes
    //! @note: this is to allow subclass to override the type definitions
    // using OutputPortSpec = video_reader_base::OutputPortSpec;
    using OutputPortPipeline_t = psg_detector::OutputPortPipelineType;
    using OutputPortModel_t = psg_detector::OutputPortModelType;

    using OutputModelSpec_t = psg_detector::OutputPortModelSpec;
    using OutputModelAction_t = OutputModelSpec_t::ActionType_t;
    using OutputModelResult_t = OutputModelAction_t::Result;

    using InitConfig_t = psg_detector::InitConfig;
    using RuntimeConfig_t = psg_detector::RuntimeConfig;

    using DownstreamPipeline_t = OutputPortPipeline_t::Downstream_t;
    using DownstreamModel_t = OutputPortModel_t::Downstream_t;
    using DownstreamSpecPipeline_t = DownstreamPipeline_t::DownstreamSpec_t;
    using DownstreamSpecModel_t = DownstreamModel_t::DownstreamSpec_t;

    using DeliveryGoalPipeline_t = OutputPortPipeline_t::ActionType_t::Goal;
    using DeliveryGoalModel_t = OutputPortModel_t::ActionType_t::Goal;
    using DeliveryRequestPipeline_t = OutputPortPipeline_t::DeliveryRequest_t;
    using DeliveryRequestModel_t = OutputPortModel_t::DeliveryRequest_t;
    using DeliveryPolicyPipeline_t = DeliveryRequestPipeline_t::DeliveryPolicy_t;
    using DeliveryPolicyModel_t = DeliveryRequestModel_t::DeliveryPolicy_t;
    using DeliveryResultPipeline_t = OutputPortPipeline_t::DeliveryResult_t;
    using DeliveryResultModel_t = OutputPortModel_t::DeliveryResult_t;
    using SendResultPipeline_t = OutputPortPipeline_t::SendResult_t;
    using SendResultModel_t = OutputPortModel_t::SendResult_t;
    using OutputSourceDataPipeline_t = OutputPortPipeline_t::SourceData_t;
    using OutputSourceDataModel_t = OutputPortModel_t::SourceData_t;
    using TargetDataPipeline_t = OutputPortPipeline_t::TargetData_t;
    using TargetDataModel_t = OutputPortModel_t::TargetData_t;
    using DeliveryTaskPipeline_t = OutputPortPipeline_t::DeliveryTask_t;
    using DeliveryTaskModel_t = OutputPortModel_t::DeliveryTask_t;

    // init config and runtime config types of StartStopNode
    using BaseInitConfig_t = common_nodes::StartStopNode::InitConfig_t;
    using BaseRuntimeConfig_t = common_nodes::StartStopNode::RuntimeConfig_t;

  public:
    //! Constructor with node options and name
    using BaseNode_t = common_nodes::StartStopNode;
    using BaseNode_t::BaseNode_t;

    //! Destructor
    virtual ~PSGDetectorNode();

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
    virtual DeliveryRequestPipeline_t _create_delivery_request(const OutputSourceDataPipeline_t &source_data,
                                                               std::optional<ControlSignalCode> control_signal_code);
    virtual DeliveryRequestModel_t _create_delivery_request(const OutputSourceDataModel_t &source_data,
                                                            std::optional<ControlSignalCode> control_signal_code);

    //! create primary output port
    virtual std::shared_ptr<OutputPortPipeline_t> _create_primary_output_port_pipeline(const InitConfig_t &init_config);
    virtual std::shared_ptr<OutputPortModel_t> _create_primary_output_port_model(const InitConfig_t &init_config);

    //! create implementation details of this node
    //! @note this must be called before any other operations, so it cannot access any member variables
    virtual std::shared_ptr<PSGDetectorImpl> _create_impl();

    //! get model result
    virtual void _get_model_result();

    //! create frame request handler
    virtual int _create_frame_request_handler(const RuntimeConfig_t &runtime_config);

    //! process frame request
    virtual int _process_frame_request();

    //! create debug image
    virtual sensor_msgs::msg::Image _create_debug_image(const psg_private_msgs::msg::PsgDocument &document);

  protected: // output port callback
    virtual int _on_deliver_to_downstream_finish(TargetDataModel_t &target_data,
                                                 SendResultModel_t &result,
                                                 const DeliveryRequestModel_t &request,
                                                 const DownstreamModel_t &ds);

    // private:
    //   virtual int _read_frame(OutputSourceDataModel_t &source_data,
    //                           std::atomic<int64_t> &frame_number);

  protected:
    // input port
    std::shared_ptr<InputPort_t> m_input_port;
    // member of downstreams
    std::shared_ptr<OutputPortPipeline_t> m_primary_output_port_pipeline;
    std::shared_ptr<OutputPortModel_t> m_primary_output_port_model;

    // publish to debug topic
    std::atomic<bool> m_publish_to_debug_topic{false};

    //! implementation details of this node
    std::shared_ptr<PSGDetectorImpl> m_impl;

    //! debug publishers
    StampedImagePub m_pub_pipeline_enqueue;
    StampedImagePub m_pub_pipeline_drop;
    StampedImagePub m_pub_model_enqueue;
    StampedImagePub m_pub_model_drop;

    //! frame number
    std::atomic<int64_t> m_frame_number{0};

    //! thread for model result
    std::shared_ptr<std::thread> m_get_model_result_thread;

    std::atomic<bool> m_get_model_result_thread_running{false};
};

} // namespace redoxi_works
