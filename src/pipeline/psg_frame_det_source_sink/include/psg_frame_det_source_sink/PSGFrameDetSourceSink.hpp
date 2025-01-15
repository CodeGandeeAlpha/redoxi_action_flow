#pragma once

#include <rclcpp/rclcpp.hpp>
#include <redoxi_common_cpp/redoxi_common_cpp.hpp>
#include <psg_detector/StampedFramePub.hpp>
#include <psg_detector/AsyncGetDetectionsOutputPort.hpp>
#include <psg_frame_det_source_sink/PSGFrameDetSourceSinkTypes.hpp>
#include <redoxi_shared_memory/SharedMemoryClient.hpp>
#include <thread>
#include <nlohmann/json.hpp>

namespace redoxi_works
{

struct PSGFrameDetSourceSinkImpl;

/**
 * @brief The base class for all video readers.
 * A frame is considered successfully sent if any downstream accepts it, in which case the frame is written into shared memory. Otherwise the frame is dropped.
 * @note: this is a stateful node, the status code is used to indicate the current state
 * state changes as following:
 * BEFORE_INIT -> [init()] -> CLOSED -> [open()] -> OPENED -> [start()] -> STARTED -> [stop()] -> STOPPED -> [close()] -> CLOSED
 * the action allowed at each state is shown in the comments of each function
 */
class PSGFrameDetSourceSink : public common_nodes::StartStopNode
{
    friend struct PSGFrameDetSourceSinkImpl;
    friend struct psg_frame_det_source_sink::InitConfig;
    friend struct psg_frame_det_source_sink::RuntimeConfig;

  public:
    //! Import all names from RedoxiVideoReaderInternalTypes
    //! @note: this is to allow subclass to override the type definitions
    // using OutputPortSpec = video_reader_base::OutputPortSpec;
    using OutputPort_t = psg_frame_det_source_sink::OutputPortType;

    using OutputSpec_t = psg_frame_det_source_sink::OutputPortSpec;
    using OutputAction_t = OutputSpec_t::ActionType_t;
    using OutputResult_t = OutputAction_t::Result;

    using InitConfig_t = psg_frame_det_source_sink::InitConfig;
    using RuntimeConfig_t = psg_frame_det_source_sink::RuntimeConfig;

    using Downstream_t = OutputPort_t::Downstream_t;
    using DownstreamSpec_t = Downstream_t::DownstreamSpec_t;

    using DeliveryGoal_t = OutputPort_t::ActionType_t::Goal;
    using DeliveryRequest_t = OutputPort_t::DeliveryRequest_t;
    using DeliveryResult_t = OutputPort_t::DeliveryResult_t;
    using DeliveryPolicy_t = DeliveryRequest_t::DeliveryPolicy_t;
    using DeliveryTask_t = OutputPort_t::DeliveryTask_t;

    using SendFrameResult_t = OutputPort_t::SendResult_t;
    using SourceData_t = OutputPort_t::SourceData_t;
    using TargetData_t = OutputPort_t::TargetData_t;
    using SendResult_t = OutputPort_t::SendResult_t;

    // init config and runtime config types of StartStopNode
    using BaseInitConfig_t = common_nodes::StartStopNode::InitConfig_t;
    using BaseRuntimeConfig_t = common_nodes::StartStopNode::RuntimeConfig_t;


  public:
    //! Constructor with node options and name
    using BaseNode_t = common_nodes::StartStopNode;
    using BaseNode_t::BaseNode_t;

    //! Destructor
    virtual ~PSGFrameDetSourceSink();

  public:
    //! enable or disable image publishing
    virtual void set_publish_to_debug_topic(bool enable);
    virtual bool get_publish_to_debug_topic() const;

  protected: // from base class
    int _start() override;
    int _stop() override;
    void _step() override;
    int _update_init_config(std::shared_ptr<BaseInitConfig_t> config) override;
    int _update_runtime_config(std::shared_ptr<BaseRuntimeConfig_t> config) override;

    DEFAULT_CONFIG_LOADER_IMPL(InitConfig_t, RuntimeConfig_t);

  protected: // from base class
    /**
     * @brief Read next frame, intended to be overridden by subclass
     * @param source_data the source data to be filled with the read frame
     * @param frame_number the frame number of this frame, you are responsible for updating this.
     *        The input value is the previous frame number, and the output value will be the current frame number.
     * @return 0 if success, otherwise error code
     */
    virtual int _read_frame(SourceData_t &source_data,
                            std::atomic<int64_t> &frame_number);

    /**
     * @brief Create a delivery request from source data
     * @param source_data the source data to be filled with the read frame
     * @return the delivery request
     */
    virtual DeliveryRequest_t _create_delivery_request(const SourceData_t &source_data);

    //! create primary output port
    virtual std::shared_ptr<OutputPort_t> _create_primary_output_port(const InitConfig_t &init_config);

    //! create implementation details of this node
    //! @note this must be called before any other operations, so it cannot access any member variables
    virtual std::shared_ptr<PSGFrameDetSourceSinkImpl> _create_impl();

    //! get model result
    virtual void _get_model_result();

  protected: // output port callback
    //! callback when a delivery task is started, about to send to any downstream
    virtual int _on_delivery_task_begin(TargetData_t &target_data,
                                        const DeliveryRequest_t &request)
    {
        return 0;
    }

    //! callback when a delivery task is finished, after sending to all downstreams
    virtual int _on_delivery_task_finish(TargetData_t &target_data,
                                         const DeliveryRequest_t &request,
                                         const DeliveryResult_t &result)
    {
        return 0;
    }

    //! callback when a frame is sent to a downstream, failure or success
    virtual int _on_deliver_to_downstream_finish(TargetData_t &target_data,
                                                 SendResult_t &result,
                                                 const DeliveryRequest_t &request,
                                                 const Downstream_t &ds);

  protected:
    // member of downstreams
    std::shared_ptr<OutputPort_t> m_primary_output_port;

    // current frame number read by this reader
    // -1 means not read any frame, increment by each _read_frame()
    // will be reset to -1 when open()
    std::atomic<int64_t> m_frame_number{-1};

    // publish to debug topic
    std::atomic<bool> m_publish_to_debug_topic{false};

    //! implementation details of this node
    std::shared_ptr<PSGFrameDetSourceSinkImpl> m_impl;

    //! thread for model result
    std::shared_ptr<std::thread> m_get_model_result_thread;
    std::atomic<bool> m_get_model_result_thread_running{false};

    //! shared memory client
    std::shared_ptr<shared_memory::SharedMemoryClient> m_shm_client;
};

} // namespace redoxi_works
