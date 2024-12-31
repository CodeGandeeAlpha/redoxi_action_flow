#pragma once

#include <rclcpp/rclcpp.hpp>
#include <optional>
#include <nlohmann/json.hpp>

#include <redoxi_common_cpp/redoxi_common_cpp.hpp>
#include <redoxi_common_cpp/ros_utils/StampedImagePub.hpp>
#include <redoxi_common_nodes/async_action_port/AsyncActionOutputPort.hpp>
#include <redoxi_shared_memory/SharedMemoryClient.hpp>
#include <redoxi_common_nodes/base_nodes/v2/OpenCloseNode.hpp>
#include <redoxi_video_reader/base/v2/VideoReaderBaseTypes.hpp>

namespace redoxi_works::video_reader::v2
{

struct RedoxiVideoReaderImpl;

/**
 * @brief The base class for all video readers.
 * A frame is considered successfully sent if any downstream accepts it, in which case the frame is written into shared memory. Otherwise the frame is dropped.
 * @note: this is a stateful node, the status code is used to indicate the current state
 * state changes as following:
 * BEFORE_INIT -> [init()] -> CLOSED -> [open()] -> OPENED -> [start()] -> STARTED -> [stop()] -> STOPPED -> [close()] -> CLOSED
 * the action allowed at each state is shown in the comments of each function
 */
class RedoxiVideoReaderBase : public common_nodes::v2::OpenCloseNode
{
    friend struct RedoxiVideoReaderImpl;
    friend struct base::v2::InitConfig;

  public:
    //! result of read frame operation
    enum class ReadFrameResult {
        OK = 0,           // success
        END_OF_VIDEO = 1, // end of video
        NO_DATA = 2,      // this frame has no data (reading too fast?), not an error
        ERROR = -1,       // unknown error
    };

  public:
    //! Import all names from RedoxiVideoReaderInternalTypes
    //! @note: this is to allow subclass to override the type definitions
    // using OutputPortSpec = video_reader_base::OutputPortSpec;
    using OutputPort_t = base::v2::OutputPortType;

    using InitConfig_t = base::v2::InitConfig;
    using RuntimeConfig_t = base::v2::RuntimeConfig;

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

    // init config and runtime config types of OpenCloseNode
    using BaseNode_t = common_nodes::v2::OpenCloseNode;
    using BaseInitConfig_t = BaseNode_t::InitConfig_t;
    using BaseRuntimeConfig_t = BaseNode_t::RuntimeConfig_t;

  public:
    //! Constructor with node options and name
    explicit RedoxiVideoReaderBase(const std::string &name, const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

    //! Destructor
    virtual ~RedoxiVideoReaderBase() noexcept;

  public:
    //! enable or disable image publishing
    virtual void set_publish_to_debug_topic(bool enable);
    virtual bool get_publish_to_debug_topic() const;
    int64_t get_last_read_frame_number() const;

  protected: // from base class
    int _open() override;
    int _start() override;
    int _stop() override;
    int _close() override;
    void _step() override;
    int _update_init_config(std::shared_ptr<BaseInitConfig_t> config) override;
    int _update_runtime_config(std::shared_ptr<BaseRuntimeConfig_t> config) override;

  protected: // video reader specific
    /**
     * @brief Read next frame, intended to be overridden by subclass
     * @param source_data the source data to be filled with the read frame
     * @param frame_number input is the frame number of PREVIOUS frame, output is the frame number of CURRENT frame.
     *        You should update this to the CURRENT frame number.
     * @return ReadFrameResult::OK if success, ReadFrameResult::END_OF_VIDEO if end of video, otherwise error code
     */
    virtual ReadFrameResult _read_frame(SourceData_t &source_data,
                                        std::atomic<int64_t> &frame_number) = 0;

    //! read frame and update the frame number, without direct access to m_last_read_frame_number
    ReadFrameResult _read_frame(SourceData_t &source_data)
    {
        return _read_frame(source_data, m_last_read_frame_number);
    }

    //! callback before a delivery request is enqueued, about to send to any downstream
    //! you can modify the request or enqueue policy here
    //! return 0 if success, otherwise error code
    virtual int _on_before_request_enqueue(DeliveryRequest_t &request, DeliveryPolicy_t &enqueue_policy)
    {
        (void)request;
        (void)enqueue_policy;
        return 0;
    }

    /**
     * @brief Create a delivery request from source data
     * @param source_data the source data to be filled with the read frame
     * @param control_signal_code the control signal code, default is std::nullopt,
     *        which leaves the control signal code in source data unchanged.
     * @return the delivery request
     */
    virtual DeliveryRequest_t _create_delivery_request(
        const SourceData_t &source_data,
        std::optional<ControlSignalCode> control_signal_code = std::nullopt);

    //! create primary output port
    virtual std::shared_ptr<OutputPort_t> _create_primary_output_port(const InitConfig_t &init_config);

    //! create implementation details of this node
    //! @note this must be called before any other operations, so it cannot access any member variables
    virtual std::shared_ptr<RedoxiVideoReaderImpl> _create_impl();


  protected: // output port callback
    //! callback when a delivery task is started, about to send to any downstream
    virtual int _on_delivery_task_begin(TargetData_t &target_data,
                                        const DeliveryRequest_t &request);

    //! callback when a delivery task is finished, after sending to all downstreams
    virtual int _on_delivery_task_finish(TargetData_t &target_data,
                                         const DeliveryRequest_t &request,
                                         const DeliveryResult_t &result);

    //! callback when a frame is sent to a downstream, failure or success
    virtual int _on_deliver_to_downstream_finish(TargetData_t &target_data,
                                                 SendResult_t &result,
                                                 const DeliveryRequest_t &request,
                                                 const Downstream_t &ds)
    {
        (void)target_data;
        (void)result;
        (void)request;
        (void)ds;

        return 0;
    };

    //! reset the frame number to initial value
    void _reset_frame_number()
    {
        m_last_read_frame_number = -1;
    }

    //! increment the frame number by the given increment, and return the new value
    //! @note: this is atomic and thread safe
    int64_t _increment_frame_number_by(std::atomic<int64_t> &frame_number, int64_t increment)
    {
        return frame_number.fetch_add(increment) + increment;
    }

  protected:
    // member of downstreams
    std::shared_ptr<OutputPort_t> m_primary_output_port;

    // publish to debug topic
    std::atomic<bool> m_publish_to_debug_topic{false};

    //! implementation details of this node
    std::shared_ptr<RedoxiVideoReaderImpl> m_impl;
    //! debug publishers
    StampedImagePub m_pub_task_enqueue;
    StampedImagePub m_pub_task_drop;

  private:
    // frame number last read by this reader, which is, the frame number of PREVIOUS frame
    // -1 means not read any frame, increment by each _read_frame()
    // will be reset to -1 when open()
    std::atomic<int64_t> m_last_read_frame_number{-1};
};

} // namespace redoxi_works::video_reader::v2
