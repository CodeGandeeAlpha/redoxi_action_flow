#pragma once

#include <rclcpp/rclcpp.hpp>
#include <redoxi_common_cpp/redoxi_common_cpp.hpp>
#include <redoxi_common_cpp/ros_utils/StampedImagePub.hpp>
#include <redoxi_common_nodes/async_action_port/AsyncActionOutputPort.hpp>
#include <redoxi_video_reader/base/VideoReaderBaseTypes.hpp>
#include <redoxi_shared_memory/SharedMemoryClient.hpp>
#include <thread>
#include <nlohmann/json.hpp>

namespace redoxi_works
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
class RedoxiVideoReaderBase : public rclcpp::Node,
                              public IOpenCloseProtocol
{
    friend struct RedoxiVideoReaderImpl;
    friend struct video_reader_base::InitConfig;

  public:
    //! Import all names from RedoxiVideoReaderInternalTypes
    //! @note: this is to allow subclass to override the type definitions
    // using OutputPortSpec = video_reader_base::OutputPortSpec;
    using OutputPort_t = video_reader_base::OutputPortType;

    using InitConfig_t = video_reader_base::InitConfig;
    using RuntimeConfig_t = video_reader_base::RuntimeConfig;

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

  public:
    //! Constructor with node options and name
    explicit RedoxiVideoReaderBase(const std::string &name, const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

    //! Destructor
    virtual ~RedoxiVideoReaderBase();

  public:
    //! enable or disable image publishing
    virtual void set_publish_to_debug_topic(bool enable);
    virtual bool get_publish_to_debug_topic() const;

    //! get json parameters parsed from ros parameters
    virtual const nlohmann::json &get_json_parameters() const
    {
        return m_json_parameters;
    }

  public:
    //! Initialize with configurations, must be called once before open()
    //! state transition: BEFORE_INIT -> CLOSED
    virtual int init(std::shared_ptr<InitConfig_t> config,
                     std::shared_ptr<RuntimeConfig_t> runtime_config);

    /**
     * @brief Update the init config, only when the node is in CLOSED status
     * @param config the new init config
     * @return 0 if success, otherwise error code
     */
    virtual int update_init_config(std::shared_ptr<InitConfig_t> config);

    /**
     * @brief Get the init config
     * @return the init config
     */
    virtual std::shared_ptr<InitConfig_t> get_init_config() const
    {
        return m_init_config;
    }

    /**
     * @brief Update the runtime config, only when the node is in CLOSED or STOPPED status
     * @param config the new runtime config
     * @return 0 if success, otherwise error code
     */
    virtual int update_runtime_config(std::shared_ptr<RuntimeConfig_t> config);

    /**
     * @brief Get the runtime config
     * @return the runtime config
     */
    virtual std::shared_ptr<RuntimeConfig_t> get_runtime_config() const
    {
        return m_runtime_config;
    }

    /**
     * @brief Open video source, get ready to read, must be called in CLOSED status
     * @return 0 if success, otherwise error code
     */
    int open() final;

    /**
     * @brief Start the node, must be called in OPENED or STOPPED status,
     * after which you cannot modify runtime config
     * @return 0 if success, otherwise error code
     */
    int start() final;

    /**
     * @brief Stop the node, must be called in STARTED status
     * @return 0 if success, otherwise error code
     */
    int stop() final;

    /**
     * @brief Close the node, must be called in OPENED or STOPPED status
     * @return 0 if success, otherwise error code
     */
    int close() final;

    /**
     * @brief Get the status code of this node
     * @return the status code
     */
    virtual int get_status_code() const
    {
        return m_status_code;
    }

  protected:
    //! Open video source, intended to be overridden by subclass
    //! @note State transition is handled by base class
    //! @return 0 if success, otherwise error code
    //! @note If return != 0, state transition will not be applied
    virtual int _open()
    {
        return 0;
    }

    //! Close video source, intended to be overridden by subclass
    //! @note State transition is handled by base class
    //! @return 0 if success, otherwise error code
    //! @note If return != 0, state transition will not be applied
    virtual int _close()
    {
        return 0;
    }

    //! Start reading frames, intended to be overridden by subclass
    //! @note State transition is handled by base class
    //! @return 0 if success, otherwise error code
    //! @note If return != 0, state transition will not be applied
    virtual int _start()
    {
        return 0;
    }

    //! Stop reading frames, intended to be overridden by subclass
    //! @note State transition is handled by base class
    //! @return 0 if success, otherwise error code
    //! @note If return != 0, state transition will not be applied
    virtual int _stop()
    {
        return 0;
    }

    /**
     * @brief Read next frame, intended to be overridden by subclass
     * @param source_data the source data to be filled with the read frame
     * @param frame_number the frame number of this frame, you are responsible for updating this.
     *        The input value is the previous frame number, and the output value will be the current frame number.
     * @return 0 if success, otherwise error code
     */
    virtual int _read_frame(SourceData_t &source_data,
                            std::atomic<int64_t> &frame_number) = 0;

    /**
     * @brief Create a delivery request from source data
     * @param source_data the source data to be filled with the read frame
     * @return the delivery request
     */
    virtual DeliveryRequest_t _create_delivery_request(const SourceData_t &source_data);

    //! create primary output port
    virtual std::shared_ptr<OutputPort_t> _create_primary_output_port();

    //! create implementation details of this node
    //! @note this must be called before any other operations, so it cannot access any member variables
    virtual std::shared_ptr<RedoxiVideoReaderImpl> _create_impl();

    //! change status code
    virtual void _set_status_code(int status_code);

    //! do periodic step operation
    virtual void _step();

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

  private:
    /**
     * @brief Declare all parameters (non-overridable)
     * @details Should be called in subclass constructor
     * @return 0 if success, otherwise return error code
     */
    int _declare_all_parameters();

  protected:
    // member of downstreams
    std::shared_ptr<OutputPort_t> m_primary_output_port;

    // configuration
    std::shared_ptr<InitConfig_t> m_init_config;
    std::shared_ptr<RuntimeConfig_t> m_runtime_config;

    // status code
    std::atomic<int> m_status_code{NodeStatusCode::BEFORE_INIT};

    // current frame number read by this reader
    // -1 means not read any frame, increment by each _read_frame()
    // will be reset to -1 when open()
    std::atomic<int64_t> m_frame_number{-1};

    // publish to debug topic
    std::atomic<bool> m_publish_to_debug_topic{false};

    //! implementation details of this node
    std::shared_ptr<RedoxiVideoReaderImpl> m_impl;

    //! json parameters read from ros parameters
    nlohmann::json m_json_parameters;

    //! debug publishers
    StampedImagePub m_pub_task_enqueue;
    StampedImagePub m_pub_task_drop;

    //! thread for periodic step
    std::shared_ptr<std::thread> m_step_thread;
    std::atomic<bool> m_step_running{false};

    //! shared memory client
    std::shared_ptr<shared_memory::SharedMemoryClient> m_shm_client;
};

} // namespace redoxi_works
