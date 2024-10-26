#ifndef REDOXI_VIDEO_READER__REDOXI_VIDEO_READER_HPP_
#define REDOXI_VIDEO_READER__REDOXI_VIDEO_READER_HPP_

#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <redoxi_common_cpp/redoxi_common_cpp.hpp>
#include <redoxi_common_cpp/ros_utils/SyncActionSender.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <redoxi_video_reader/visibility_control.h>
#include <redoxi_video_reader/base/VideoReaderBaseTypes.hpp>
namespace redoxi_works
{
class RedoxiVideoReaderImpl;

/**
 * @brief The base class for all video readers.
 * A frame is considered successfully sent if any downstream accepts it, in which case the frame is written into shared memory. Otherwise the frame is dropped.
 * @note: this is a stateful node, the status code is used to indicate the current state
 * state changes as following:
 * BEFORE_INIT -> [init()] -> CLOSED -> [open()] -> OPENED -> [start()] -> STARTED -> [stop()] -> STOPPED -> [close()] -> CLOSED
 * the action allowed at each state is shown in the comments of each function
 */
class REDOXI_VIDEO_READER_PUBLIC
    RedoxiVideoReaderBase : public rclcpp::Node,
                            public IOpenCloseProtocol
{
    friend class RedoxiVideoReaderImpl;
    friend class RedoxiVideoReaderBaseTypes::InitConfig;

  public:
    //! Import all names from RedoxiVideoReaderInternalTypes
    //! @note: this is to allow subclass to override the type definitions
    using InitConfig_t = RedoxiVideoReaderBaseTypes::InitConfig;
    using RuntimeConfig_t = RedoxiVideoReaderBaseTypes::RuntimeConfig;
    using FrameMessage_t = RedoxiVideoReaderBaseTypes::InternalTypes::MSG_Frame;
    using Downstream_t = RedoxiVideoReaderBaseTypes::Downstream;
    using FrameDeliveryTask_t = RedoxiVideoReaderBaseTypes::FrameDeliveryTask;
    using FrameDeliveryOptions_t = RedoxiVideoReaderBaseTypes::FrameDeliveryOptions;
    using SendFrameResult_t = SyncActionSender<Downstream_t::ActionType_t>::_SendResult;

  public:
    //! Constructor with node options and name
    explicit RedoxiVideoReaderBase(const std::string &name, const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

    //! Destructor
    virtual ~RedoxiVideoReaderBase();

  public:
    //! enable or disable image publishing
    virtual void set_publish_to_debug_topic(bool enable);
    virtual bool get_publish_to_debug_topic() const;

  public:
    //! Initialize with configurations, must be called once before open()
    virtual int init(const std::shared_ptr<InitConfig_t> &config,
                     const std::shared_ptr<RuntimeConfig_t> &runtime_config);

    /**
     * @brief Update the init config, only when the node is in CLOSED status
     * @param config the new init config
     * @return 0 if success, otherwise error code
     */
    virtual int update_init_config(const std::shared_ptr<InitConfig_t> &config);

    /**
     * @brief Get the init config
     * @return the init config
     */
    virtual const std::shared_ptr<InitConfig_t> &get_init_config() const
    {
        return m_init_config;
    }

    /**
     * @brief Update the runtime config, only when the node is in CLOSED or STOPPED status
     * @param config the new runtime config
     * @return 0 if success, otherwise error code
     */
    virtual int update_runtime_config(const std::shared_ptr<RuntimeConfig_t> &config);

    /**
     * @brief Get the runtime config
     * @return the runtime config
     */
    virtual const std::shared_ptr<RuntimeConfig_t> &get_runtime_config() const
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
    virtual int get_status_code() const;

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
     * @param frame the frame to be filled with the read frame
     * @param frame_number the frame number of this frame, you are responsible for updating this.
     *        The input value is the previous frame number, and the output value will be the current frame number.
     * @return 0 if success, otherwise error code
     */
    virtual int _read_frame(cv::Mat &frame,
                            std::atomic<int64_t> &frame_number) = 0;

    /**
     * @brief Initialize frame delivery tasks processor
     * @details Should be called in init()
     * @return 0 if success, otherwise error code
     */
    virtual int _init_frame_delivery_tasks();

    /**
     * @brief Preprocess frame for delivery, can be done out of order
     * @param task_input the input task
     * @param task_output the output task
     * @return 0 if success, otherwise error code
     */
    virtual int _do_frame_delivery_preprocess(
        const FrameDeliveryTask_t &task_input,
        FrameDeliveryTask_t &task_output);

    /**
     * @brief Deliver frame to downstreams
     * @param task_input the input task
     * @return 0 if success, otherwise error code
     */
    virtual int _do_frame_delivery_main(const FrameDeliveryTask_t &task_input);

    /**
     * @brief Create a frame delivery task
     * @param frame the frame to be delivered
     * @param task_output the output task
     */
    virtual void _create_frame_delivery_task(const cv::Mat &frame, FrameDeliveryTask_t &task_output);

    /**
     * @brief Add a frame to shared memory
     * @param frame the frame to be added
     * @param object_id output the object id of the frame
     * @return 0 if success, otherwise error code
     */
    virtual int _add_frame_to_shared_memory(const cv::Mat &frame, uint64_t &object_id);

    /**
     * @brief Remove a frame from shared memory
     * @param shared_memory_id the object id of the frame
     * @return 0 if success, otherwise error code
     */
    virtual int _remove_frame_from_shared_memory(uint64_t shared_memory_id);

    /**
     * @brief Create a frame message from delivery task
     * @param task_input the input task
     * @param payload_type the payload type of the frame
     * @param shared_memory_id the id of the allocated shared memory, if any, depends on payload type
     * @return the frame message
     */
    virtual FrameMessage_t _create_frame_message(
        const FrameDeliveryTask_t &task_input,
        FrameDeliveryOptions_t::FramePayloadType payload_type,
        std::optional<uint64_t> shared_memory_id = std::nullopt);

    //! find and connect to downstreams, return 0 if success, otherwise error code
    virtual int _connect_to_downstreams();

    //! change status code
    virtual void _set_status_code(int status_code);

    /**
     * @brief Deliver a frame to a specific downstream, optionally applying retry and waiting for response,
     *        according to the downstream's retry strategy
     * @param frame_msg the frame message to be delivered
     * @param ds the downstream to deliver the frame to
     * @return 0 if success, otherwise error code
     */
    virtual int _deliver_frame(
        const FrameMessage_t &frame_msg,
        const std::shared_ptr<Downstream_t> &ds);

    /**
     * @brief Send a frame to a specific downstream
     *
     * @details This function sends a frame to a given downstream, with optional waiting.
     * If timeout is negative, it will wait indefinitely until the goal is received.
     *
     * @param frame_msg The frame message to be sent
     * @param ds The downstream to send the frame to
     * @param timeout Timeout for waiting for the downstream response, in DefaultTimeUnit_t.
     *                Negative value means wait indefinitely, 0 means no wait, positive value means wait for that duration.
     * @return SendFrameResult_t A struct containing:
     *         - response_code: An optional ActionDownstreamResponse indicating the result (ACCEPTED, REJECTED, TIMEOUT, or not set)
     *         - goal_handle_future: A shared future that can be used to retrieve the goal handle
     *
     * @note If timeout is negative, the response_code in the result will not be set, and the user should use
     *       goal_handle_future.wait() to wait for and process the result.
     */
    virtual SendFrameResult_t _send_frame_to_downstream(
        const FrameMessage_t &frame_msg,
        const std::shared_ptr<Downstream_t> &ds,
        DefaultTimeUnit_t timeout = DefaultTimeUnit_t(-1));

    /**
     * @brief Ping downstream to check if they are ready to accept new frame
     * @details If timeout is negative, it will wait indefinitely until the goal is received.
     *          If timeout is 0, a ping message will still be sent to downstream, but this function
     *          will return false regardless of the downstream reply (because we cannot get it in no time).
     * @param ds The downstream to ping
     * @param timeout Timeout for waiting for the downstream response, in DefaultTimeUnit_t.
     *                Negative value means wait indefinitely, 0 means no wait (returns false but still sends ping),
     *                positive value means wait for that duration.
     * @return true if the downstream is ready to accept new frame, false otherwise (including timeout or when timeout is 0)
     */
    virtual bool _ping(const std::shared_ptr<Downstream_t> &ds,
                       DefaultTimeUnit_t timeout = DefaultParams::PingActionRetryInterval);

    //! do periodic step operation
    virtual void _step();

    //! create the implementation based on configs, will be called in init()
    virtual std::shared_ptr<RedoxiVideoReaderImpl> _create_impl(const std::shared_ptr<InitConfig_t> &init_config,
                                                                const std::shared_ptr<RuntimeConfig_t> &runtime_config);

  private:
    /**
     * @brief Declare all parameters (non-overridable)
     * @details Should be called in subclass constructor
     * @return 0 if success, otherwise return error code
     */
    int _declare_all_parameters();

  protected:
    // member of downstreams
    std::map<std::string, std::shared_ptr<Downstream_t>> m_downstreams;

    // configuration
    std::shared_ptr<InitConfig_t> m_init_config;
    std::shared_ptr<RuntimeConfig_t> m_runtime_config;

    // status code
    std::atomic<int> m_status_code{NodeStatusCode::BEFORE_INIT};

    // current frame number read by this reader
    // -1 means not read any frame, increment by each _read_frame()
    // will be reset to -1 when open()
    std::atomic<int64_t> m_frame_number{-1};

    // hidden implementation data
    std::shared_ptr<RedoxiVideoReaderImpl> m_impl;
};

} // namespace redoxi_works

#endif // REDOXI_VIDEO_READER__REDOXI_VIDEO_READER_HPP_
