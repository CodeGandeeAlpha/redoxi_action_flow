#ifndef REDOXI_VIDEO_READER_BASE__REDOXI_VIDEO_READER_BASE_HPP_
#define REDOXI_VIDEO_READER_BASE__REDOXI_VIDEO_READER_BASE_HPP_

#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <redoxi_video_reader_base/visibility_control.h>
#include <redoxi_common_cpp/redoxi_common_cpp.hpp>
#include <redoxi_video_reader_base/redoxi_video_reader_types.hpp>
#include <sensor_msgs/msg/image.hpp>
namespace redoxi_works
{
class RedoxiVideoReaderImpl;

/**
 * @brief The base class for all video readers.
 * A frame is considered successfully sent if any downstream accepts it, in which case the frame is written into shared memory. Otherwise the frame is dropped.
 */
class REDOXI_VIDEO_READER_BASE_PUBLIC
    RedoxiVideoReaderBase : public rclcpp::Node,
                            public IOpenCloseProtocol
{
    friend class RedoxiVideoReaderImpl;

  public:
    //! Import all names from RedoxiVideoReaderInternalTypes
    //! @note: this is to allow subclass to override the type definitions
    using ACT_AcceptFrame_t = RedoxiVideoReaderBaseTypes::InternalTypes::ACT_AcceptFrame;
    using InitConfig_t = RedoxiVideoReaderBaseTypes::InitConfig;
    using RuntimeConfig_t = RedoxiVideoReaderBaseTypes::RuntimeConfig;
    using FrameMessage_t = RedoxiVideoReaderBaseTypes::InternalTypes::MSG_Frame;
    using GoalHandle_t = RedoxiVideoReaderBaseTypes::InternalTypes::GoalHandle;
    using Downstream_t = RedoxiVideoReaderBaseTypes::Downstream;
    using FrameDeliveryTask_t = RedoxiVideoReaderBaseTypes::FrameDeliveryTask;

  public:
    //! Constructor with node options and name
    explicit RedoxiVideoReaderBase(const std::string &name, const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

    //! Destructor
    virtual ~RedoxiVideoReaderBase();

  public:
    //! debug topic for visualization
    virtual std::string get_publish_image_topic_name() const
    {
        return "Image";
    }

    //! debug topic for visualization
    virtual int get_publish_image_queue_size() const
    {
        return 10;
    }

    virtual void set_publish_image(bool enable);

  public:
    //! Initialize with configurations, must be called once before open()
    virtual int init(const std::shared_ptr<InitConfig_t> &config,
                     const std::shared_ptr<RuntimeConfig_t> &runtime_config);

    //! You can set configuration before open() or after close()
    virtual int update_init_config(const std::shared_ptr<InitConfig_t> &config);
    virtual const std::shared_ptr<InitConfig_t> &get_init_config() const;

    //! Modify runtime settings, must be called before start(), after stop() or close()
    virtual int update_runtime_config(const std::shared_ptr<RuntimeConfig_t> &config);
    virtual const std::shared_ptr<RuntimeConfig_t> &get_runtime_config() const;

    //! Open video source, get ready to read
    virtual int open() override;

    //! After calling this, you cannot modify runtime config
    virtual int start() override;

    //! Call this before you modify runtime config
    virtual int stop() override;

    //! Call this before you want to modify init config
    virtual int close() override;

    //! Get the status code of this node
    virtual int get_status_code() const;

  protected:
    /**
     * @brief Read next frame, intended to be overridden by subclass
     * @param frame the frame to be filled with the read frame
     * @return 0 if success, otherwise error code
     */
    virtual int _read_frame(cv::Mat &frame) = 0;

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
     * @param shared_memory_id the object id of the frame
     * @return the frame message
     */
    virtual FrameMessage_t _create_frame_message(
        const FrameDeliveryTask_t &task_input,
        std::optional<uint64_t> shared_memory_id = std::nullopt);

    // create publisher for visualization
    virtual void _create_debug_topics();

    // find and connect to downstreams
    virtual void _connect_to_downstreams();

    // change status code
    virtual void _set_status_code(int status_code);

  protected:
    virtual void _step();

    // check if all downstreams are ready to accept new frame
    virtual bool _check_downstreams_ready();

    // ping downstream to check if they are ready to accept new frame
    virtual bool _ping(const std::shared_ptr<Downstream_t> &ds);

    // send frame in shared memory to all downstreams
    // return whether the frame is actually sent
    virtual bool _send_frame_to_downstreams(const FrameMessage_t &frame_msg);

    // read next frame and return true if success
    virtual bool _read_frame_local(cv::Mat &frame);

    // read next frame from orbbec net device and return true if success
    virtual bool _read_frame_orbbec(cv::Mat &frame);

    // publish frame msg for visualization
    virtual void _publish_frame(const cv::Mat &frame);

  private:
    //! Declare all parameters (non-overridable)
    //! should be called in subclass constructor
    void _declare_all_parameters();

  protected:
    // member of downstreams
    std::map<std::string, std::shared_ptr<Downstream_t>> m_downstreams;

    // configuration
    std::shared_ptr<InitConfig_t> m_init_config;
    std::shared_ptr<RuntimeConfig_t> m_runtime_config;

    // impl data
    std::shared_ptr<RedoxiVideoReaderImpl> m_impl;

    // status code
    int m_status_code = NodeStatusCode::BEFORE_INIT;

    // publish info for visualization
    bool m_publish_image = false;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_topic_image;

    // current frame number read by this reader
    // -1 means not read any frame, starting from 0 regardless of the absolute frame number in cv::VideoCapture
    int64_t m_frame_number = -1;
};

} // namespace redoxi_works

#endif // REDOXI_VIDEO_READER_BASE__REDOXI_VIDEO_READER_BASE_HPP_
