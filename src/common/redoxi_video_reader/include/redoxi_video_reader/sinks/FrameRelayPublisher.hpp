#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <redoxi_video_reader/base/VideoReaderBase.hpp>
#include <redoxi_public_msgs/action/process_frame.hpp>
#include <future>

namespace redoxi_works
{

struct FrameRelayPublisherImpl;

//! A sink that publishes the frames received by action/publish/service to a ROS 2 topic
class FrameRelayPublisher : public rclcpp::Node
{
    friend FrameRelayPublisherImpl;

  public:
    //! name of the frame receive action
    inline static std::string DefaultFrameReceiveActionName = "in/action";
    //! name of the image topic
    inline static std::string DefaultImageTopicName = "out/image_raw";
    //! name of the compressed image topic
    inline static std::string DefaultCompressedImageTopicName = "out/image_compressed";
    //! default buffer size for the async processing
    inline static constexpr int DefaultAsyncBufferSize = 1;
    //! default publish queue size
    inline static constexpr int DefaultPublishQueueSize = 20;

    //! rename the action type to FrameReceiveAction_t
    using FrameReceiveAction_t = redoxi_public_msgs::action::ProcessFrame;
    using FrameReceiveGoalHandle_t = rclcpp_action::ServerGoalHandle<FrameReceiveAction_t>;

    struct InitConfig_t {
        virtual ~InitConfig_t() = default;
        std::string frame_receive_action_name = DefaultFrameReceiveActionName;
        std::string image_topic_name = DefaultImageTopicName;
        std::string compressed_image_topic_name = DefaultCompressedImageTopicName;
        int publish_queue_size = DefaultPublishQueueSize;

        bool publish_raw_image = true;
        bool publish_compressed_image = true;

        //! If true, the node will use async processing, otherwise it will use sync processing
        bool use_async = false;

        //! The buffer size for the async processing
        int async_buffer_size = DefaultAsyncBufferSize;
    };

    struct FrameDeliveryPayload_t {
        std::shared_ptr<FrameReceiveGoalHandle_t> goal_handle;
        int64_t ith_received_frame = -1;
        int64_t ith_sent_frame = -1;
    };

    //! The task type for delivering frames
    struct FrameDeliveryTask_t {
        virtual ~FrameDeliveryTask_t() = default;
        boost::uuids::uuid goal_uuid;
        std::shared_future<FrameDeliveryPayload_t> payload;
    };

  public:
    FrameRelayPublisher(const std::string &name, const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    virtual ~FrameRelayPublisher();

    //! Initializes the node, creating the publishers and action servers
    virtual void init(std::shared_ptr<InitConfig_t> config);

    //! Gets the initialization configuration
    const auto &get_init_config() const
    {
        return m_config;
    }

  protected:
    //! The callback function for the goal request
    virtual rclcpp_action::GoalResponse
        _on_goal_received(const rclcpp_action::GoalUUID &uuid,
                          std::shared_ptr<const FrameReceiveAction_t::Goal> goal);

    //! The callback function for the accepted goal
    virtual void _on_goal_accepted(std::shared_ptr<FrameReceiveGoalHandle_t> goal_handle);

    //! The callback function for the goal cancel request
    virtual rclcpp_action::CancelResponse
        _on_goal_canceled(std::shared_ptr<FrameReceiveGoalHandle_t> goal_handle);

    //! publish the frame to the topic
    virtual int _deliver_frame(FrameDeliveryTask_t &task);

    //! try to enqueue the goal, return 0 if success, otherwise return -1
    virtual int _try_enqueue_goal(const rclcpp_action::GoalUUID &uuid,
                                  const FrameReceiveAction_t::Goal &goal);

    //! resolve the goal, return 0 if success, otherwise return -1
    virtual int _resolve_goal(std::shared_ptr<FrameReceiveGoalHandle_t> goal_handle);

  protected:
    //! The publisher for the image topic
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_image_publisher;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr m_compressed_image_publisher;

    //! action server for frame receiving
    rclcpp_action::Server<FrameReceiveAction_t>::SharedPtr m_frame_receive_action_server;

    //! The initialization configuration
    std::shared_ptr<InitConfig_t> m_config;

    //! The implementation of the node
    std::unique_ptr<FrameRelayPublisherImpl> m_impl;
};

} // namespace redoxi_works