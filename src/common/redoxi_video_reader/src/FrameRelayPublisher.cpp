#include <redoxi_video_reader/sinks/FrameRelayPublisher.hpp>
#include <redoxi_common_cpp/async_processor/SingleBufferExecNode.hpp>
#include <redoxi_common_cpp/redoxi_ros_util.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <tbb/tbb.h>
#include <functional>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_hash.hpp>
#include <future>

using namespace std::placeholders;

namespace redoxi_works
{

struct FrameRelayPublisherImpl {
    using FrameReceiveAction_t = FrameRelayPublisher::FrameReceiveAction_t;
    using FrameReceiveGoalHandle_t = FrameRelayPublisher::FrameReceiveGoalHandle_t;
    using FrameDeliveryTask_t = FrameRelayPublisher::FrameDeliveryTask_t;
    using FrameDeliveryPayload_t = FrameRelayPublisher::FrameDeliveryPayload_t;

    //! The graph for the node
    std::shared_ptr<tbb::flow::graph> m_async_graph;
    std::shared_ptr<async_processor::SingleBufferExecNode<FrameDeliveryTask_t>> m_async_node;

    //! mapping from goal UUID to payload promise
    tbb::concurrent_unordered_map<
        boost::uuids::uuid,
        std::promise<FrameDeliveryPayload_t>,
        boost::hash<boost::uuids::uuid>>
        m_goal2payload;
};

FrameRelayPublisher::FrameRelayPublisher(const std::string &name, const rclcpp::NodeOptions &options)
    : rclcpp::Node(name, options)
{
    m_impl = std::make_unique<FrameRelayPublisherImpl>();
}

FrameRelayPublisher::~FrameRelayPublisher()
{
    if (m_impl && m_impl->m_async_graph) {
        m_impl->m_async_graph->wait_for_all();
    }
}

void FrameRelayPublisher::init(std::shared_ptr<InitConfig_t> config)
{
    m_config = config;

    // create the async processing graph
    m_impl->m_async_graph = std::make_shared<tbb::flow::graph>();
    m_impl->m_async_node = std::make_shared<async_processor::SingleBufferExecNode<FrameDeliveryTask_t>>(*m_impl->m_async_graph);
    m_impl->m_async_node->set_input_data_buffer_size(m_config->async_buffer_size);
    m_impl->m_async_node->build();

    // create the action server
    m_frame_receive_action_server =
        rclcpp_action::create_server<FrameReceiveAction_t>(
            this,
            config->frame_receive_action_name,
            std::bind(&FrameRelayPublisher::_on_goal_received, this, _1, _2),
            std::bind(&FrameRelayPublisher::_on_goal_canceled, this, _1),
            std::bind(&FrameRelayPublisher::_on_goal_accepted, this, _1));

    // create publisher
    m_image_publisher = this->create_publisher<sensor_msgs::msg::Image>(
        config->image_topic_name, config->publish_queue_size);
    m_compressed_image_publisher = this->create_publisher<sensor_msgs::msg::CompressedImage>(
        config->compressed_image_topic_name, config->publish_queue_size);
}

int FrameRelayPublisher::_deliver_frame(FrameDeliveryTask_t &task)
{
    auto payload = task.payload.get();
    const auto &incoming_frame = payload.goal_handle->get_goal()->frame;
    auto msg_uuid = uuid_from_ros_msg(payload.goal_handle->get_goal()->x_uid);
    // TODO: here


    sensor_msgs::msg::Image msg_raw;
    sensor_msgs::msg::CompressedImage msg_compressed;
    if (m_config->publish_raw_image) {
        if (!incoming_frame.raw_image.data.empty()) {
            msg_raw = incoming_frame.raw_image;
        }
        RCLCPP_DEBUG(this->get_logger(), "[_deliver_frame()] <Publishing raw image>");
        m_image_publisher->publish(msg_raw);
    }

    if (m_config->publish_compressed_image) {
        if (!incoming_frame.encoded_image.data.empty()) {
            msg_compressed = incoming_frame.encoded_image;
        } else {
            const auto &_raw = incoming_frame.raw_image;
            if (!_raw.data.empty()) {
                //! Convert raw image to OpenCV format
                auto cv_ptr = cv_bridge::toCvCopy(_raw, sensor_msgs::image_encodings::BGR8);

                //! Encode the image as JPEG
                std::vector<uchar> jpeg_buffer;
                cv::imencode(".jpg", cv_ptr->image, jpeg_buffer);

                //! Fill the compressed image message
                msg_compressed.format = "jpeg";
                msg_compressed.data = jpeg_buffer;
            }
        }
        RCLCPP_DEBUG(this->get_logger(), "[_deliver_frame()] <Publishing compressed image>");
        m_compressed_image_publisher->publish(msg_compressed);
    }

    return 0;
}

//! The callback function for the goal request
rclcpp_action::GoalResponse
    FrameRelayPublisher::_on_goal_received(const rclcpp_action::GoalUUID &uuid,
                                           std::shared_ptr<const FrameReceiveAction_t::Goal> goal)
{
    //! Create boost uuid from the goal UUID and the message UUID
    boost::uuids::uuid msg_uuid;
    std::copy(goal->x_uid.uuid.begin(), goal->x_uid.uuid.end(), msg_uuid.begin());
    boost::uuids::uuid goal_uuid;
    std::copy(uuid.begin(), uuid.end(), goal_uuid.begin());

    RCLCPP_DEBUG(this->get_logger(), "Received goal: [msg_uuid]=%s, [goal_uuid]=%s",
                 boost::uuids::to_string(msg_uuid).c_str(), boost::uuids::to_string(goal_uuid).c_str());


    if (m_config->use_async) {
        // just push the goal to the async node
        FrameDeliveryTask_t d_task;
        d_task.goal_uuid = goal_uuid;
        std::promise<FrameDeliveryPayload_t> payload_promise;
        d_task.payload = payload_promise.get_future();

        // try to push the task to the async node
        if (!m_impl->m_async_node->put_data(d_task)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to enqueue the task, rejecting: [msg_uuid]=%s, [goal_uuid]=%s",
                         boost::uuids::to_string(msg_uuid).c_str(), boost::uuids::to_string(goal_uuid).c_str());
            return rclcpp_action::GoalResponse::REJECT;
        } else {
            RCLCPP_DEBUG(this->get_logger(), "Enqueued the task: [msg_uuid]=%s, [goal_uuid]=%s",
                         boost::uuids::to_string(msg_uuid).c_str(), boost::uuids::to_string(goal_uuid).c_str());

            // save this promise to the map
            m_impl->m_goal2payload[goal_uuid] = std::move(payload_promise);
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }
    } else {
        RCLCPP_DEBUG(this->get_logger(), "Accepting goal: [msg_uuid]=%s, [goal_uuid]=%s",
                     boost::uuids::to_string(msg_uuid).c_str(), boost::uuids::to_string(goal_uuid).c_str());
        // do frame publishing in execution callback
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
}

} // namespace redoxi_works
