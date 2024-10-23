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
#include <random>

using namespace std::placeholders;

namespace redoxi_works
{

//! Global static variable to control thread ID printing
static const bool print_thread_id = true;

struct TbbBoostUuidHash {
    //! Hash function for boost::uuids::uuid to be used with tbb::concurrent_hash_map
    size_t hash(const boost::uuids::uuid &id) const
    {
        return boost::hash<boost::uuids::uuid>{}(id);
    }

    //! Equality comparison for boost::uuids::uuid
    bool equal(const boost::uuids::uuid &id1, const boost::uuids::uuid &id2) const
    {
        return id1 == id2;
    }
};

struct FrameRelayPublisherImpl {
    inline constexpr static size_t DefaultPayloadMapSize = 100;
    using FrameReceiveAction_t = FrameRelayPublisher::FrameReceiveAction_t;
    using FrameReceiveGoalHandle_t = FrameRelayPublisher::FrameReceiveGoalHandle_t;
    using FrameDeliveryTask_t = FrameRelayPublisher::FrameDeliveryTask_t;
    using FrameDeliveryPayload_t = FrameRelayPublisher::FrameDeliveryPayload_t;
    using FramePayloadProducer_t = std::packaged_task<FrameDeliveryPayload_t(std::shared_ptr<FrameReceiveGoalHandle_t>)>;

    //! The type for the registered goal, which is cached in the hash map
    using RegisteredGoal_t = std::pair<FrameDeliveryTask_t, FramePayloadProducer_t>;

    //! The graph for the node
    std::shared_ptr<tbb::flow::graph> m_async_graph;
    std::shared_ptr<async_processor::SingleBufferExecNode<FrameDeliveryTask_t>> m_async_node;

    //! mapping from goal UUID to payload promise
    tbb::concurrent_hash_map<
        boost::uuids::uuid,
        RegisteredGoal_t,
        TbbBoostUuidHash>
        m_goal2payload{DefaultPayloadMapSize};

    //! frame count bookkeeping
    std::atomic<size_t> m_num_sent_frame{0};
    std::atomic<size_t> m_num_received_frame{0};
};

FrameRelayPublisher::FrameRelayPublisher(const std::string &name, const rclcpp::NodeOptions &options)
    : rclcpp::Node(name, options)
{
    this->get_logger().set_level(rclcpp::Logger::Level::Debug);

    // declare parameters
    auto ret = declare_default_parameters_for_node(this);
    if (ret != 0) {
        RDX_RAISE_ERROR("[{}] Failed to declare default parameters", this->get_name());
    }
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
    m_impl->m_async_node->set_output_callback([this](const auto &output) -> int {
        auto task = std::get<0>(output);
        auto ret = _deliver_frame(task);
        m_impl->m_goal2payload.erase(task.goal_uuid);
        return ret;
    });
    m_impl->m_async_node->build();

    // create the action server
    auto server_opt = rcl_action_server_get_default_options();


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
    auto msg_uuid = to_boost_uuid(payload.goal_handle->get_goal()->x_uid);

    sensor_msgs::msg::Image msg_raw;
    sensor_msgs::msg::CompressedImage msg_compressed;
    // if (m_config->publish_raw_image) {
    //     if (!incoming_frame.raw_image.data.empty()) {
    //         msg_raw = incoming_frame.raw_image;
    //     }
    //     RDX_LOG_INFO(this, __func__, print_thread_id, "[msg_uuid={}] Publishing raw image", boost::uuids::to_string(msg_uuid));
    //     m_image_publisher->publish(msg_raw);
    // }

    // if (m_config->publish_compressed_image) {
    //     if (!incoming_frame.encoded_image.data.empty()) {
    //         msg_compressed = incoming_frame.encoded_image;
    //     } else {
    //         const auto &_raw = incoming_frame.raw_image;
    //         if (!_raw.data.empty()) {
    //             //! Convert raw image to OpenCV format
    //             auto cv_ptr = cv_bridge::toCvCopy(_raw, sensor_msgs::image_encodings::BGR8);

    //             //! Encode the image as JPEG
    //             std::vector<uchar> jpeg_buffer;
    //             cv::imencode(".jpg", cv_ptr->image, jpeg_buffer);

    //             //! Fill the compressed image message
    //             msg_compressed.format = "jpeg";
    //             msg_compressed.data = jpeg_buffer;
    //         }
    //     }
    //     RDX_LOG_INFO(this, __func__, print_thread_id, "[msg_uuid={}] Publishing compressed image", boost::uuids::to_string(msg_uuid));
    //     m_compressed_image_publisher->publish(msg_compressed);
    // }

    // signal the goal as success
    auto result = std::make_shared<FrameReceiveAction_t::Result>();
    result->return_code = 0;
    payload.goal_handle->succeed(result);

    return 0;
}

int FrameRelayPublisher::_try_enqueue_goal(
    const rclcpp_action::GoalUUID &uuid,
    const FrameReceiveAction_t::Goal &goal)
{
    auto goal_uuid = to_boost_uuid(uuid);
    auto msg_uuid = to_boost_uuid(goal.x_uid);
    auto &async_node = *m_impl->m_async_node;

    // save this with the payload producer
    int64_t ith_received_frame = m_impl->m_num_received_frame++;

    // just push the goal to the async node
    FrameRelayPublisherImpl::FramePayloadProducer_t payload_producer(
        [ith_received_frame](auto goal_handle) {
            FrameDeliveryPayload_t payload;
            payload.goal_handle = goal_handle;
            payload.ith_received_frame = ith_received_frame;
            return payload;
        });
    FrameDeliveryTask_t d_task;
    d_task.goal_uuid = goal_uuid;
    d_task.payload = payload_producer.get_future();

    if (m_config->use_async) {
        // for async case, try push the task to the async node
        if (async_node.put_data(d_task)) {
            RDX_LOG_INFO(this, __func__, print_thread_id, "[msg_uuid={}] Goal enqueued (async mode)",
                         boost::uuids::to_string(msg_uuid));
            // ok, save the promise to the map
            decltype(m_impl->m_goal2payload)::accessor acc;
            m_impl->m_goal2payload.insert(acc, goal_uuid);
            acc->second = FrameRelayPublisherImpl::RegisteredGoal_t{std::move(d_task), std::move(payload_producer)};
            return 0;
        } else {
            // failed to push the task to the async node
            RDX_LOG_ERROR(this, __func__, "[msg_uuid={}] Failed to queue goal",
                          boost::uuids::to_string(msg_uuid));
            return -1;
        }
    } else {
        // for sync case, always accept the goal
        RDX_LOG_INFO(this, __func__, print_thread_id, "[msg_uuid={}] Goal enqueued (sync mode)",
                     boost::uuids::to_string(msg_uuid));
        decltype(m_impl->m_goal2payload)::accessor acc;
        m_impl->m_goal2payload.insert(acc, goal_uuid);
        acc->second = FrameRelayPublisherImpl::RegisteredGoal_t{std::move(d_task), std::move(payload_producer)};
        return 0;
    }
}


//! The callback function for the goal request
//! if accepted, the frame will be sent in execution callback
rclcpp_action::GoalResponse
    FrameRelayPublisher::_on_goal_received(const rclcpp_action::GoalUUID &uuid,
                                           std::shared_ptr<const FrameReceiveAction_t::Goal> goal)
{
    //! Create boost uuid from the message UUID
    auto msg_uuid = to_boost_uuid(goal->x_uid);

    //! Is this goal just a ping request?
    bool is_ping_request = goal->x_control.code == goal->x_control.PING;
    RDX_LOG_INFO(this, __func__, print_thread_id, "[msg_uuid={}] Received goal, is_ping_request={}",
                 boost::uuids::to_string(msg_uuid), is_ping_request ? "true" : "false");

    if (is_ping_request) {
        //! Randomly reject ping requests with a probability of 0.3
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> dis(0.0, 1.0);
        if (dis(gen) < 0.3) {
            RDX_LOG_INFO(this, __func__, print_thread_id, "[msg_uuid={}] Randomly rejecting ping request",
                         boost::uuids::to_string(msg_uuid));
            return rclcpp_action::GoalResponse::REJECT;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    // for async mode, try to enqueue the goal
    auto ret = _try_enqueue_goal(uuid, *goal);
    if (ret == 0) {
        RDX_LOG_INFO(this, __func__, print_thread_id, "[msg_uuid={}] Goal accepted",
                     boost::uuids::to_string(msg_uuid));
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    } else {
        RDX_LOG_ERROR(this, __func__, "[msg_uuid={}] Goal rejected",
                      boost::uuids::to_string(msg_uuid));
        return rclcpp_action::GoalResponse::REJECT;
    }
}

int FrameRelayPublisher::_resolve_goal(std::shared_ptr<FrameReceiveGoalHandle_t> goal_handle)
{
    auto goal_uuid = to_boost_uuid(goal_handle->get_goal_id());
    bool is_ping_request = goal_handle->get_goal()->x_control.code == goal_handle->get_goal()->x_control.PING;
    RDX_LOG_INFO(this, __func__, print_thread_id, "[msg_uuid={}] Resolving goal, is_ping_request={}",
                 boost::uuids::to_string(goal_uuid), is_ping_request ? "true" : "false");
    if (is_ping_request) {
        // ping request does not need to resolve, not in the map either
        return 0;
    }

    //! Get the promise from the concurrent hash map
    decltype(m_impl->m_goal2payload)::accessor acc;
    if (m_impl->m_goal2payload.find(acc, goal_uuid)) {
        //! Create the payload

        //! Resolve the promise by setting the value
        auto &resolver = acc->second.second;
        resolver(goal_handle);

        RDX_LOG_INFO(this, __func__, print_thread_id, "[msg_uuid={}] Goal resolved successfully",
                     boost::uuids::to_string(goal_uuid));
        return 0;
    } else {
        // this should never happen
        RDX_RAISE_ERROR("[msg_uuid={}] Failed to find goal in map",
                        boost::uuids::to_string(goal_uuid));
        return -1;
    }
    return 0;
}

//! The callback function for the accepted goal
void FrameRelayPublisher::_on_goal_accepted(std::shared_ptr<FrameReceiveGoalHandle_t> goal_handle)
{
    auto goal_uuid = to_boost_uuid(goal_handle->get_goal_id());
    auto msg_uuid = to_boost_uuid(goal_handle->get_goal()->x_uid);

    //! Is this goal just a ping request?
    bool is_ping_request = goal_handle->get_goal()->x_control.code == goal_handle->get_goal()->x_control.PING;
    RDX_LOG_INFO(this, __func__, print_thread_id, "[msg_uuid={}] Goal execution started, is_ping_request={}",
                 boost::uuids::to_string(msg_uuid), is_ping_request ? "true" : "false");
    if (is_ping_request) {
        RDX_LOG_INFO(this, __func__, print_thread_id, "[msg_uuid={}] Signaling goal as success (ping request)",
                     boost::uuids::to_string(msg_uuid));

        auto result = std::make_shared<FrameReceiveAction_t::Result>();
        result->return_code = 0;
        goal_handle->succeed(result);
        // goal_handle->canceled(result);

        RDX_LOG_INFO(this, __func__, print_thread_id, "[msg_uuid={}] Signaled goal as success (ping request)",
                     boost::uuids::to_string(msg_uuid));
        return;
    }

    auto ret = _resolve_goal(goal_handle);
    if (ret != 0) {
        RDX_LOG_ERROR(this, __func__, "[msg_uuid={}] Failed to resolve goal",
                      boost::uuids::to_string(msg_uuid));
    } else {
        RDX_LOG_INFO(this, __func__, print_thread_id, "[msg_uuid={}] Goal resolved successfully",
                     boost::uuids::to_string(msg_uuid));
    }

    // deliver the frame in sync mode
    if (!m_config->use_async) {
        decltype(m_impl->m_goal2payload)::accessor acc;
        if (m_impl->m_goal2payload.find(acc, goal_uuid)) {
            _deliver_frame(acc->second.first);

            // done, remove the goal from the map
            m_impl->m_goal2payload.erase(acc);
        }
    } else {
        // let the async node handle the frame delivery
        // do nothing
    }
}

//! The callback function for the goal cancel request
rclcpp_action::CancelResponse
    FrameRelayPublisher::_on_goal_canceled(std::shared_ptr<FrameReceiveGoalHandle_t> goal_handle)
{
    auto goal_uuid = to_boost_uuid(goal_handle->get_goal_id());
    auto msg_uuid = to_boost_uuid(goal_handle->get_goal()->x_uid);

    RDX_LOG_INFO(this, __func__, print_thread_id, "[msg_uuid={}] Goal cancellation requested",
                 boost::uuids::to_string(msg_uuid));

    //! Remove the goal from the hash map
    decltype(m_impl->m_goal2payload)::accessor acc;
    if (m_impl->m_goal2payload.find(acc, goal_uuid)) {
        //! Get the registered goal and resolve it before erasing
        //! avoid blocking the callers who awaits the shared_future
        auto &resolver = acc->second.second;
        resolver(goal_handle);

        // done, remove the goal from the map
        m_impl->m_goal2payload.erase(acc);
        RDX_LOG_INFO(this, __func__, print_thread_id, "[msg_uuid={}] Goal resolved and removed from map",
                     boost::uuids::to_string(msg_uuid));
    }

    //! Return accept
    return rclcpp_action::CancelResponse::ACCEPT;
}

} // namespace redoxi_works
