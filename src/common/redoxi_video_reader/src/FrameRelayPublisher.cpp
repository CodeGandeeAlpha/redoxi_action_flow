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
#include <nlohmann/json.hpp>

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
    inline constexpr static size_t DefaultPayloadMapSize = 10000;
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
        FrameDeliveryTask_t,
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
    RDX_LOG_INFO(this, __func__, print_thread_id, "Creating async processing graph");
    m_impl->m_async_graph = std::make_shared<tbb::flow::graph>();
    m_impl->m_async_node = std::make_shared<async_processor::SingleBufferExecNode<FrameDeliveryTask_t>>(*m_impl->m_async_graph);
    m_impl->m_async_node->set_input_data_buffer_size(m_config->goal_buffer_size);
    m_impl->m_async_node->set_output_callback([this](const auto &output) -> int {
        RDX_LOG_INFO(this, __func__, print_thread_id, "Delivering frame using output callback");
        FrameDeliveryTask_t task = std::get<0>(output);

        RDX_LOG_INFO(this, __func__, print_thread_id, "[goal_uuid={}] Trying to get payload",
                     boost::uuids::to_string(task.goal_uuid));
        auto payload = task.payload.get();
        if (payload->is_valid()) {
            auto msg_uuid = to_boost_uuid(payload->goal_handle->get_goal()->x_uid);

            RDX_LOG_INFO(this, __func__, print_thread_id, "[msg_uuid={}][goal_uuid={}] Delivering frame",
                         boost::uuids::to_string(msg_uuid), boost::uuids::to_string(task.goal_uuid));
            auto ret = _deliver_frame(task);
            return ret;
        } else {
            RDX_LOG_ERROR(this, __func__, print_thread_id, "[goal_uuid={}] payload is not valid, skipping",
                          boost::uuids::to_string(task.goal_uuid));
            return 0;
        }
    });
    m_impl->m_async_node->build();

    // create the action server
    auto server_opt = rcl_action_server_get_default_options();
    {
        std::chrono::nanoseconds timeout = std::chrono::milliseconds(300);
        server_opt.result_timeout.nanoseconds = timeout.count();
    }


    RDX_LOG_INFO(this, __func__, print_thread_id, "Creating action server");
    m_frame_receive_action_server =
        rclcpp_action::create_server<FrameReceiveAction_t>(
            this,
            config->frame_receive_action_name,
            std::bind(&FrameRelayPublisher::_on_goal_received, this, _1, _2),
            std::bind(&FrameRelayPublisher::_on_goal_canceled, this, _1),
            std::bind(&FrameRelayPublisher::_on_goal_accepted, this, _1),
            server_opt);

    // create publisher
    RDX_LOG_INFO(this, __func__, print_thread_id, "Creating image publisher");
    m_image_publisher = this->create_publisher<sensor_msgs::msg::Image>(
        config->image_topic_name, config->publish_queue_size);
    m_compressed_image_publisher = this->create_publisher<sensor_msgs::msg::CompressedImage>(
        config->compressed_image_topic_name, config->publish_queue_size);

    RDX_LOG_INFO(this, __func__, print_thread_id, "Initialization completed");
}

int FrameRelayPublisher::_deliver_frame(FrameDeliveryTask_t &task)
{
    RDX_LOG_INFO(this, __func__, print_thread_id, "Trying to get payload");
    auto payload = task.payload.get();

    if (payload->is_valid()) {
        RDX_LOG_INFO(this, __func__, print_thread_id, "Got payload");
        const auto &incoming_frame = payload->goal_handle->get_goal()->frame;
        auto msg_uuid = to_boost_uuid(payload->goal_handle->get_goal()->x_uid);

        RDX_LOG_INFO(this, __func__, print_thread_id, "[msg_uuid={}][goal_uuid={}] Delivering frame",
                     boost::uuids::to_string(msg_uuid), boost::uuids::to_string(task.goal_uuid));

        sensor_msgs::msg::Image msg_raw;
        sensor_msgs::msg::CompressedImage msg_compressed;
        if (m_config->publish_raw_image) {
            if (!incoming_frame.raw_image.data.empty()) {
                msg_raw = incoming_frame.raw_image;
            }
            RDX_LOG_INFO(this, __func__, print_thread_id, "[msg_uuid={}][goal_uuid={}] Publishing raw image",
                         boost::uuids::to_string(msg_uuid), boost::uuids::to_string(task.goal_uuid));
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
            RDX_LOG_INFO(this, __func__, print_thread_id, "[msg_uuid={}][goal_uuid={}] Publishing compressed image",
                         boost::uuids::to_string(msg_uuid), boost::uuids::to_string(task.goal_uuid));
            m_compressed_image_publisher->publish(msg_compressed);
        }

        // signal the goal as success
        RDX_LOG_INFO(this, __func__, print_thread_id, "[msg_uuid={}][goal_uuid={}] Signaling goal as success",
                     boost::uuids::to_string(msg_uuid), boost::uuids::to_string(task.goal_uuid));
        auto result = std::make_shared<FrameReceiveAction_t::Result>();
        result->return_code = 0;
        payload->goal_handle->succeed(result);
    } else {
        RDX_LOG_ERROR(this, __func__, print_thread_id, "[goal_uuid={}] payload is not valid, skipping",
                      boost::uuids::to_string(task.goal_uuid));
    }

    // after this, the goal in the hash map can be erased
    auto erased = m_impl->m_goal2payload.erase(task.goal_uuid);
    if (erased) {
        RDX_LOG_INFO(this, __func__, print_thread_id, "[goal_uuid={}] Erased goal from map, result={}",
                     boost::uuids::to_string(task.goal_uuid), "true");
    } else {
        // may have been canceled
        RDX_LOG_ERROR(this, __func__, print_thread_id, "[goal_uuid={}] Failed to erase goal from map, result={}",
                      boost::uuids::to_string(task.goal_uuid), "false");
    }

    return 0;
}

int FrameRelayPublisher::_resolve_goal_payload(std::shared_ptr<FrameReceiveGoalHandle_t> goal_handle)
{
    // find the goal in the map
    auto goal_uuid = to_boost_uuid(goal_handle->get_goal_id());
    auto msg_uuid = to_boost_uuid(goal_handle->get_goal()->x_uid);
    RDX_LOG_INFO(this, __func__, print_thread_id, "[msg_uuid={}][goal_uuid={}] Resolving goal payload",
                 boost::uuids::to_string(msg_uuid), boost::uuids::to_string(goal_uuid));

    decltype(m_impl->m_goal2payload)::accessor acc;
    if (m_impl->m_goal2payload.find(acc, goal_uuid)) {
        RDX_LOG_INFO(this, __func__, print_thread_id, "[msg_uuid={}][goal_uuid={}] Found goal in map, resolve it",
                     boost::uuids::to_string(msg_uuid), boost::uuids::to_string(goal_uuid));
        auto &task = acc->second;
        FrameDeliveryPayload_t::Ptr payload = std::make_shared<FrameDeliveryPayload_t>();
        payload->goal_handle = goal_handle;
        task._payload_promise->set_value(payload);
        return 0;
    } else {
        // may have been canceled
        RDX_LOG_ERROR(this, __func__, print_thread_id, "[msg_uuid={}][goal_uuid={}] Failed to find goal in map",
                      boost::uuids::to_string(msg_uuid), boost::uuids::to_string(goal_uuid));
    }
    return -1;
}

int FrameRelayPublisher::_create_delivery_task(FrameDeliveryTask_t &output_task,
                                               const rclcpp_action::GoalUUID &uuid,
                                               const FrameReceiveAction_t::Goal &goal,
                                               const FrameDeliveryTask_t *preset)
{
    auto goal_uuid = to_boost_uuid(uuid);
    auto msg_uuid = to_boost_uuid(goal.x_uid);
    output_task.goal_uuid = goal_uuid;
    if (preset) {
        output_task.ith_received_frame = preset->ith_received_frame;
        output_task.ith_sent_frame = preset->ith_sent_frame;
    }

    // create the payload promise and future
    output_task._payload_promise = std::make_shared<std::promise<FrameDeliveryPayload_t::Ptr>>();
    output_task.payload = output_task._payload_promise->get_future();

    // done
    RDX_LOG_INFO(this, __func__, print_thread_id, "[msg_uuid={}][goal_uuid={}] Created delivery task",
                 boost::uuids::to_string(msg_uuid), boost::uuids::to_string(goal_uuid));
    return 0;
}

int FrameRelayPublisher::_try_enqueue_goal(const rclcpp_action::GoalUUID &uuid, const FrameReceiveAction_t::Goal &goal)
{
    auto goal_uuid = to_boost_uuid(uuid);
    auto msg_uuid = to_boost_uuid(goal.x_uid);

    auto &async_node = *m_impl->m_async_node;
    int64_t ith_received_frame = m_impl->m_num_received_frame++;
    FrameDeliveryTask_t preset;
    preset.ith_received_frame = ith_received_frame;
    FrameDeliveryTask_t d_task;
    auto ret = _create_delivery_task(d_task, uuid, goal, &preset);
    if (ret != 0) {
        RDX_LOG_ERROR(this, __func__, "[msg_uuid={}][goal_uuid={}] Failed to create delivery task",
                      boost::uuids::to_string(msg_uuid), boost::uuids::to_string(goal_uuid));
        return ret;
    }

    // reserve a slot in the map first, otherwise we will problem of hash conflict
    // particularly in multi-threaded environment
    using MapAccessor_t = decltype(m_impl->m_goal2payload)::accessor;
    MapAccessor_t acc;
    bool already_exists = !m_impl->m_goal2payload.insert(acc, goal_uuid);
    if (already_exists) {
        RDX_LOG_ERROR(this, __func__, "[msg_uuid={}][goal_uuid={}] Goal already exists, skipping",
                      boost::uuids::to_string(msg_uuid), boost::uuids::to_string(goal_uuid));
        return -1;
    }

    if (m_config->use_async) {
        // for async case, try push the task to the async node
        if (async_node.put_data(d_task)) {
            RDX_LOG_INFO(this, __func__, print_thread_id, "[msg_uuid={}][goal_uuid={}] Goal enqueued (async mode)",
                         boost::uuids::to_string(msg_uuid), boost::uuids::to_string(d_task.goal_uuid));
            //! save the task to map
            acc->second = d_task;
            RDX_LOG_INFO(this, __func__, print_thread_id, "[msg_uuid={}][goal_uuid={}] Goal saved to map",
                         boost::uuids::to_string(msg_uuid), boost::uuids::to_string(d_task.goal_uuid));
            return 0;
        } else {
            // failed to push the task to the async node
            RDX_LOG_ERROR(this, __func__, "[msg_uuid={}][goal_uuid={}] Failed to queue goal",
                          boost::uuids::to_string(msg_uuid), boost::uuids::to_string(goal_uuid));
            // remove the goal from the map, we do not need it
            m_impl->m_goal2payload.erase(acc);
            return -1;
        }
    } else {
        // for sync case, always accept the goal
        RDX_LOG_INFO(this, __func__, print_thread_id, "[msg_uuid={}][goal_uuid={}] Goal enqueued (sync mode)",
                     boost::uuids::to_string(msg_uuid), boost::uuids::to_string(goal_uuid));

        // save the task to map
        acc->second = d_task;
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
        // std::random_device rd;
        // std::mt19937 gen(rd());
        // std::uniform_real_distribution<> dis(0.0, 1.0);
        // if (dis(gen) < 0.3) {
        //     RDX_LOG_INFO(this, __func__, print_thread_id, "[msg_uuid={}] Randomly rejecting ping request",
        //                  boost::uuids::to_string(msg_uuid));
        //     return rclcpp_action::GoalResponse::REJECT;
        // }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    // queue the goal, whether async or sync
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

//! The callback function for the accepted goal
void FrameRelayPublisher::_on_goal_accepted(std::shared_ptr<FrameReceiveGoalHandle_t> goal_handle)
{
    auto goal_uuid = to_boost_uuid(goal_handle->get_goal_id());
    auto msg_uuid = to_boost_uuid(goal_handle->get_goal()->x_uid);

    //! Is this goal just a ping request? If yes, directly signal the goal as success
    bool is_ping_request = goal_handle->get_goal()->x_control.code == goal_handle->get_goal()->x_control.PING;
    RDX_LOG_INFO(this, __func__, print_thread_id, "[msg_uuid={}][goal_uuid={}] Goal execution started, is_ping_request={}",
                 boost::uuids::to_string(msg_uuid), boost::uuids::to_string(goal_uuid), is_ping_request ? "true" : "false");

    if (is_ping_request) {
        RDX_LOG_INFO(this, __func__, print_thread_id, "[msg_uuid={}][goal_uuid={}] Signaling goal as success (ping request)",
                     boost::uuids::to_string(msg_uuid), boost::uuids::to_string(goal_uuid));

        auto result = std::make_shared<FrameReceiveAction_t::Result>();
        result->return_code = 0;
        goal_handle->succeed(result);

        RDX_LOG_INFO(this, __func__, print_thread_id, "[msg_uuid={}][goal_uuid={}] Signaled goal as success (ping request)",
                     boost::uuids::to_string(msg_uuid), boost::uuids::to_string(goal_uuid));
    } else {

        // resolve the goal payload
        auto ret = _resolve_goal_payload(goal_handle);
        if (ret != 0) {
            RDX_LOG_ERROR(this, __func__, "[msg_uuid={}][goal_uuid={}] Failed to resolve goal payload",
                          boost::uuids::to_string(msg_uuid), boost::uuids::to_string(goal_uuid));
            return;
        }

        // if in sync mode, we need to call _deliver_frame by ourselves
        if (!m_config->use_async) {
            // get task from the map
            FrameDeliveryTask_t task;
            {
                // find the goal in the map, wrap it in a scope to avoid long living reference
                decltype(m_impl->m_goal2payload)::accessor acc;
                if (m_impl->m_goal2payload.find(acc, goal_uuid)) {
                    task = acc->second;
                } else {
                    RDX_LOG_ERROR(this, __func__, "[msg_uuid={}][goal_uuid={}] Failed to find goal in map",
                                  boost::uuids::to_string(msg_uuid), boost::uuids::to_string(goal_uuid));
                    return;
                }
            }
            _deliver_frame(task);
        } else {
            // do nothing, async work will be done in the tbb graph
        }
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
        // just remove the goal from the map
        m_impl->m_goal2payload.erase(acc);
        RDX_LOG_INFO(this, __func__, print_thread_id, "[msg_uuid={}] Goal removed from map",
                     boost::uuids::to_string(msg_uuid));
    }

    //! Return accept
    return rclcpp_action::CancelResponse::ACCEPT;
}

void FrameRelayPublisher::InitConfig_t::from_parameters(const rclcpp::Node *node)
{
    // example json parameters
    /*
    {
        "declare_params": {},
        "init_config": {
            "frame_receive_action_name": "/video_source/out/action",
            "image_topic_name": "/video_source/out/image",
            "compressed_image_topic_name": "/video_source/out/compressed_image",
            "publish_queue_size": 10,
            "publish_raw_image": True,
            "publish_compressed_image": True,
            "use_async": True,
            "goal_buffer_size": 10,
        },
    }
    */
    //! Load parameters from the node
    using JsonPointer_t = nlohmann::json::json_pointer;
    auto json_params = RDX_GET_JSON_PARAM_FROM_NODE(node);
    if (json_params.empty()) {
        RDX_LOG_ERROR(node, __func__, "Failed to get JSON parameters from node");
        return;
    }

    //! Load the parameters
    //! Load parameters using JSON pointers
    if (json_params.contains(JsonPointer_t("/init_config"))) {
        const auto &init_config = json_params[JsonPointer_t("/init_config")];

        if (init_config.contains(JsonPointer_t("/frame_receive_action_name"))) {
            frame_receive_action_name = init_config[JsonPointer_t("/frame_receive_action_name")].get<std::string>();
            RDX_LOG_DEBUG(node, __func__, "frame_receive_action_name: {}", frame_receive_action_name);
        }

        if (init_config.contains(JsonPointer_t("/image_topic_name"))) {
            image_topic_name = init_config[JsonPointer_t("/image_topic_name")].get<std::string>();
            RDX_LOG_DEBUG(node, __func__, "image_topic_name: {}", image_topic_name);
        }

        if (init_config.contains(JsonPointer_t("/compressed_image_topic_name"))) {
            compressed_image_topic_name = init_config[JsonPointer_t("/compressed_image_topic_name")].get<std::string>();
            RDX_LOG_DEBUG(node, __func__, "compressed_image_topic_name: {}", compressed_image_topic_name);
        }

        if (init_config.contains(JsonPointer_t("/publish_queue_size"))) {
            publish_queue_size = init_config[JsonPointer_t("/publish_queue_size")].get<size_t>();
            RDX_LOG_DEBUG(node, __func__, "publish_queue_size: {}", publish_queue_size);
        }

        if (init_config.contains(JsonPointer_t("/publish_raw_image"))) {
            publish_raw_image = init_config[JsonPointer_t("/publish_raw_image")].get<bool>();
            RDX_LOG_DEBUG(node, __func__, "publish_raw_image: {}", publish_raw_image);
        }

        if (init_config.contains(JsonPointer_t("/publish_compressed_image"))) {
            publish_compressed_image = init_config[JsonPointer_t("/publish_compressed_image")].get<bool>();
            RDX_LOG_DEBUG(node, __func__, "publish_compressed_image: {}", publish_compressed_image);
        }

        if (init_config.contains(JsonPointer_t("/use_async"))) {
            use_async = init_config[JsonPointer_t("/use_async")].get<bool>();
            RDX_LOG_DEBUG(node, __func__, "use_async: {}", use_async);
        }

        if (init_config.contains(JsonPointer_t("/goal_buffer_size"))) {
            goal_buffer_size = init_config[JsonPointer_t("/goal_buffer_size")].get<size_t>();
            RDX_LOG_DEBUG(node, __func__, "goal_buffer_size: {}", goal_buffer_size);
        }
    }
}


} // namespace redoxi_works
