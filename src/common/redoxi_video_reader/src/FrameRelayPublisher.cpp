#include <redoxi_video_reader/sinks/FrameRelayPublisher.hpp>
#include <redoxi_common_cpp/async_processor/SingleBufferExecNode.hpp>
#include <tbb/tbb.h>
#include <functional>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_io.hpp>

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

    //! mapping from goal UUID to goal handle
    tbb::concurrent_unordered_map<
        boost::uuids::uuid,
        std::shared_ptr<FrameDeliveryPayload_t>>
        m_goal_handles;
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

    // create the action server
    m_frame_receive_action_server =
        rclcpp_action::create_server<FrameReceiveAction_t>(
            this,
            config->frame_receive_action_name,
            std::bind(&FrameRelayPublisher::_on_goal_received, this, _1, _2),
            std::bind(&FrameRelayPublisher::_on_goal_canceled, this, _1),
            std::bind(&FrameRelayPublisher::_on_goal_accepted, this, _1));
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

    // TODO: implement the sync processing
    if (m_config->use_async) {
        // just push the goal to the async node
        FrameDeliveryTask_t task{goal_uuid, cv::Mat(), goal->frame_number};
        m_impl->m_async_node->push(FrameDeliveryTask_t{goal_uuid, cv::Mat(), goal->frame_number});
    }

    //! Accept the goal
    RCLCPP_INFO(this->get_logger(), "Accepting goal for frame number: %ld", goal->frame_number);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}


} // namespace redoxi_works
