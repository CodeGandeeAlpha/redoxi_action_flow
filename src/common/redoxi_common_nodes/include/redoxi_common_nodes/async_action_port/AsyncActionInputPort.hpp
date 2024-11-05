#pragma once

#include <functional>
#include <tbb/concurrent_hash_map.h>
#include <tbb/concurrent_queue.h>

#include <redoxi_common_nodes/async_action_port/input_port_concepts.hpp>
#include <redoxi_common_nodes/async_action_port/AsyncActionInputTypes.hpp>
#include <redoxi_common_cpp/redoxi_tbb_util.hpp>

namespace redoxi_works
{
using TSpec = input_port_types::AsyncActionInputPortSpec;

class AsyncActionInputPort : public IStartStopProtocol
{
  private:
    inline static constexpr auto PRINT_THREAD_ID = false;

  public:
    AsyncActionInputPort(rclcpp::Node *parent_node)
        : m_parent_node(parent_node)
    {
        m_ping_response = std::make_shared<ActionResult_t>();
    }

    virtual ~AsyncActionInputPort() noexcept = default;

    // useful types
    using MasterSpec_t = TSpec; // master specification of this port
    using ActionType_t = TSpec::ActionType_t;
    using ActionGoal_t = TSpec::ActionGoal_t;
    using ActionResult_t = ActionType_t::Result;
    using ActionFeedback_t = ActionType_t::Feedback;
    using ActionDataTrait_t = TSpec::ActionDataTrait_t;
    using InitConfig_t = TSpec::InitConfig_t;
    using SourceData_t = TSpec::ReceiveSourceData_t;
    using GoalHandle_t = SourceData_t::GoalHandle_t;
    using GoalUUID_t = rclcpp_action::GoalUUID;

  public:
    //! Initialize the port
    void init(const InitConfig_t &init_config);

    //! Start the port
    int start() override;

    //! Stop the port
    int stop() override;

  protected:
    //! Handle goal received
    virtual rclcpp_action::GoalResponse _on_goal_received(const rclcpp_action::GoalUUID &uuid,
                                                          std::shared_ptr<const ActionGoal_t> goal)
    {
        // notify user first
        if (m_on_goal_received_callback) {
            int ret = m_on_goal_received_callback(uuid, goal);
            if (ret != 0) {
                return rclcpp_action::GoalResponse::REJECT;
            }
        }

        // TODO: here

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    //! Handle cancel request
    virtual rclcpp_action::CancelResponse _on_goal_cancel_request(std::shared_ptr<GoalHandle_t> goal_handle);

    //! Handle accepted goal
    virtual void _on_goal_accepted(std::shared_ptr<GoalHandle_t> goal_handle);

  protected:
    rclcpp::Node *m_parent_node = nullptr;
    InitConfig_t m_init_config;

    // buffer map for storing source data, unordered
    using SourceDataMap_t = tbb::concurrent_hash_map<boost::uuids::uuid,
                                                     std::shared_ptr<SourceData_t>,
                                                     TbbBoostUuidHash>;
    // buffer map for storing source data, unordered
    SourceDataMap_t m_source_data_map;

    // queue for storing source data, ordered
    using SourceDataQueue_t = tbb::concurrent_queue<std::shared_ptr<SourceData_t>>;
    SourceDataQueue_t m_source_data_queue;

  protected:
    // callbacks

    //! Callback for goal received, return 0 if accepted, otherwise rejected
    std::function<int(const GoalUUID_t &, std::shared_ptr<const ActionGoal_t>)> m_on_goal_received_callback;

    //! Callback for goal enqueued
    //! This happens when the goal is accepted but goal handle is not created yet
    std::function<void(std::shared_ptr<SourceData_t>)> m_on_goal_enqueued_callback;

    //! Callback for goal cancel request, return 0 if accepted, otherwise rejected
    std::function<int(std::shared_ptr<GoalHandle_t>)> m_on_goal_cancel_request_callback;

    //! Callback for goal accepted
    std::function<void(std::shared_ptr<GoalHandle_t>)> m_on_goal_accepted_callback;

  public:
    //! Set callback for goal received
    //! @param callback the callback function, return 0 if accepted, otherwise rejected
    void set_on_goal_received_callback(std::function<int(const GoalUUID_t &, std::shared_ptr<const ActionGoal_t>)> callback)
    {
        m_on_goal_received_callback = callback;
    }

    //! Set callback for goal enqueued
    void set_on_goal_enqueued_callback(std::function<void(std::shared_ptr<SourceData_t>)> callback)
    {
        m_on_goal_enqueued_callback = callback;
    }

    //! Set callback for goal cancel request
    //! @param callback the callback function, return 0 if accepted, otherwise rejected
    void set_on_goal_cancel_request_callback(std::function<int(std::shared_ptr<GoalHandle_t>)> callback)
    {
        m_on_goal_cancel_request_callback = callback;
    }

    //! Set callback for goal accepted
    void set_on_goal_accepted_callback(std::function<void(std::shared_ptr<GoalHandle_t>)> callback)
    {
        m_on_goal_accepted_callback = callback;
    }

  private:
    // cache ping response
    std::shared_ptr<ActionResult_t>
        m_ping_response;
};

} // namespace redoxi_works
