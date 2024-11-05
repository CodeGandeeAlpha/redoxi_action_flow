#pragma once

#include <redoxi_common_cpp/redoxi_concepts.hpp>
#include <redoxi_common_nodes/async_action_port/input_port_concepts.hpp>
#include <redoxi_public_msgs/action/process_frame.hpp>

namespace redoxi_works
{

namespace input_port_types
{

using _SampleAction = redoxi_public_msgs::action::ProcessFrame;

//! A data received by action server (the port)
struct DefaultReceiveSourceData {
    using ActionType_t = _SampleAction;
    using Goal_t = typename ActionType_t::Goal;
    using GoalHandle_t = rclcpp_action::ServerGoalHandle<ActionType_t>;
    using GoalHandlePromise_t = std::promise<std::shared_ptr<GoalHandle_t>>;

    //! Get goal uuid
    rclcpp_action::GoalUUID get_goal_uuid() const
    {
        return m_goal_uuid;
    }

    //! Get goal handle future
    std::shared_future<std::shared_ptr<GoalHandle_t>> get_goal_handle_future() const
    {
        return m_goal_handle_future;
    }

    //! Get goal handle promise
    std::shared_ptr<GoalHandlePromise_t> get_goal_handle_promise()
    {
        return m_goal_handle_promise;
    }

    //! Get goal
    const Goal_t *get_goal() const
    {
        return m_goal.get();
    }

    //! Set goal, using shared_ptr because the goal is owned by the ROS action server
    void set_goal(std::shared_ptr<const Goal_t> goal)
    {
        m_goal = goal;
    }

    //! Set goal uuid
    void set_goal_uuid(rclcpp_action::GoalUUID goal_uuid)
    {
        m_goal_uuid = goal_uuid;
    }

    //! Set goal handle promise
    void set_goal_handle_promise(std::shared_ptr<GoalHandlePromise_t> goal_handle_promise)
    {
        m_goal_handle_promise = goal_handle_promise;

        // create the future
        m_goal_handle_future = m_goal_handle_promise->get_future();
    }

  public:
    rclcpp_action::GoalUUID m_goal_uuid;
    std::shared_ptr<const Goal_t> m_goal;
    std::shared_future<std::shared_ptr<GoalHandle_t>> m_goal_handle_future;
    std::shared_ptr<GoalHandlePromise_t> m_goal_handle_promise;
};
static_assert(ReceiveSourceDataConcept<DefaultReceiveSourceData>,
              "DefaultReceiveSourceData does not satisfy ReceiveSourceDataConcept");

//! The init config for the action input port
struct DefaultInitConfig {
    //! Get number of buffer requests
    //! If the value is not positive, it means the queue is unbounded
    int64_t get_num_buffer_requests() const
    {
        return num_buffer_requests;
    }

    //! Get the name of the action
    std::string get_action_name() const
    {
        return action_name;
    }

  protected:
    int64_t num_buffer_requests = -1;
    std::string action_name;
};

//! The specification for the action input port
struct AsyncActionInputPortSpec {
    using ActionType_t = _SampleAction;
    using ActionGoal_t = typename ActionType_t::Goal;
    using ActionDataTrait_t = RedoxiActionDataTrait<ActionType_t>;

    using ReceiveSourceData_t = DefaultReceiveSourceData;
    using InitConfig_t = DefaultInitConfig;
};
} // namespace input_port_types

} // namespace redoxi_works
