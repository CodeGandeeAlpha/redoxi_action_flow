#pragma once

#include <redoxi_common_cpp/redoxi_concepts.hpp>
#include <redoxi_common_nodes/async_action_port/input_port_concepts.hpp>
#include <redoxi_public_msgs/action/process_frame.hpp>
#include <json_struct/json_struct.h>

namespace redoxi_works
{

namespace input_port_types
{

// using _SampleAction = redoxi_public_msgs::action::ProcessFrame;

//! A data received by action server (the port)
template <RosActionConcept TAction>
struct DefaultReceiveSourceData {
    using ActionType_t = TAction;
    using Goal_t = typename ActionType_t::Goal;
    using GoalHandle_t = rclcpp_action::ServerGoalHandle<ActionType_t>;
    using GoalHandlePromise_t = std::promise<std::shared_ptr<GoalHandle_t>>;

    //! Get goal uuid
    virtual rclcpp_action::GoalUUID get_goal_uuid() const
    {
        return m_goal_uuid;
    }

    //! Get goal handle future
    virtual std::shared_future<std::shared_ptr<GoalHandle_t>> get_goal_handle_future() const
    {
        return m_goal_handle_future;
    }

    //! Get goal handle promise
    virtual std::shared_ptr<GoalHandlePromise_t> get_goal_handle_promise()
    {
        return m_goal_handle_promise;
    }

    //! Get goal
    virtual const Goal_t *get_goal() const
    {
        return m_goal.get();
    }

    //! Set goal, using shared_ptr because the goal is owned by the ROS action server
    virtual void set_goal(std::shared_ptr<const Goal_t> goal)
    {
        m_goal = goal;
    }

    //! Set goal uuid
    virtual void set_goal_uuid(rclcpp_action::GoalUUID goal_uuid)
    {
        m_goal_uuid = goal_uuid;
    }

    //! Set goal handle promise
    virtual void set_goal_handle_promise(std::shared_ptr<GoalHandlePromise_t> goal_handle_promise)
    {
        m_goal_handle_promise = goal_handle_promise;

        // create the future
        m_goal_handle_future = m_goal_handle_promise->get_future();
    }

  public:
    rclcpp_action::GoalUUID m_goal_uuid;
    std::shared_ptr<const Goal_t> m_goal;
    std::shared_ptr<GoalHandlePromise_t> m_goal_handle_promise;
    std::shared_future<std::shared_ptr<GoalHandle_t>> m_goal_handle_future;
};
static_assert(ReceiveSourceDataConcept<DefaultReceiveSourceData<_SampleAction>>,
              "DefaultReceiveSourceData does not satisfy ReceiveSourceDataConcept");

//! The init config for the action input port
template <TimeDurationConcept TTimeUnit>
struct DefaultInitConfig {
    using TimeUnit_t = TTimeUnit;
    inline static constexpr TimeUnit_t DefaultGoalResultExpireTime{std::chrono::milliseconds(1000)};
    virtual ~DefaultInitConfig() = default;

    //! Get number of buffer requests
    //! If the value is not positive, it means the queue is unbounded
    virtual int64_t get_buffer_capacity() const
    {
        return buffer_capacity;
    }

    //! Get the name of the action
    virtual const std::string &get_action_name() const
    {
        return action_name;
    }

    //! Set the buffer capacity
    virtual void set_buffer_capacity(int64_t capacity)
    {
        buffer_capacity = capacity;
    }

    //! Set the action name
    virtual void set_action_name(const std::string &name)
    {
        action_name = name;
    }

    //! Get the goal result expire time
    virtual TimeUnit_t get_goal_result_expire_time() const
    {
        return goal_result_expire_time;
    }

    //! Set the goal result expire time
    virtual void set_goal_result_expire_time(TimeUnit_t expire_time)
    {
        goal_result_expire_time = expire_time;
    }

  protected:
    int64_t buffer_capacity = -1;
    std::string action_name;
    TimeUnit_t goal_result_expire_time = DefaultGoalResultExpireTime;

  public:
    JS_OBJECT(JS_MEMBER(buffer_capacity),
              JS_MEMBER(action_name),
              JS_MEMBER(goal_result_expire_time));
};
static_assert(InitConfigConcept<DefaultInitConfig<_SampleTimeUnit>>);

//! for quick construction of a specification
template <RosActionConcept TAction,
          ActionDataTraitConcept TActionDataTrait,
          TimeDurationConcept TTimeUnit>
struct DefaultAsyncActionInputPortSpec {
    using ActionType_t = TAction;
    using ActionGoal_t = typename ActionType_t::Goal;
    using ActionDataTrait_t = TActionDataTrait;
    using TimeUnit_t = TTimeUnit;

    using ReceiveSourceData_t = DefaultReceiveSourceData<ActionType_t>;
    using InitConfig_t = DefaultInitConfig<TTimeUnit>;
};
static_assert(AsyncActionInputPortSpecConcept<
              DefaultAsyncActionInputPortSpec<_SampleAction,
                                              RedoxiActionDataTrait<_SampleAction>,
                                              _SampleTimeUnit>>);

} // namespace input_port_types

} // namespace redoxi_works
