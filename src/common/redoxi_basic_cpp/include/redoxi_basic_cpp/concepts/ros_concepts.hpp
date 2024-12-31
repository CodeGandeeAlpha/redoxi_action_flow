#pragma once

// basic concepts
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <redoxi_basic_cpp/concepts/basic_concepts.hpp>

namespace redoxi_works
{

//! When this action is triggered by another action (source task),
//! this metadata info is used to track the source task
struct RosActionTaskMetadata {
    //! UUID of the source task
    UUIDType source_task_id{0};

    //! Info string about the source task
    std::string source_task_info;
};

//! Concept to check if a type is ROS message
template <typename T>
concept RosMessageConcept = requires(T t)
{
    requires std::copyable<T>;
    requires std::is_copy_assignable_v<T>;
    requires std::is_default_constructible_v<T>;
};

//! Concept to check if a type is ROS action definition
template <typename T>
concept RosActionConcept = requires
{
    // copyable and default constructible
    requires std::copyable<T>;
    requires std::is_default_constructible_v<T>;

    //! Check Goal type exists and is accessible
    typename T::Goal;
    requires RosMessageConcept<typename T::Goal>;

    //! Check Result type exists and is accessible
    typename T::Result;
    requires RosMessageConcept<typename T::Result>;

    //! Check Feedback type exists and is accessible
    typename T::Feedback;
    requires RosMessageConcept<typename T::Feedback>;

    //! Check Impl struct exists and has required service/message types
    typename T::Impl::SendGoalService;
    typename T::Impl::GetResultService;
    typename T::Impl::FeedbackMessage;
    typename T::Impl::CancelGoalService;
    typename T::Impl::GoalStatusMessage;
};

//! Publisher concept
template <typename T>
concept RosPublisherConcept = requires(T pub)
{
    //! Publish a message, without annotation text
    typename T::MessageType_t;

    //! Inner publisher type
    typename T::Publisher_t;
    requires std::same_as<typename T::Publisher_t, rclcpp::Publisher<typename T::MessageType_t>>;

    requires RosMessageConcept<typename T::MessageType_t>;
    {
        pub.publish(std::declval<const typename T::MessageType_t &>())
        } -> std::same_as<int>;

    //! Publish a message with annotation text
    {
        pub.publish(std::declval<const typename T::MessageType_t &>(),
                    std::declval<const std::string &>())
        } -> std::same_as<int>;

    //! Get the inner publisher
    {
        std::declval<const T &>().get_publisher()
        } -> std::same_as<std::shared_ptr<typename T::Publisher_t>>;

    //! init with inner publisher
    {
        pub.init(std::declval<std::shared_ptr<typename T::Publisher_t>>())
        } -> std::same_as<void>;
};

//! Concept to check if a type is a ROS node, which is either the normal Node or the LifecycleNode
template <typename T>
concept RosNodeConcept = requires(T node)
{
    requires std::is_base_of_v<rclcpp::Node, T> || std::is_base_of_v<rclcpp_lifecycle::LifecycleNode, T>;
};

//! None publisher type, used as dummy publisher for when no publisher is available
template <RosMessageConcept MessageType>
class NoneRosPublisher
{
  public:
    using MessageType_t = MessageType;
    using Publisher_t = rclcpp::Publisher<MessageType>;
    virtual ~NoneRosPublisher() = default;

    virtual std::shared_ptr<Publisher_t> get_publisher() const
    {
        return nullptr;
    }

    virtual void init(std::shared_ptr<Publisher_t> pub)
    {
        (void)pub;
    }

    virtual int publish(const MessageType_t &) const
    {
        return 0;
    }

    virtual int publish(const MessageType_t &, const std::string &) const
    {
        return 0;
    }
};
static_assert(RosPublisherConcept<NoneRosPublisher<int>>,
              "NoneRosPublisher must satisfy RosPublisherConcept");

//! Simple ROS publisher, just a wrapper around the ros publisher
template <RosMessageConcept MessageType>
class SimpleRosPublisher
{
  public:
    using MessageType_t = MessageType;
    using Publisher_t = rclcpp::Publisher<MessageType>;
    virtual ~SimpleRosPublisher() = default;

    virtual void init(std::shared_ptr<Publisher_t> pub)
    {
        m_publisher = pub;
    }

    virtual int publish(const MessageType_t &msg) const
    {
        if (m_publisher) {
            m_publisher->publish(msg);
        }
        return 0;
    }

    virtual int publish(const MessageType_t &msg, const std::string &) const
    {
        if (m_publisher) {
            m_publisher->publish(msg);
        }
        return 0;
    }

    virtual std::shared_ptr<Publisher_t> get_publisher() const
    {
        return m_publisher;
    }

  protected:
    std::shared_ptr<Publisher_t> m_publisher;
};
static_assert(RosPublisherConcept<SimpleRosPublisher<int>>,
              "SimpleRosPublisher must satisfy RosPublisherConcept");

template <typename T>
concept ActionDataTraitConcept = requires(T t)
{
    typename T::ActionType_t;
    requires RosActionConcept<typename T::ActionType_t>;

    typename T::Goal_t;
    requires std::same_as<typename T::Goal_t, typename T::ActionType_t::Goal>;

    typename T::Result_t;
    requires std::same_as<typename T::Result_t, typename T::ActionType_t::Result>;

    typename T::Feedback_t;
    requires std::same_as<typename T::Feedback_t, typename T::ActionType_t::Feedback>;

    // get control signal code from goal
    {
        T::get_control_signal_code(std::declval<const typename T::Goal_t &>())
        } -> std::same_as<ControlSignalCode>;

    // mark a goal with a control signal
    {
        T::mark_with_control_signal(std::declval<typename T::Goal_t &>(), std::declval<ControlSignalCode>())
        } -> std::same_as<void>;

    // get uuid from goal
    {
        T::get_uuid(std::declval<const typename T::Goal_t &>())
        } -> std::same_as<boost::uuids::uuid>;

    // write uuid to goal
    {
        T::set_uuid(std::declval<typename T::Goal_t &>(), std::declval<boost::uuids::uuid>())
        } -> std::same_as<void>;

    /**
     * @brief Get the source task metadata from the goal
     * @details A source task is an action that initiates other actions. When an action (the source task)
     *          spawns multiple other actions, the source task's UUID is stored in each of those actions
     *          to track their relationship back to the originating task. For Redoxi actions, this ID
     *          comes from ActionDataTrait::get_uuid(source_action.goal) which is stored in x_uid.
     */
    {
        T::get_source_task_metadata(std::declval<const typename T::Goal_t &>())
        } -> std::same_as<RosActionTaskMetadata>;

    {
        T::set_source_task_metadata(std::declval<typename T::Goal_t &>(), std::declval<const RosActionTaskMetadata &>())
        } -> std::same_as<void>;
};

template <RosActionConcept ActionType>
struct NoneActionDataTrait {
    using ActionType_t = ActionType;
    using Goal_t = typename ActionType_t::Goal;
    using Result_t = typename ActionType_t::Result;
    using Feedback_t = typename ActionType_t::Feedback;

    static ControlSignalCode get_control_signal_code(const Goal_t &)
    {
        return ControlSignalCode::Unknown;
    }

    static void mark_with_control_signal(Goal_t &, ControlSignalCode)
    {
    }

    static boost::uuids::uuid get_uuid(const Goal_t &)
    {
        return boost::uuids::uuid();
    }

    static void set_uuid(Goal_t &, const boost::uuids::uuid &)
    {
    }

    // source task metadata
    static RosActionTaskMetadata get_source_task_metadata(const Goal_t &)
    {
        return RosActionTaskMetadata();
    }

    static void set_source_task_metadata(Goal_t &, const RosActionTaskMetadata &)
    {
    }
};

//! Interface for retry policy, if anything needs to retry, its configuration should be here
//! Concept for retry policy interface
template <typename T>
concept RetryPolicyConcept = requires(T t,
                                      std::optional<int64_t> retry_count,
                                      std::optional<typename T::DurationType_t> wait_time,
                                      bool use_fallback_if_not_set)
{
    typename T::DurationType_t;
    requires TimeDurationConcept<typename T::DurationType_t>;

    //! Must be default constructible
    requires std::is_default_constructible_v<T>;

    //! Must be copyable
    requires std::copyable<T>;

    //! Must have methods to get/set number of retries
    {
        std::declval<const T &>().get_number_of_retry(use_fallback_if_not_set)
        } -> std::same_as<std::optional<int64_t>>;
    {
        t.set_number_of_retry(retry_count)
        } -> std::same_as<void>;
    {
        std::declval<const T &>().get_fallback_number_of_retry()
        } -> std::same_as<int64_t>;

    //! Must have methods to get/set wait time between retries
    //! negative wait time means wait indefinitely, 0 means no wait
    {
        std::declval<const T &>().get_wait_time_between_retry(use_fallback_if_not_set)
        } -> std::same_as<std::optional<typename T::DurationType_t>>;
    {
        t.set_wait_time_between_retry(wait_time)
        } -> std::same_as<void>;
    {
        std::declval<const T &>().get_fallback_wait_time_between_retry()
        } -> std::same_as<typename T::DurationType_t>;

    //! Must have methods to get/set wait time for retry response
    //! @note This is the wait time for the downstream action to respond to the goal
    //! negative wait time means wait indefinitely, 0 means no wait
    {
        std::declval<const T &>().get_wait_time_retry_response(use_fallback_if_not_set)
        } -> std::same_as<std::optional<typename T::DurationType_t>>;
    {
        t.set_wait_time_retry_response(wait_time)
        } -> std::same_as<void>;
    {
        std::declval<const T &>().get_fallback_wait_time_retry_response()
        } -> std::same_as<typename T::DurationType_t>;
};

} // namespace redoxi_works