#pragma once

#include <redoxi_common_cpp/common_concepts.hpp>
#include <redoxi_common_cpp/ros_utils/message_conversion.hpp>
#include <redoxi_public_msgs/msg/control.hpp>

namespace redoxi_works
{
//! Concept to check if a type is a Redoxi action, which is used by most of this library
template <typename T>
concept RedoxiActionConcept = requires(T t)
{
    // must be a ROS action
    requires RosActionConcept<T>;

    // the goal must have a control signal message
    {
        std::declval<typename T::Goal>().x_control
        } -> std::convertible_to<redoxi_public_msgs::msg::Control>;

    // the goal must have a UUID
    {
        std::declval<typename T::Goal>().x_uid
        } -> std::convertible_to<unique_identifier_msgs::msg::UUID>;
};

//! Default implementation of ActionDataTraitConcept
//! For use with actions that has x_uid, x_control, x_return, x_feedback
template <RedoxiActionConcept ActionType>
struct RedoxiActionDataTrait {
    using ActionType_t = ActionType;
    using Goal_t = typename ActionType_t::Goal;
    using Result_t = typename ActionType_t::Result;
    using Feedback_t = typename ActionType_t::Feedback;
    using ControlSignalMessage_t = redoxi_public_msgs::msg::Control;

    RedoxiActionDataTrait()
    {
        static_assert(ActionDataTraitConcept<RedoxiActionDataTrait>, "RedoxiActionDataTrait must satisfy ActionDataTraitConcept");
    }

    /**
     * @brief Get the control signal code from the goal.
     *
     * @param goal The goal from which to extract the control signal code.
     * @return ControlSignalCode The control signal code extracted from the goal.
     */
    static ControlSignalCode get_control_signal_code(const Goal_t &goal)
    {
        return ControlSignalCode(goal.x_control.code);
    }

    /**
     * @brief Mark the goal with a specific control signal code.
     *
     * @param goal The goal to be marked.
     * @param code The control signal code to mark the goal with.
     */
    static void mark_with_control_signal(Goal_t &goal, ControlSignalCode code)
    {
        goal.x_control.code = int64_t(code);
    }

    /**
     * @brief Get the UUID from the goal.
     *
     * @param goal The goal from which to extract the UUID.
     * @return boost::uuids::uuid The UUID extracted from the goal.
     */
    static boost::uuids::uuid get_uuid(const Goal_t &goal)
    {
        return to_boost_uuid(goal.x_uid);
    }

    /**
     * @brief Set the UUID for the goal.
     *
     * @param goal The goal for which the UUID is to be set.
     * @param uuid The UUID to set for the goal.
     */
    static void set_uuid(Goal_t &goal, const boost::uuids::uuid &uuid)
    {
        goal.x_uid = to_ros_uuid_msg(uuid);
    }

    /**
     * @brief Set the return code for the result.
     *
     * @param result The result for which the return code is to be set.
     * @param code The return code to set for the result.
     */
    static void set_return_code(Result_t &result, int64_t code)
    {
        result.x_return.code = code;
    }

    /**
     * @brief Get the return code from the result.
     *
     * @param result The result from which to extract the return code.
     * @return int64_t The return code extracted from the result.
     */
    static int64_t get_return_code(const Result_t &result)
    {
        return result.x_return.code;
    }

    /**
     * @brief Set the return message for the result.
     *
     * @param result The result for which the return message is to be set.
     * @param message The return message to set for the result.
     */
    static void set_return_message(Result_t &result, const std::string &message)
    {
        result.x_return.message = message;
    }

    /**
     * @brief Get the return message from the result.
     *
     * @param result The result from which to extract the return message.
     * @return std::string The return message extracted from the result.
     */
    static std::string get_return_message(const Result_t &result)
    {
        return result.x_return.message;
    }

    /**
     * @brief Set the feedback code for the feedback.
     *
     * @param feedback The feedback for which the feedback code is to be set.
     * @param code The feedback code to set for the feedback.
     */
    static void set_feedback_code(Feedback_t &feedback, int64_t code)
    {
        feedback.x_feedback.code = code;
    }

    /**
     * @brief Get the feedback code from the feedback.
     *
     * @param feedback The feedback from which to extract the feedback code.
     * @return int64_t The feedback code extracted from the feedback.
     */
    static int64_t get_feedback_code(const Feedback_t &feedback)
    {
        return feedback.x_feedback.code;
    }

    /**
     * @brief Set the feedback message for the feedback.
     *
     * @param feedback The feedback for which the feedback message is to be set.
     * @param message The feedback message to set for the feedback.
     */
    static void set_feedback_message(Feedback_t &feedback, const std::string &message)
    {
        feedback.x_feedback.message = message;
    }

    /**
     * @brief Get the feedback message from the feedback.
     *
     * @param feedback The feedback from which to extract the feedback message.
     * @return std::string The feedback message extracted from the feedback.
     */
    static std::string get_feedback_message(const Feedback_t &feedback)
    {
        return feedback.x_feedback.message;
    }

    /**
     * @brief Get the source task id from the goal.
     * @details Source task is what initiated this action, a single source task may initiate multiple actions
     *          and the source task uid is used to relate all these actions together.
     *          Source task id is what you get from ActionDataTrait::get_uuid(source_action.goal), x_uid for redoxi actions
     * @param goal The goal from which to extract the source task id
     * @return The source task id as a boost UUID
     */
    static RosActionTaskMetadata get_source_task_metadata(const Goal_t &goal)
    {
        return RosActionTaskMetadata{
            .source_task_id = to_boost_uuid(goal.x_task_metadata.source_task_uid),
            .source_task_info = goal.x_task_metadata.source_task_info};
    }

    /**
     * @brief Set the source task metadata in the goal.
     * @param goal The goal in which to set the source task metadata
     * @param metadata The source task metadata to set
     */
    static void set_source_task_metadata(Goal_t &goal, const RosActionTaskMetadata &metadata)
    {
        goal.x_task_metadata.source_task_info = metadata.source_task_info;
        goal.x_task_metadata.source_task_uid = to_ros_uuid_msg(metadata.source_task_id);
    }
};


} // namespace redoxi_works