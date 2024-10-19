#pragma once
#include <optional>
#include <future>
#include <chrono>
#include <mutex>
#include <condition_variable>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

// just for example
#include <redoxi_common_cpp/ros_utils/common.hpp>
#include <redoxi_public_msgs/action/process_frame.hpp>
#include <redoxi_public_msgs/msg/frame.hpp>

namespace redoxi_works
{
// synchronous action sender, which will block and wait for action response

// these are used during development, because clangd cannot infer template types
// using ActionType = redoxi_public_msgs::action::ProcessFrame;
// using DurationType = std::chrono::milliseconds;

/**
 * @brief A synchronous action sender that blocks and waits for action responses.
 *
 * This class provides a way to send action goals and wait for their responses
 * synchronously. It is particularly useful when you need to ensure that an action
 * has been accepted or rejected before proceeding with further operations.
 *
 * @tparam ActionType The ROS 2 action type (e.g., MyCustomAction)
 * @tparam DurationType The duration type for timeouts (default: std::chrono::milliseconds)
 *
 * @example
 * // Create a SyncActionSender for a custom action
 * auto node = std::make_shared<rclcpp::Node>("my_node");
 * SyncActionSender<MyCustomAction> sender();
 *
 * // Create and send a goal
 * auto goal = MyCustomAction::Goal();
 * goal.some_field = 42;
 *
 * auto client = rclcpp_action::create_client<MyCustomAction>(node, "my_action");
 * auto result = sender.send(goal, *client, std::chrono::seconds(5));
 *
 * // Check the result
 * if (result.response_code) {
 *     if (result.response_code.value() == ActionDownstreamResponse::ACCEPTED) {
 *         auto goal_handle = result.goal_handle_future.get();
 *         // Process the accepted goal
 *     } else if (result.response_code.value() == ActionDownstreamResponse::REJECTED) {
 *         // Handle rejection
 *     } else if (result.response_code.value() == ActionDownstreamResponse::TIMEOUT) {
 *         // Handle timeout
 *     }
 * } else {
 *     // Handle case where response_code is not set (unknown result)
 * }
 */
template <typename ActionType,
          typename DurationType = DefaultTimeUnit_t,
          typename = std::enable_if_t<hacky::is_duration<DurationType>::value>>
class SyncActionSender
{
  public:
    using ActionType_t = ActionType;
    using ActionClient_t = rclcpp_action::Client<ActionType_t>;
    using GoalHandle_t = rclcpp_action::ClientGoalHandle<ActionType_t>;
    using Goal_t = typename ActionType_t::Goal;

    /**
     * @brief The result of sending a goal to downstream action server
     * @details If response_code is set to ACCEPTED, it means the goal is accepted,
     *          you can use goal_handle_future.get() to get the goal handle without waiting.
     *          If response_code is set to REJECTED, the goal was rejected.
     *          If response_code is set to TIMEOUT, you should use goal_handle_future.wait() to wait for the result.
     *          If response_code is not set, the result is unknown, which occurs when you have never waited for the result.
     */
    struct _SendResult {
        /**
         * @brief The response code from the downstream action server
         * @details ACCEPTED: The goal is accepted, goal_handle_future.get() can be used without waiting
         *          REJECTED: The goal is rejected, goal_handle_future.get() will return nullptr
         *          TIMEOUT: The result is unknown, goal_handle_future.wait() should be used to wait for the result
         *          If not set, the result is unknown, this is the case when you have never waited for the result
         */
        std::optional<ActionDownstreamResponse> response_code;

        /**
         * @brief The goal handle future, obtained from the ROS action
         * @details Can be resolved to get the goal handle
         */
        std::shared_future<typename GoalHandle_t::SharedPtr> goal_handle_future;
    };

    // just for consistency
    using SendResult_t = _SendResult;

  public:
    /**
     * @brief Send an action goal and wait for the response
     *
     * @details This method sends an action goal to the specified client and waits for a response.
     * The waiting behavior is determined by the timeout parameter.
     *
     * @param goal The action goal to send
     * @param client The action client to use for sending the goal
     * @param timeout Timeout duration.
     *                If timeout < 0, wait indefinitely until the goal is received.
     *                If timeout == 0, no wait (returns immediately).
     *                If timeout > 0, wait for that duration before considering the request timed out.
     *
     * @return SendResult_t A struct containing:
     *         - response_code: An optional ActionDownstreamResponse indicating the result (ACCEPTED, REJECTED, TIMEOUT, or not set)
     *         - goal_handle_future: A shared future that can be used to retrieve the goal handle
     *
     * @note If timeout < 0, the response_code in the result will not be set, and the user should use
     *       goal_handle_future.wait() to wait for and process the result.
     */
    SendResult_t send(
        const Goal_t &goal,
        ActionClient_t &client,
        DurationType timeout = DurationType(-1)) const
    {
        SendResult_t result;

        if (timeout == DurationType::zero()) {
            //! No waiting, send goal without callback and return immediately
            auto goal_handle_future = client.async_send_goal(goal);
            result.goal_handle_future = goal_handle_future;
            return result;
        }

        //! Initialize synchronization primitives and response flag
        std::mutex mutex;
        std::condition_variable cv;
        bool goal_response_received = false;

        //! Configure goal send options with a callback
        typename rclcpp_action::Client<ActionType_t>::SendGoalOptions opt;
        opt.goal_response_callback =
            [&](const auto &goal_handle) {
                std::lock_guard<std::mutex> lock(mutex);
                if (goal_handle) {
                    result.response_code = ActionDownstreamResponse::ACCEPTED;
                } else {
                    result.response_code = ActionDownstreamResponse::REJECTED;
                }
                goal_response_received = true;
                cv.notify_one(); //! Notify waiting thread
            };

        //! Send goal asynchronously and store the future
        auto goal_handle_future = client.async_send_goal(goal, opt);
        result.goal_handle_future = goal_handle_future;

        //! Handle waiting behavior
        std::unique_lock<std::mutex> lock(mutex);
        if (timeout > DurationType::zero()) {
            //! Wait for the specified duration or until goal response is received
            if (!cv.wait_for(lock, timeout, [&] { return goal_response_received; })) {
                //! Timeout occurred
                result.response_code = ActionDownstreamResponse::TIMEOUT;
            }
            //! No action needed if goal response received within timeout, as result is set in callback
        } else {
            //! Negative timeout specified, wait indefinitely until goal response is received
            cv.wait(lock, [&] { return goal_response_received; });
        }
        //! Result is set in callback for non-timeout cases

        return result;
    }
};

} // namespace redoxi_works
