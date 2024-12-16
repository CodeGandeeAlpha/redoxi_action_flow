#pragma once

#include <thread>
#include <atomic>

#include <rclcpp/rclcpp.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <redoxi_ros_logging/redoxi_ros_logging.hpp>
#include <redoxi_common_cpp/redoxi_common_cpp.hpp>

namespace redoxi_works
{

namespace DefaultParams
{

//! goal handle timeout
constexpr DefaultTimeUnit_t GoalHandleTimeout = std::chrono::milliseconds(3000);

//! publisher queue size when the topic is intended to be used for debugging
constexpr int DebugPublisherQueueSize = 10;
constexpr int DataPublisherQueueSize = 50;

//! QoS for the debug publisher
const rclcpp::QoS DebugPublisherQoS = rclcpp::SensorDataQoS();

//! QoS for the data publisher, reliable and with larger history depth
const rclcpp::QoS DataPublisherQoS = rclcpp::QoS(DataPublisherQueueSize).reliable();

// used for performance probing
const rclcpp::QoS ProbePublisherQoS = rclcpp::RosoutQoS();

} // namespace DefaultParams

//! A dummy token that can be used as a placeholder for time token
struct DummyTimeToken {
};

//! Macro to get JSON parameter from node, defined as macro to avoid include nlohmann/json.hpp in this file
//! you must include nlohmann/json.hpp in the file where you use this macro
#define RDX_GET_JSON_PARAM_FROM_NODE(node)                                      \
    ([](const rclcpp::Node *node) -> nlohmann::json {                           \
        rclcpp::Parameter _json_params;                                         \
        auto pkey = redoxi_works::RosParams::ParamAsJsonString::MainKey;        \
        if (!node->get_parameter(pkey, _json_params))                           \
            return nlohmann::json();                                            \
        try {                                                                   \
            auto str = _json_params.as_string();                                \
            return str.empty() ? nlohmann::json() : nlohmann::json::parse(str); \
        } catch (const nlohmann::json::parse_error &e) {                        \
            RDX_RAISE_ERROR("Failed to parse json string: {}", e.what());       \
            return nlohmann::json();                                            \
        }                                                                       \
    })(node)


enum class ActionDownstreamResponse {
    ACCEPTED = 0,
    REJECTED = 1,
    TIMEOUT = 2,
};

//! declare some default parameters for the node, some are looked up by json string
//! return 0 if success, otherwise return error code
int declare_default_parameters_for_node(rclcpp::Node *node);

//! Convert milliseconds to ros2 time message
builtin_interfaces::msg::Time ros2_time_msg_from_ms(double ms);

//! Convert seconds to ros2 time message
builtin_interfaces::msg::Time ros2_time_msg_from_sec(double sec);

} // namespace redoxi_works
