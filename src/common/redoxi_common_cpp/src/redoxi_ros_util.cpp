#include <redoxi_common_cpp/redoxi_ros_util.hpp>
#include <nlohmann/json.hpp>
#include <chrono>

namespace redoxi_works
{

//! Convert milliseconds to ros2 time message
builtin_interfaces::msg::Time ros2_time_msg_from_ms(double ms)
{
    std::chrono::nanoseconds ns(static_cast<int64_t>(ms * 1e6));
    std::chrono::seconds sec = std::chrono::duration_cast<std::chrono::seconds>(ns);
    std::chrono::nanoseconds nsec = ns - sec;
    builtin_interfaces::msg::Time time_msg;
    time_msg.sec = sec.count();
    time_msg.nanosec = nsec.count();
    return time_msg;
}

//! Convert seconds to ros2 time message
builtin_interfaces::msg::Time ros2_time_msg_from_sec(double sec)
{
    builtin_interfaces::msg::Time time_msg;
    std::chrono::nanoseconds ns(static_cast<int64_t>(sec * 1e9));
    std::chrono::seconds s = std::chrono::duration_cast<std::chrono::seconds>(ns);
    std::chrono::nanoseconds nsec = ns - s;
    time_msg.sec = s.count();
    time_msg.nanosec = nsec.count();
    return time_msg;
}

int declare_default_parameters_for_node(rclcpp::Node *node)
{
    //! Declare and get the param_as_json_string parameter
    node->declare_parameter(RosParams::ParamAsJsonString::MainKey, "");
    std::string param_as_json_string = node->get_parameter(RosParams::ParamAsJsonString::MainKey).as_string();

    RDX_INFO_DEV(node, __func__, false, "param_as_json_string: {}", param_as_json_string);

    //! Parse the JSON string
    nlohmann::json json_params;
    try {
        if (!param_as_json_string.empty()) {
            RDX_INFO_DEV(node, __func__, false, "json parameter string is not empty, parsing: {}", param_as_json_string);
            json_params = nlohmann::json::parse(param_as_json_string);
        }
    } catch (const nlohmann::json::parse_error &e) {
        RDX_INFO_DEV(node, __func__, false, "[{}] Failed to parse param_as_json_string: {}", node->get_name(), e.what());
        return -1;
    }

    if (json_params.empty()) {
        RDX_INFO_DEV(node, __func__, false, "[{}] No json parameters to parse, exiting", node->get_name());
        return 0;
    }

    // get the declare_params
    {
        auto pkey = RosParams::ParamAsJsonString::DeclareParams;
        if (json_params.contains(pkey) && json_params[pkey].is_object()) {
            for (const auto &[param_name, param_value] : json_params[pkey].items()) {
                if (param_value.is_string()) {
                    node->declare_parameter(param_name, param_value.get<std::string>());
                } else if (param_value.is_number_integer()) {
                    node->declare_parameter(param_name, param_value.get<int64_t>());
                } else if (param_value.is_number_float()) {
                    node->declare_parameter(param_name, param_value.get<double>());
                } else if (param_value.is_boolean()) {
                    node->declare_parameter(param_name, param_value.get<bool>());
                } else {
                    RCLCPP_WARN(node->get_logger(), "Unsupported parameter type for %s", param_name.c_str());
                }
            }
        } else {
            RCLCPP_WARN(node->get_logger(), "No parameters to declare or invalid format in %s", pkey.c_str());
        }
    }

    return 0;
}
} // namespace redoxi_works