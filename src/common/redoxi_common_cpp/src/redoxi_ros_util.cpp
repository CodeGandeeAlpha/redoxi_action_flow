#include <redoxi_common_cpp/redoxi_ros_util.hpp>
#include <nlohmann/json.hpp>

namespace redoxi_works
{

int declare_default_parameters_for_node(rclcpp::Node *node)
{
    //! Declare and get the param_as_json_string parameter
    node->declare_parameter(RosParams::ParamAsJsonString::MainKey, "");
    std::string param_as_json_string = node->get_parameter(RosParams::ParamAsJsonString::MainKey).as_string();

    RDX_INFO_DEV(node, __func__, false, "param_as_json_string: {}", param_as_json_string);

    //! Parse the JSON string
    nlohmann::json json_params;
    try {
        if (!param_as_json_string.empty())
            json_params = nlohmann::json::parse(param_as_json_string);
    } catch (const nlohmann::json::parse_error &e) {
        RCLCPP_ERROR(node->get_logger(), "[%s] Failed to parse param_as_json_string: %s", node->get_name(), e.what());
        return -1;
    }

    if (json_params.empty())
        return 0;

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