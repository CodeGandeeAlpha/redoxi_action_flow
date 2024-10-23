#include <redoxi_video_reader/base/VideoReaderBaseTypes.hpp>
#include <redoxi_video_reader/base/VideoReaderBase.hpp>
#include <nlohmann/json.hpp>
#include <map>
#include <regex>
#include <fmt/format.h>
#include <spdlog/spdlog.h>
// ROS parameter json format
/*
{
    "declare_params": {
        "custom_var_1": 100.0,
        "custom_var_2": 10.0,
    },
    "runtime_config": {
        "frame_interval_ms": 10000.0,
        "step_interval_ms": 1000,
        "publish_to_debug_topic": True,
    },
    "init_config": {
        "downstreams": {
            "actions": [
                    {
                        "name": "/video_sink/in/action",
                        "retry_strategy": {
                            "max_retries": 3,
                            "retry_interval_ms": 50.0,
                        }
                    }
                ]
        },
    },
}

*/


namespace redoxi_works
{

namespace RedoxiVideoReaderBaseTypes
{
void InitConfig::from_parameters(RedoxiVideoReaderBase *node)
{
    //! Try to get params from json string
    nlohmann::json json_params = RDX_GET_JSON_PARAM_FROM_NODE(node);

    //! Nothing to parse, return
    if (json_params.empty()) {
        RDX_LOG_INFO(node, __func__, "No JSON parameters found");
        return;
    }

    //! Parse the init_config and downstreams configuration
    using json_pointer = nlohmann::json::json_pointer;
    json_pointer init_config_ptr("/init_config");
    json_pointer downstreams_ptr("/init_config/downstreams/actions");

    if (json_params.contains(init_config_ptr) && json_params.contains(downstreams_ptr) && json_params[downstreams_ptr].is_array()) {
        for (const auto &action_config : json_params[downstreams_ptr]) {
            if (!action_config.contains("name")) {
                RDX_LOG_WARN(node, __func__, "Skipping downstream action without a name");
                continue;
            }

            std::string action_name = action_config["name"].get<std::string>();
            auto spec = std::make_shared<DownstreamSpec>();
            spec->accept_frame_action = action_name;
            RDX_LOG_INFO(node, __func__, "Configuring downstream action: {}", action_name);

            //! Configure retry strategy if present
            if (action_config.contains("retry_strategy")) {
                const auto &retry_strategy = action_config["retry_strategy"];

                if (retry_strategy.contains("max_retries")) {
                    int max_retries = retry_strategy["max_retries"].get<int>();
                    spec->retry_strategy->set_max_number_of_retries(max_retries);
                    RDX_LOG_INFO(node, __func__, "Set max retries for {}: {}", action_name, max_retries);
                }

                if (retry_strategy.contains("retry_interval_ms")) {
                    int64_t retry_interval = retry_strategy["retry_interval_ms"].get<int64_t>();
                    spec->retry_strategy->set_wait_time_for_retry(std::chrono::milliseconds(retry_interval));
                    RDX_LOG_INFO(node, __func__, "Set retry interval for {}: {} ms", action_name, retry_interval);
                }
            }

            //! Add the downstream spec to the configuration using action name as key
            this->downstreams[spec->accept_frame_action] = spec;
            RDX_LOG_INFO(node, __func__, "Added downstream spec for action: {}", action_name);
        }
    } else {
        RDX_LOG_INFO(node, __func__, "No valid downstream configuration found in JSON parameters");
    }
}

void RuntimeConfig::from_parameters(RedoxiVideoReaderBase *node)
{
    using JsonPointer_t = nlohmann::json::json_pointer;
    auto json_params = RDX_GET_JSON_PARAM_FROM_NODE(node);

    if (json_params.empty())
        return;

    std::string KeyRuntimeConfig = "runtime_config";
    std::string KeyFrameIntervalMs = "frame_interval_ms";
    std::string KeyStepIntervalMs = "step_interval_ms";
    std::string KeyPublishToDebugTopic = "publish_to_debug_topic";
    {
        auto jkey = JsonPointer_t(fmt::format("/{}/{}", KeyRuntimeConfig, KeyFrameIntervalMs));
        if (json_params.contains(jkey)) {
            this->frame_interval_ms = json_params[jkey].get<double>();
            RDX_LOG_INFO(node, __func__, "Got frame_interval_ms from JSON: {}", this->frame_interval_ms);
        }
    }

    {
        auto jkey = JsonPointer_t(fmt::format("/{}/{}", KeyRuntimeConfig, KeyStepIntervalMs));
        if (json_params.contains(jkey)) {
            this->step_interval_ms = json_params[jkey].get<double>();
            RDX_LOG_INFO(node, __func__, "Got step_interval_ms from JSON: {}", this->step_interval_ms);
        }
    }

    {
        auto jkey = JsonPointer_t(fmt::format("/{}/{}", KeyRuntimeConfig, KeyPublishToDebugTopic));
        if (json_params.contains(jkey)) {
            this->publish_to_debug_topic = json_params[jkey].get<bool>();
            RDX_LOG_INFO(node, __func__, "Got publish_to_debug_topic from JSON: {}", this->publish_to_debug_topic);
        }
    }
}

} // namespace RedoxiVideoReaderBaseTypes

} // namespace redoxi_works
