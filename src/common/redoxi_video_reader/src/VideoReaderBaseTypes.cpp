#include <redoxi_video_reader/base/VideoReaderBaseTypes.hpp>
#include <redoxi_video_reader/base/VideoReaderBase.hpp>
#include <nlohmann/json.hpp>
#include <map>
#include <fmt/format.h>
#include <spdlog/spdlog.h>
//! ROS parameter json format
/*
{
    //! default time unit for all parameters, unless otherwise specified. time values can be int or double
    //! if timeunit is not specified, it is ms
    "timeunit": "ms",

    //! declare custom parameters here, these will be set into node->declare_parameter()
    "declare_params": {
        "custom_var_1": 100.0,
        "custom_var_2": 10.0,
    },

    //! runtime config
    "runtime_config": {
        "frame_interval_ms": 10000.0,
        "step_interval_ms": 1000,
        "publish_to_debug_topic": true,
        "delivery_policy_fallback": {
            "number_of_enqueue_retry": 5,
            "wait_time_between_enqueue_retry": 10.0,
            "number_of_delivery_retry": 5,
            "wait_time_between_delivery_retry": 20.0,
            "wait_time_for_delivery_response": 100.0
        },
        "delivery_options": {
            "frame_payload_type": "uncompressed", //can be "uncompressed", "uncompressed_by_shared_memory", "jpeg_encoded", "png_encoded"
            "drop_frame_strategy": "no_drop",      //can be "no_drop" or "drop_as_needed"
            "jpeg_quality": 90,                    //only valid when frame_payload_type is "jpeg_encoded"
            "num_buffer_frames": 1  //number of frames to buffer waiting for delivery
        }
    },

    //! init config
    "init_config": {
        "use_debug_pub": true,
        "downstreams": {
            "actions": [
                    {
                        "name": "/video_sink/in/action",
                        "delivery_policy": {
                            "number_of_enqueue_retry": 10,
                            "wait_time_between_enqueue_retry": 5.0,
                            "number_of_delivery_retry": 10,
                            "wait_time_between_delivery_retry": 10.0,
                            "wait_time_for_delivery_response": 50.0
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

    // RDX_INFO_DEV(node, __func__, json_params.dump(4));

    //! Nothing to parse, return
    if (json_params.empty()) {
        RDX_INFO_DEV(node, __func__, "{}", "No JSON parameters found");
        return;
    }

    //! Check if timeunit is specified and if it's "ms"
    if (json_params.contains("timeunit")) {
        std::string timeunit = json_params["timeunit"].get<std::string>();
        if (timeunit != "ms") {
            RDX_RAISE_ERROR("Invalid timeunit specified. Only 'ms' is supported. Exiting.");
            return;
        }
    }

    //! Parse the init_config and downstreams configuration
    using json_pointer = nlohmann::json::json_pointer;
    json_pointer init_config_ptr("/init_config");
    json_pointer downstreams_ptr("/init_config/downstreams/actions");

    if (json_params.contains(init_config_ptr)) {
        const auto &init_config = json_params[init_config_ptr];

        if (init_config.contains("use_debug_pub")) {
            this->create_debug_pub = init_config["use_debug_pub"].get<bool>();
            RDX_INFO_DEV(node, __func__, "Set use_debug_pub: {}", this->create_debug_pub);
        }
    }

    if (json_params.contains(downstreams_ptr) && json_params[downstreams_ptr].is_array()) {
        for (const auto &action_config : json_params[downstreams_ptr]) {
            if (!action_config.contains("name")) {
                RDX_LOG_WARN(node, __func__, "{}", "Skipping downstream action without a name");
                continue;
            }

            std::string action_name = action_config["name"].get<std::string>();
            auto spec = std::make_shared<DownstreamSpec>();
            spec->accept_frame_action = action_name;
            RDX_INFO_DEV(node, __func__, "Configuring downstream action: {}", action_name);

            //! Configure delivery policy if present
            if (action_config.contains("delivery_policy")) {
                const auto &delivery_policy = action_config["delivery_policy"];

                if (delivery_policy.contains("number_of_enqueue_retry")) {
                    int64_t number_of_enqueue_retry = delivery_policy["number_of_enqueue_retry"].get<int64_t>();
                    spec->retry_strategy->set_number_of_enqueue_retry(number_of_enqueue_retry);
                    RDX_INFO_DEV(node, __func__, "Set number of enqueue retry for {}: {}", action_name, number_of_enqueue_retry);
                }

                if (delivery_policy.contains("wait_time_between_enqueue_retry")) {
                    double wait_time_between_enqueue_retry = delivery_policy["wait_time_between_enqueue_retry"].get<double>();
                    spec->retry_strategy->set_wait_time_between_enqueue_retry(std::chrono::duration_cast<DefaultTimeUnit_t>(std::chrono::duration<double, std::milli>(wait_time_between_enqueue_retry)));
                    RDX_INFO_DEV(node, __func__, "Set wait time between enqueue retry for {}: {} ms", action_name, wait_time_between_enqueue_retry);
                }

                if (delivery_policy.contains("number_of_delivery_retry")) {
                    int64_t number_of_delivery_retry = delivery_policy["number_of_delivery_retry"].get<int64_t>();
                    spec->retry_strategy->set_number_of_delivery_retry(number_of_delivery_retry);
                    RDX_INFO_DEV(node, __func__, "Set number of delivery retry for {}: {}", action_name, number_of_delivery_retry);
                }

                if (delivery_policy.contains("wait_time_between_delivery_retry")) {
                    double wait_time_between_delivery_retry = delivery_policy["wait_time_between_delivery_retry"].get<double>();
                    spec->retry_strategy->set_wait_time_between_delivery_retry(std::chrono::duration_cast<DefaultTimeUnit_t>(std::chrono::duration<double, std::milli>(wait_time_between_delivery_retry)));
                    RDX_INFO_DEV(node, __func__, "Set wait time between delivery retry for {}: {} ms", action_name, wait_time_between_delivery_retry);
                }

                if (delivery_policy.contains("wait_time_for_delivery_response")) {
                    double wait_time_for_delivery_response = delivery_policy["wait_time_for_delivery_response"].get<double>();
                    spec->retry_strategy->set_wait_time_for_delivery_response(std::chrono::duration_cast<DefaultTimeUnit_t>(std::chrono::duration<double, std::milli>(wait_time_for_delivery_response)));
                    RDX_INFO_DEV(node, __func__, "Set wait time for delivery response for {}: {} ms", action_name, wait_time_for_delivery_response);
                }
            }

            //! Add the downstream spec to the configuration using action name as key
            this->downstreams[spec->accept_frame_action] = spec;
            RDX_INFO_DEV(node, __func__, "Added downstream spec for action: {}", action_name);
        }
    } else {
        RDX_INFO_DEV(node, __func__, "{}", "No valid downstream configuration found in JSON parameters");
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
    std::string KeyDeliveryPolicyFallback = "delivery_policy_fallback";
    std::string KeyDeliveryOptions = "delivery_options";

    {
        auto jkey = JsonPointer_t(fmt::format("/{}/{}", KeyRuntimeConfig, KeyFrameIntervalMs));
        if (json_params.contains(jkey)) {
            this->frame_interval_ms = json_params[jkey].get<double>();
            RDX_INFO_DEV(node, __func__, "Got frame_interval_ms from JSON: {}", this->frame_interval_ms);
        }
    }

    {
        auto jkey = JsonPointer_t(fmt::format("/{}/{}", KeyRuntimeConfig, KeyStepIntervalMs));
        if (json_params.contains(jkey)) {
            this->step_interval_ms = json_params[jkey].get<double>();
            RDX_INFO_DEV(node, __func__, "Got step_interval_ms from JSON: {}", this->step_interval_ms);
        }
    }

    {
        auto jkey = JsonPointer_t(fmt::format("/{}/{}", KeyRuntimeConfig, KeyPublishToDebugTopic));
        if (json_params.contains(jkey)) {
            this->publish_to_debug_topic = json_params[jkey].get<bool>();
            RDX_INFO_DEV(node, __func__, "Got publish_to_debug_topic from JSON: {}", this->publish_to_debug_topic);
        }
    }

    {
        auto jkey = JsonPointer_t(fmt::format("/{}/{}", KeyRuntimeConfig, KeyDeliveryPolicyFallback));
        if (json_params.contains(jkey)) {
            const auto &fallback_policy = json_params[jkey];

            if (fallback_policy.contains("number_of_enqueue_retry")) {
                int64_t value = fallback_policy["number_of_enqueue_retry"].get<int64_t>();
                this->delivery_policy_fallback->set_number_of_enqueue_retry(value);
                RDX_INFO_DEV(node, __func__, "Set fallback number of enqueue retry: {}", value);
            }

            if (fallback_policy.contains("wait_time_between_enqueue_retry")) {
                double value = fallback_policy["wait_time_between_enqueue_retry"].get<double>();
                this->delivery_policy_fallback->set_wait_time_between_enqueue_retry(std::chrono::duration_cast<DefaultTimeUnit_t>(std::chrono::duration<double, std::milli>(value)));
                RDX_INFO_DEV(node, __func__, "Set fallback wait time between enqueue retry: {} ms", value);
            }

            if (fallback_policy.contains("number_of_delivery_retry")) {
                int64_t value = fallback_policy["number_of_delivery_retry"].get<int64_t>();
                this->delivery_policy_fallback->set_number_of_delivery_retry(value);
                RDX_INFO_DEV(node, __func__, "Set fallback number of delivery retry: {}", value);
            }

            if (fallback_policy.contains("wait_time_between_delivery_retry")) {
                double value = fallback_policy["wait_time_between_delivery_retry"].get<double>();
                this->delivery_policy_fallback->set_wait_time_between_delivery_retry(std::chrono::duration_cast<DefaultTimeUnit_t>(std::chrono::duration<double, std::milli>(value)));
                RDX_INFO_DEV(node, __func__, "Set fallback wait time between delivery retry: {} ms", value);
            }

            if (fallback_policy.contains("wait_time_for_delivery_response")) {
                double value = fallback_policy["wait_time_for_delivery_response"].get<double>();
                this->delivery_policy_fallback->set_wait_time_for_delivery_response(std::chrono::duration_cast<DefaultTimeUnit_t>(std::chrono::duration<double, std::milli>(value)));
                RDX_INFO_DEV(node, __func__, "Set fallback wait time for delivery response: {} ms", value);
            }
        }
    }

    {
        using FramePayloadType = FrameDeliveryOptions::FramePayloadType;
        using DropFrameStrategy = FrameDeliveryOptions::DropFrameStrategy;
        auto jkey = JsonPointer_t(fmt::format("/{}/{}", KeyRuntimeConfig, KeyDeliveryOptions));
        if (json_params.contains(jkey)) {
            const auto &delivery_options = json_params[jkey];

            if (delivery_options.contains("frame_payload_type")) {
                std::string frame_payload_type = delivery_options["frame_payload_type"].get<std::string>();
                if (frame_payload_type == "uncompressed") {
                    this->frame_delivery_options->frame_payload_type = FramePayloadType::Uncompressed;
                } else if (frame_payload_type == "uncompressed_by_shared_memory") {
                    this->frame_delivery_options->frame_payload_type = FramePayloadType::UncompressedBySharedMemory;
                } else if (frame_payload_type == "jpeg_encoded") {
                    this->frame_delivery_options->frame_payload_type = FramePayloadType::JpegEncoded;
                } else if (frame_payload_type == "png_encoded") {
                    this->frame_delivery_options->frame_payload_type = FramePayloadType::PngEncoded;
                }
                RDX_INFO_DEV(node, __func__, "Set frame payload type: {}", frame_payload_type);
            }

            if (delivery_options.contains("drop_frame_strategy")) {
                std::string drop_frame_strategy = delivery_options["drop_frame_strategy"].get<std::string>();
                if (drop_frame_strategy == "no_drop") {
                    this->frame_delivery_options->drop_frame_strategy = DropFrameStrategy::NoDrop;
                } else if (drop_frame_strategy == "drop_as_needed") {
                    this->frame_delivery_options->drop_frame_strategy = DropFrameStrategy::DropAsNeeded;
                }
                RDX_INFO_DEV(node, __func__, "Set drop frame strategy: {}", drop_frame_strategy);
            }

            if (delivery_options.contains("jpeg_quality")) {
                int jpeg_quality = delivery_options["jpeg_quality"].get<int>();
                this->frame_delivery_options->jpeg_quality = jpeg_quality;
                RDX_INFO_DEV(node, __func__, "Set JPEG quality: {}", jpeg_quality);
            }

            if (delivery_options.contains("num_buffer_frames")) {
                int num_buffer_frames = delivery_options["num_buffer_frames"].get<int>();
                this->frame_delivery_options->num_buffer_frames = num_buffer_frames;
                RDX_INFO_DEV(node, __func__, "Set number of buffer frames: {}", num_buffer_frames);
            }
        }
    }
}

} // namespace RedoxiVideoReaderBaseTypes

} // namespace redoxi_works
