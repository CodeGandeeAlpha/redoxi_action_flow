#include <redoxi_video_reader/base/VideoReaderBaseTypes.hpp>
#include <redoxi_video_reader/base/VideoReaderBase.hpp>
#include <nlohmann/json.hpp>
#include <map>
#include <regex>

namespace redoxi_works
{

namespace RedoxiVideoReaderBaseTypes
{
void InitConfig::from_parameters(RedoxiVideoReaderBase *node)
{
    // try to get params from json string
    nlohmann::json json_params;
    {
        rclcpp::Parameter _json_params;
        auto pkey = redoxi_works::RosParams::ParamAsJsonString::MainKey;
        auto ret = node->get_parameter(pkey, _json_params);
        if (!ret)
            return;
        // try to parse the parameter as json
        try {
            json_params = nlohmann::json::parse(_json_params.as_string());
        } catch (const nlohmann::json::parse_error &e) {
            RDX_RAISE_ERROR("Failed to parse json string: {}", e.what());
            return;
        }
    }

    // nothing to parse, return
    if (json_params.empty())
        return;

    // get downstream parameters
    std::string key_downstream = "downstreams";
    std::string key_ds_action = "accept_frame_actions";

    if (!json_params.contains(key_downstream))
        return;

    /**
     * The json format is like:
     * {
     *   "downstreams": {
     *     "video_sink": {  // the name of the downstream node
     *       "accept_frame_actions": ["action_1", "action_2"]
     *     }
     *   }
     * }
     */
    //! Parse the downstreams configuration
    if (json_params[key_downstream].is_object()) {
        for (const auto &[downstream_name, downstream_config] : json_params[key_downstream].items()) {
            //! Check if the downstream has accept_frame_actions
            if (downstream_config.contains(key_ds_action) && downstream_config[key_ds_action].is_array()) {
                const auto &actions = downstream_config[key_ds_action];
                for (const auto &action : actions) {
                    if (action.is_string()) {
                        auto spec = std::make_shared<DownstreamSpec>();
                        spec->accept_frame_action = action.get<std::string>();

                        //! Add the downstream spec to the configuration using action name as key
                        this->downstreams[spec->accept_frame_action] = spec;
                        RCLCPP_DEBUG(node->get_logger(), "[%s] got downstream %s", node->get_name(), spec->accept_frame_action.c_str());
                    }
                }
            }
        }
    }
}

void RuntimeConfig::from_parameters(RedoxiVideoReaderBase *node)
{
}

} // namespace RedoxiVideoReaderBaseTypes

} // namespace redoxi_works
