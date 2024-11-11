#include <psg_master_node/MasterNodeTypes.hpp>
#include <psg_master_node/MasterNode.hpp>
#include <redoxi_common_cpp/ros_utils/common.hpp>
#include <json_struct/json_struct.h>


#define PRINT_THREAD_ID_IN_LOG (false)

namespace redoxi_works
{
namespace psg_master_node
{

void InitConfig::from_parameters(PSGMasterNode *node)
{
    RDX_INFO_DEV(node, __func__, PRINT_THREAD_ID_IN_LOG, "{}", "load init config from node");
    auto &json_params = node->get_json_parameters();

    //! Load init config from json parameters if exists
    if (json_params.contains("init_config")) {
        RDX_INFO_DEV(node, __func__, PRINT_THREAD_ID_IN_LOG, "{}", "found init_config in json parameters");

        std::string json_str = json_params["init_config"].dump();
        JS::ParseContext context(json_str);
        auto error = context.parseTo(*this);
        if (error != JS::Error::NoError) {
            RDX_RAISE_ERROR("[{}] Error parsing init_config: {}", __func__, context.makeErrorString());
        }
    }

    RDX_INFO_DEV(node, __func__, PRINT_THREAD_ID_IN_LOG, "{}", "init config loaded");
}

void RuntimeConfig::from_parameters(PSGMasterNode *node)
{
    RDX_INFO_DEV(node, __func__, PRINT_THREAD_ID_IN_LOG, "{}", "load runtime config from node");
    auto &json_params = node->get_json_parameters();

    //! Load runtime config from json parameters if exists
    if (json_params.contains("runtime_config")) {
        RDX_INFO_DEV(node, __func__, PRINT_THREAD_ID_IN_LOG, "{}", "found runtime_config in json parameters");
        std::string json_str = json_params["runtime_config"].dump();
        JS::ParseContext context(json_str);
        auto error = context.parseTo(*this);
        if (error != JS::Error::NoError) {
            RDX_RAISE_ERROR("[{}] Error parsing runtime_config: {}", __func__, context.makeErrorString());
        }
    }

    RDX_INFO_DEV(node, __func__, PRINT_THREAD_ID_IN_LOG, "{}", "runtime config loaded");
}

} // namespace psg_master_node
} // namespace redoxi_works
