#include <master_node/master_node_types.hpp>
#include <master_node/master_node.hpp>

namespace FlowRos2Pipeline {
    void MasterNodeInitConfig::from_parameters(MasterNode* node) {
        downstream_action_name = node->get_parameter("downstream_action").as_string();
    }

    void MasterNodeRuntimeConfig::from_parameters(MasterNode* node) {
        step_interval_ms = node->get_parameter("step_interval_ms").as_double();
    }
}