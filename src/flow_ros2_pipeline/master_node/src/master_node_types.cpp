#include <master_node/master_node_types.hpp>
#include <master_node/master_node.hpp>

namespace FlowRos2Pipeline {
    void MasterNodeInitConfig::from_parameters(MasterNode* node) {
        downstream_action_name = node->get_parameter("downstream_action").as_string();
    }

    void MasterNodeRuntimeConfig::from_parameters(MasterNode* node) {
        frame_internal_ms = node->get_parameter("frame_internal_ms").as_double();
    }
}