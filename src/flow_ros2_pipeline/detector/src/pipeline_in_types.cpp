#include <detector/pipeline_in_types.hpp>
#include <detector/pipeline_in.hpp>

namespace FlowRos2Pipeline {
    void DetectorInInitConfig::from_parameters(DetectorIn* node) {
        process_document_action = node->get_parameter("process_document_action").as_string();
    }

    void DetectorInRuntimeConfig::from_parameters(DetectorIn* node) {
        step_interval_ms = node->get_parameter("step_interval_ms").as_double();
        timeout_ms_send_to_downstream = node->get_parameter("timeout_ms_send_to_downstream").as_double();
    }
}