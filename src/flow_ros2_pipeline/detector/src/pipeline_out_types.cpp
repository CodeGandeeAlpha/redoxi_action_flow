#include <detector/pipeline_out_types.hpp>
#include <detector/pipeline_out.hpp>

namespace FlowRos2Pipeline {
    void DetectorOutInitConfig::from_parameters(DetectorOut* node) {
        process_document_action = node->get_parameter("process_document_action").as_string();
        process_detections_action = node->get_parameter("process_detections_action").as_string();
    }

    void DetectorOutRuntimeConfig::from_parameters(DetectorOut* node) {
        step_interval_ms = node->get_parameter("step_interval_ms").as_double();
        timeout_ms_send_to_downstream = node->get_parameter("timeout_ms_send_to_downstream").as_double();
    }
}