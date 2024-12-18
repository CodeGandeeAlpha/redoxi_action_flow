#pragma once

#include <redoxi_common_nodes/driver_nodes/CallActionDriverBase.hpp>
#include <psg_document_sink/AsyncDocumentInputPort.hpp>
#include <psg_detector/AsyncGetDetectionsOutputPort.hpp>
#include <psg_master_node/AsyncDocumentOutputPort.hpp>

namespace redoxi_works
{
class PSGDetectorDriver : public common_nodes::drivers::CallActionDriverBase<AsyncDocumentInputPort,
                                                                             AsyncGetDetectionsOutputPort,
                                                                             AsyncDocumentOutputPort>
{
  public:
    using BaseNode_t = common_nodes::drivers::CallActionDriverBase<AsyncDocumentInputPort,
                                                                   AsyncGetDetectionsOutputPort,
                                                                   AsyncDocumentOutputPort>;
    using BaseNode_t::CallActionDriverBase;

  protected:
    int _on_process_callee_result(OutputTypes::OutputRequest_t *output_request,
                                  OutputTypes::OutputDeliveryPolicy_t *output_enqueue_policy,
                                  std::shared_ptr<const CalleeTypes::RequestOutputActionResult_t> callee_result,
                                  const CalleeTypes::RequestOutputRequest_t &callee_request,
                                  const CalleeTypes::Downstream_t &downstream) override;

    int _on_process_input_request(InputRequestHandler_t::OutputRequest_t *output_request,
                                  std::optional<InputRequestHandler_t::OutputDeliveryPolicy_t> *output_enqueue_policy,
                                  InputRequestHandler_t::InputActionResult_t *output_result,
                                  std::shared_ptr<const InputTypes::SourceData_t> source_data,
                                  InputRequestHandler_t::ResourceToken_t &resource_token) override;
};
} // namespace redoxi_works