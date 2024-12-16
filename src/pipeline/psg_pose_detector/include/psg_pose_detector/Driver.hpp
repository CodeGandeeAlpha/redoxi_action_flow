#pragma once

#include <redoxi_common_nodes/driver_nodes/CallActionDriverBase.hpp>
#include <psg_document_sink/AsyncDocumentInputPort.hpp>
#include <psg_pose_detector/AsyncGetKeypointsOutputPort.hpp>
#include <psg_master_node/AsyncDocumentOutputPort.hpp>

namespace redoxi_works
{
class PSGPoseDetectorDriver : public common_nodes::drivers::CallActionDriverBase<AsyncDocumentInputPort,
                                                                                 AsyncGetKeypointsOutputPort,
                                                                                 AsyncDocumentOutputPort>
{
  public:
    using BaseNode_t = common_nodes::drivers::CallActionDriverBase<AsyncDocumentInputPort,
                                                                   AsyncGetKeypointsOutputPort,
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

  protected:
    //! 记录document的异步字典
    using DocumentMap_t = boost::synchronized_value<std::map<int, std::shared_ptr<psg_private_msgs::msg::PsgDocument>>>;
    DocumentMap_t m_document_map;
};
} // namespace redoxi_works
