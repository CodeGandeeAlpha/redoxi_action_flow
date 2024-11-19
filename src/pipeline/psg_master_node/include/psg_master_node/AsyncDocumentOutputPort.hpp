#pragma once

#include <redoxi_common_nodes/async_action_port/AsyncActionOutputPort.hpp>
#include <psg_master_node/DocumentOutputSpec.hpp>

namespace redoxi_works
{

using AsyncDocumentOutputPort = AsyncActionOutputPort<async_action_document_output_port::PSGDocumentOutputPortSpec>;

} // namespace redoxi_works