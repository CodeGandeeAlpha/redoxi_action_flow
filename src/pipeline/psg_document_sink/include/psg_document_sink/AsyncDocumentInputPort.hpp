#pragma once

#include <redoxi_common_nodes/async_action_port/AsyncActionInputPort.hpp>
#include <psg_document_sink/DocumentInputSpec.hpp>

namespace redoxi_works
{
using AsyncDocumentInputPort = AsyncActionInputPort<async_action_document_input_port::PSGDocumentInputPortSpec>;

} // namespace redoxi_works
