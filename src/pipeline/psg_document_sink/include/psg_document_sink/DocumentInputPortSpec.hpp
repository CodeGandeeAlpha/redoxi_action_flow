#pragma once

#include <redoxi_common_cpp/redoxi_common_cpp.hpp>
#include <redoxi_common_nodes/async_action_port/AsyncActionInputTypes.hpp>
#include <psg_private_msgs/action/process_psg_document.hpp>

namespace redoxi_works
{

namespace async_action_document_input_port
{
//! The specification for the document input port
using TimeUnit = DefaultTimeUnit_t;
using DocumentActionType = psg_private_msgs::action::ProcessPsgDocument;
using DocumentActionDataTrait = RedoxiActionDataTrait<DocumentActionType>;

using PSGDocumentInputPortSpec =
    input_port_types::DefaultAsyncActionInputPortSpec<DocumentActionType, DocumentActionDataTrait, TimeUnit>;

} // namespace async_action_document_input_port

} // namespace redoxi_works
