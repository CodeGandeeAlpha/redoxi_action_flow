#pragma once

#include <redoxi_common_cpp/redoxi_common_cpp.hpp>
#include <redoxi_common_nodes/async_action_port/AsyncActionInputTypes.hpp>
#include <redoxi_public_msgs/action/process_frame.hpp>

namespace redoxi_works
{

namespace async_action_image_input_port
{
//! The specification for the image input port
using TimeUnit = DefaultTimeUnit_t;
using ImageActionType = redoxi_public_msgs::action::ProcessFrame;
using ImageActionDataTrait = RedoxiActionDataTrait<ImageActionType>;

using ImageActionInputPortSpec =
    input_port_types::DefaultAsyncActionInputPortSpec<ImageActionType, ImageActionDataTrait, TimeUnit>;

} // namespace async_action_image_input_port

} // namespace redoxi_works
