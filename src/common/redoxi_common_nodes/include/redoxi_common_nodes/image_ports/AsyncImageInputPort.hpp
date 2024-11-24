#pragma once

#include <redoxi_common_nodes/async_action_port/AsyncActionInputPort.hpp>
#include <redoxi_common_nodes/image_ports/ImageInputPortSpec.hpp>

namespace redoxi_works
{

using AsyncImageInputPort = AsyncActionInputPort<async_action_image_input_port::ImageActionInputPortSpec>;

} // namespace redoxi_works
