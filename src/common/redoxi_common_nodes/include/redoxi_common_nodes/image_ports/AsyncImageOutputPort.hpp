#pragma once

#include <redoxi_common_nodes/async_action_port/AsyncActionOutputPort.hpp>
#include <redoxi_common_nodes/image_ports/ImageOutputPortSpec.hpp>

namespace redoxi_works
{

using AsyncImageOutputPort = AsyncActionOutputPort<async_action_image_output_port::ImageOutputPortSpec>;

} // namespace redoxi_works
