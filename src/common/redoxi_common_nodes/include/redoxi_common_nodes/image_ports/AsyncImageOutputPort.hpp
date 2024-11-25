#pragma once

#include <redoxi_common_nodes/async_action_port/AsyncActionOutputPort.hpp>
#include <redoxi_common_nodes/image_ports/ImageOutputPortSpec.hpp>

namespace redoxi_works::image_ports
{

using AsyncImageOutputPort = AsyncActionOutputPort<types::ImageActionOutputPortSpec>;

} // namespace redoxi_works::image_ports
