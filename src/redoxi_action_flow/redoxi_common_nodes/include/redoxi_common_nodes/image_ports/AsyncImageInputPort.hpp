#pragma once

#include <redoxi_common_nodes/async_action_port/AsyncActionInputPort.hpp>
#include <redoxi_common_nodes/image_ports/ImageInputPortSpec.hpp>

namespace redoxi_works::image_ports
{

using AsyncImageInputPort = AsyncActionInputPort<types::ImageActionInputPortSpec>;

} // namespace redoxi_works::image_ports
