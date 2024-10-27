#pragma once

#include "redoxi_common_nodes/redoxi_common_nodes.hpp"

namespace redoxi_works
{

//! Sends action requests to downstream nodes, asynchronously
//! Thread safe, can be used in multi thread executor
class AsyncActionOutputPort
{
  public:
    AsyncActionOutputPort();
    virtual ~AsyncActionOutputPort();
};

} // namespace redoxi_works