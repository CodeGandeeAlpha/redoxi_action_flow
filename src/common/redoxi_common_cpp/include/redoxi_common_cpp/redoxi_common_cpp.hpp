#ifndef REDOXI_COMMON_CPP__REDOXI_COMMON_CPP_HPP_
#define REDOXI_COMMON_CPP__REDOXI_COMMON_CPP_HPP_

#include "redoxi_common_cpp/visibility_control.h"

namespace redoxi_works
{
class IOpenCloseProtocol
{
  public:
    virtual ~IOpenCloseProtocol()
    {
    }

    // return 0 if success, otherwise return error code
    virtual int open() = 0;
    virtual int start() = 0;
    virtual int stop() = 0;
    virtual int close() = 0;
};

class IStartStopProtocol
{
  public:
    virtual ~IStartStopProtocol()
    {
    }

    // return 0 if success, otherwise return error code
    virtual int start() = 0;
    virtual int stop() = 0;
};

namespace ReturnCode
{
const int SUCCESS = 0;
const int REJECTED = 1;
const int ERROR = -1;

// reserved status code, your custom status code should be greater than this
const int MAX_RESERVED_STATUS = 10000;
}; // namespace ReturnCode

namespace NodeStatusCode
{
const int BEFORE_INIT = 0;
const int INITIALIZED = 1;
const int OPENED = 2;
const int STARTED = 3;
const int STOPPED = 4;
const int CLOSED = 5;

// reserved status code, your custom status code should be greater than this
const int MAX_RESERVED_STATUS = 10000;
}; // namespace NodeStatusCode

namespace SignalCode
{
const int RUN = 0;
const int FLUSH = 1;
const int TERMINATE = 2;
}; // namespace SignalCode


const double DefaultTimeoutMs = 10000;

// how many ms to wait before next _step()
const double DefaultNodeStepIntervalMs = 1;

} // namespace redoxi_works

#endif // REDOXI_COMMON_CPP__REDOXI_COMMON_CPP_HPP_
