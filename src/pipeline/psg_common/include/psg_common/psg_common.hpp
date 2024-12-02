#pragma once
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>
#include <vineyard/basic/ds/tensor.h>
#include <vineyard/client/client.h>

namespace vineyard
{
class Client;
};

std::shared_ptr<vineyard::Client> create_v6d_client(const std::string &socket = "");
std::shared_ptr<vineyard::Tensor<uint8_t>> get_tensor_by_v6d_id(uint64_t id, std::shared_ptr<vineyard::Client> &client);
cv::Mat from_v6d_tensor_to_cvmat(const std::shared_ptr<vineyard::Tensor<uint8_t>> &tensor);
std::string uuid_to_string(const std::array<uint8_t, 16> &uuid);
cv::Scalar get_color(const int id);


namespace FlowRos2Pipeline
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
} // namespace FlowRos2Pipeline