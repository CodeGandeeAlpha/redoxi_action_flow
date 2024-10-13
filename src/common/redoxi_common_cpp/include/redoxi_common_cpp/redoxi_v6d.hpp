#pragma once

#include "redoxi_common_cpp/redoxi_common_cpp.hpp"
#include <opencv2/opencv.hpp>
#include <vineyard/basic/ds/tensor.h>
#include <vineyard/client/client.h>

namespace redoxi_works
{
std::shared_ptr<vineyard::Client> create_v6d_client(const std::string &socket = "");
std::shared_ptr<vineyard::Tensor<uint8_t>> get_tensor_by_v6d_id(uint64_t id, std::shared_ptr<vineyard::Client> &client);
cv::Mat from_v6d_tensor_to_cvmat(const std::shared_ptr<vineyard::Tensor<uint8_t>> &tensor);

} // namespace redoxi_works