#pragma once

#include <redoxi_shm_v6d/redoxi_shm_v6d.hpp>
#include <memory>
#include <opencv2/opencv.hpp>
#include <vineyard/basic/ds/tensor.h>
#include <vineyard/client/client.h>

namespace redoxi_works::shared_memory
{

std::shared_ptr<vineyard::Client> create_v6d_client(const std::string &socket);
cv::Mat from_v6d_tensor_to_cvmat(const std::shared_ptr<vineyard::Tensor<uint8_t>> &tensor);

template <typename T>
std::shared_ptr<vineyard::Tensor<T>>
    get_tensor_by_v6d_id(uint64_t id, vineyard::Client *client)
{
    // Attempt to get the object from the vineyard client
    auto object = client->GetObject(id);
    if (!object) {
        return nullptr;
    }

    // Try to cast the object to a Tensor<T>
    auto tensor = std::dynamic_pointer_cast<vineyard::Tensor<T>>(object);
    if (!tensor) {
        return nullptr;
    }

    return tensor;
}

} // namespace redoxi_works::shared_memory
