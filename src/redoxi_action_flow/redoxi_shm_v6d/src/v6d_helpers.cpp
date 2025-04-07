#include <redoxi_shm_v6d/v6d_helpers.hpp>
#include <redoxi_basic_cpp/logging/ros_logging.hpp>

namespace redoxi_works::shared_memory
{
std::shared_ptr<vineyard::Client> create_v6d_client(const std::string &socket)
{
    if (socket.empty())
        return nullptr;

    auto v6d_client = std::make_shared<vineyard::Client>();
    VINEYARD_CHECK_OK(v6d_client->Connect(socket));

    return v6d_client;
}

cv::Mat from_v6d_tensor_to_cvmat(const std::shared_ptr<vineyard::Tensor<uint8_t>> &tensor)
{
    RDX_DEBUG_DEV(nullptr, __func__, "{}", "getting tensor shape");
    auto shape = tensor->shape();
    int height = shape[0];
    int width = shape[1];
    int elem_size = shape[2];

    RDX_DEBUG_DEV(nullptr, __func__, "tensor shape: height={}, width={}, elem_size={}", height, width, elem_size);

    RDX_DEBUG_DEV(nullptr, __func__, "{}", "getting tensor data pointer");
    const uint8_t *tensor_data = tensor->data();

    RDX_DEBUG_DEV(nullptr, __func__, "{}", "creating cv::Mat from tensor data");
    cv::Mat frame(height, width, CV_8UC(elem_size), const_cast<uint8_t *>(tensor_data));

    RDX_DEBUG_DEV(nullptr, __func__, "{}", "cv::Mat created successfully");
    return frame;
}

} // namespace redoxi_works::shared_memory
