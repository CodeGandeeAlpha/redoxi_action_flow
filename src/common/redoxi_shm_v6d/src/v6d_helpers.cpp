#include <redoxi_shm_v6d/v6d_helpers.hpp>

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
    // 获取 Tensor 的尺寸信息
    auto shape = tensor->shape();
    int height = shape[0];
    int width = shape[1];
    int elem_size = shape[2];

    // 获取 Tensor 的数据指针
    const uint8_t *tensor_data = tensor->data();

    // 使用数据指针创建 cv::Mat 对象
    cv::Mat frame(height, width, CV_8UC(elem_size), const_cast<uint8_t *>(tensor_data));

    return frame;
}

} // namespace redoxi_works::shared_memory
