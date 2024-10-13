#include "redoxi_common_cpp/redoxi_v6d.hpp"
#include <memory>

namespace redoxi_works
{

std::shared_ptr<vineyard::Client> create_v6d_client(const std::string &socket)
{
    std::string v6d_ipc_socket = socket;
    if (v6d_ipc_socket.empty())
        v6d_ipc_socket = "/var/run/vineyard.sock";

    auto v6d_client = std::make_shared<vineyard::Client>();
    VINEYARD_CHECK_OK(v6d_client->Connect(v6d_ipc_socket));

    return v6d_client;
}

// std::shared_ptr<vineyard::Tensor<uint8_t>> get_tensor_by_v6d_id(uint64_t id, std::shared_ptr<vineyard::Client> &client)
// {
//     // 从 v6d_client 中获取 Tensor 对象
//     auto tensor = std::dynamic_pointer_cast<vineyard::Tensor<uint8_t>>(client->GetObject(id));

//     return tensor;
// }

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

VineyardClient::~VineyardClient() = default;

int VineyardClient::connect(const std::string &socket)
{
    try {
        m_client = create_v6d_client(socket);
        return 0;
    } catch (const std::exception &e) {
        // Handle any exceptions that might occur during connection
        return -1;
    }
}

int VineyardClient::read_cvmat(vineyard::ObjectID object_id, cv::Mat &output)
{
    try {
        auto tensor = get_tensor_by_v6d_id<uint8_t>(object_id, m_client);
        if (!tensor) {
            return -1;
        }
        output = from_v6d_tensor_to_cvmat(tensor);
        return 0;
    } catch (const std::exception &e) {
        // Handle any exceptions that might occur during the process
        return -1;
    }
}

int VineyardClient::write_cvmat(const cv::Mat &input, vineyard::ObjectID &object_id)
{
    try {
        int height = input.rows;
        int width = input.cols;
        int elem_size = input.elemSize();

        vineyard::TensorBuilder<uint8_t> builder(*m_client, {height, width, elem_size});
        auto tensor_data = builder.data();

        std::memcpy(tensor_data, input.data, height * width * elem_size);

        auto sealed = std::dynamic_pointer_cast<vineyard::Tensor<uint8_t>>(builder.Seal(*m_client));
        VINEYARD_CHECK_OK(m_client->Persist(sealed->id()));

        object_id = sealed->id();
        return 0;
    } catch (const std::exception &e) {
        // Handle any exceptions that might occur during the process
        return -1;
    }
}

} // namespace redoxi_works