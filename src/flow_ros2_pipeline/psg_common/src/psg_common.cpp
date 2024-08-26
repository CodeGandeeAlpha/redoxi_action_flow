#include <boost/uuid/uuid_io.hpp>
#include <psg_common/psg_common.hpp>

std::shared_ptr<vineyard::Client> create_v6d_client(const std::string &socket)
{
    std::string v6d_ipc_socket = socket;
    if (v6d_ipc_socket.empty())
        v6d_ipc_socket = "/var/run/vineyard.sock";

    auto v6d_client = std::make_shared<vineyard::Client>();
    VINEYARD_CHECK_OK(v6d_client->Connect(v6d_ipc_socket));

    return v6d_client;
}

std::shared_ptr<vineyard::Tensor<uint8_t>> get_tensor_by_v6d_id(uint64_t id, std::shared_ptr<vineyard::Client> &client)
{
    // 从 v6d_client 中获取 Tensor 对象
    auto tensor = std::dynamic_pointer_cast<vineyard::Tensor<uint8_t>>(client->GetObject(id));

    return tensor;
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

std::string uuid_to_string(const std::array<uint8_t, 16> &uuid)
{
    boost::uuids::uuid uuid_ = *reinterpret_cast<const boost::uuids::uuid *>(uuid.data());
    return boost::uuids::to_string(uuid_);
}