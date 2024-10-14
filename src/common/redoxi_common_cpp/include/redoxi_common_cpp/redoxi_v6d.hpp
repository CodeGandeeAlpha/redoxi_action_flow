#pragma once

#include "redoxi_common_cpp/redoxi_common_cpp.hpp"
#include <opencv2/opencv.hpp>
#include <vineyard/basic/ds/tensor.h>
#include <vineyard/client/client.h>

namespace redoxi_works
{
std::shared_ptr<vineyard::Client> create_v6d_client(const std::string &socket = "");
std::string get_default_v6d_socket();
cv::Mat from_v6d_tensor_to_cvmat(const std::shared_ptr<vineyard::Tensor<uint8_t>> &tensor);

template <typename T>
std::shared_ptr<vineyard::Tensor<T>> get_tensor_by_v6d_id(uint64_t id, std::shared_ptr<vineyard::Client> &client)
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

class VineyardClient
{
  public:
    VineyardClient(){};
    virtual ~VineyardClient();

    /**
     * @brief Connect to the vineyard server
     * @param socket The socket of the vineyard server
     * @return 0 if success, -1 if failed
     */
    int connect(const std::string &socket = "");

    // interface with opencv

    /**
     * @brief Get a cv::Mat from a vineyard object ID
     * @param object_id The ID of the vineyard object
     * @param output The output cv::Mat, the memory will be copied from vineyard server
     * @return 0 if success, -1 if failed
     */
    int read_cvmat(vineyard::ObjectID object_id, cv::Mat &output);

    /**
     * @brief Write a cv::Mat to a vineyard object ID
     * @param input The input cv::Mat
     * @param object_id The ID of the vineyard object
     * @return 0 if success, -1 if failed
     */
    int write_cvmat(const cv::Mat &input, vineyard::ObjectID &object_id);

  private:
    std::shared_ptr<vineyard::Client> m_client;
};

} // namespace redoxi_works