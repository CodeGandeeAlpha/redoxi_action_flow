#include "redoxi_common_cpp/redoxi_v6d.hpp"
#include <opencv2/opencv.hpp>
#include <vineyard/client/client.h>
#include <vineyard/common/util/uuid.h>
#include <spdlog/spdlog.h>

namespace v6d = vineyard;
int test_v6d_raw();
int test_v6d_client();

int main()
{
    test_v6d_raw();
    test_v6d_client();
    return 0;
}

int test_v6d_client()
{
    // Create a VineyardClient instance
    redoxi_works::VineyardClient v6d_client;

    // Connect to the vineyard server
    if (v6d_client.connect(redoxi_works::get_default_v6d_socket()) != 0) {
        spdlog::error("Failed to connect to vineyard server");
        return -1;
    }

    // Create a test cv::Mat
    cv::Mat test_mat(3, 4, CV_8UC3);
    cv::randu(test_mat, cv::Scalar(0, 0, 0), cv::Scalar(255, 255, 255));

    // Print the original cv::Mat
    spdlog::info("Original cv::Mat:");
    for (int i = 0; i < test_mat.rows; ++i) {
        for (int j = 0; j < test_mat.cols; ++j) {
            cv::Vec3b pixel = test_mat.at<cv::Vec3b>(i, j);
            spdlog::info("({}, {}, {})", pixel[0], pixel[1], pixel[2]);
        }
    }

    // Write the cv::Mat to vineyard
    vineyard::ObjectID object_id;
    if (v6d_client.write_cvmat(test_mat, object_id) != 0) {
        spdlog::error("Failed to write cv::Mat to vineyard");
        return -1;
    }

    spdlog::info("Wrote cv::Mat to vineyard with ID: {}", object_id);

    // Read the cv::Mat back from vineyard
    cv::Mat retrieved_mat;
    if (v6d_client.read_cvmat(object_id, retrieved_mat) != 0) {
        spdlog::error("Failed to read cv::Mat from vineyard");
        return -1;
    }

    // Print the retrieved cv::Mat
    spdlog::info("Retrieved cv::Mat:");
    for (int i = 0; i < retrieved_mat.rows; ++i) {
        for (int j = 0; j < retrieved_mat.cols; ++j) {
            cv::Vec3b pixel = retrieved_mat.at<cv::Vec3b>(i, j);
            spdlog::info("({}, {}, {})", pixel[0], pixel[1], pixel[2]);
        }
    }

    // Check if the retrieved cv::Mat matches the original
    if (test_mat.size() == retrieved_mat.size() && test_mat.type() == retrieved_mat.type()) {
        cv::Mat diff;
        cv::absdiff(test_mat, retrieved_mat, diff);
        bool matrices_equal = (cv::countNonZero(diff.reshape(1)) == 0);

        if (matrices_equal) {
            spdlog::info("Test passed: Retrieved cv::Mat matches the original");
        } else {
            spdlog::error("Test failed: Retrieved cv::Mat does not match the original");
        }
    } else {
        spdlog::error("Test failed: Retrieved cv::Mat has different size or type");
    }

    // Clean up (assuming there's a method to delete objects, if not, you may need to implement one)
    // v6d_client.delete_object(object_id);
    return 0;
}

int test_v6d_raw()
{
    // Create a vineyard client
    vineyard::Client client;
    VINEYARD_CHECK_OK(client.Connect(redoxi_works::get_default_v6d_socket()));

    // Create a vineyard tensor
    std::vector<int64_t> shape = {3, 4, 5};
    std::vector<int> data(3 * 4 * 5);
    for (size_t i = 0; i < data.size(); ++i) {
        data[i] = static_cast<int>(i);
    }

    // fill data into tensor
    v6d::TensorBuilder<int> builder(client, shape);
    auto tensor_data = builder.data();
    memcpy(tensor_data, data.data(), data.size() * sizeof(int));

    // seal the tensor, and then send to vineyard server by persist
    auto sealed = std::dynamic_pointer_cast<v6d::Tensor<int>>(builder.Seal(client));
    vineyard::ObjectID tensor_id = sealed->id();
    VINEYARD_CHECK_OK(client.Persist(tensor_id));

    spdlog::info("Created tensor with ID: {}", tensor_id);

    // Retrieve the tensor back
    std::shared_ptr<vineyard::Tensor<int>> retrieved_tensor;
    VINEYARD_CHECK_OK(client.GetObject(tensor_id, retrieved_tensor));

    // Check if the retrieved tensor matches the original
    auto &retrieved_shape = retrieved_tensor->shape();
    auto retrieved_data = retrieved_tensor->data();

    bool shapes_match = (retrieved_shape == shape);
    bool data_matches = std::equal(data.begin(), data.end(), retrieved_data, retrieved_data + data.size());

    if (shapes_match && data_matches) {
        spdlog::info("Test passed: Retrieved tensor matches the original");
    } else {
        spdlog::error("Test failed: Retrieved tensor does not match the original");
    }

    // Clean up
    VINEYARD_CHECK_OK(client.DelData(tensor_id));

    return 0;
}
