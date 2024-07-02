/** Copyright 2020-2023 Alibaba Group Holding Limited.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

#include <memory>
#include <string>
#include <thread>

#include "arrow/api.h"
#include "arrow/io/api.h"

#include "basic/ds/tensor.h"
#include "client/client.h"
#include "client/ds/object_meta.h"
#include "common/util/logging.h"

using namespace vineyard;  // NOLINT(build/namespaces)

#include <opencv2/opencv.hpp>

int main(int argc, char** argv) {
    std::string ipc_socket = "/var/run/vineyard.sock";

    // 连接到 Vineyard 服务器
    Client client;
    VINEYARD_CHECK_OK(client.Connect(ipc_socket));
    LOG(INFO) << "Connected to IPCServer: " << ipc_socket;

    // 使用 OpenCV 读取图像
    std::string image_path = "/mnt/chengxiao/code/psf_ros2_ws/src/flow_ros2_pipeline/test_v6d/test.jpg";  // 替换为您的图像路径
    cv::Mat image = cv::imread(image_path, cv::IMREAD_COLOR);
    if (image.empty()) {
        LOG(ERROR) << "Failed to read image: " << image_path;
        return -1;
    }

    // 获取图像的尺寸
    int height = image.rows;
    int width = image.cols;
    int ch = image.channels();

    // 创建 TensorBuilder，并根据图像尺寸构建 Tensor
    TensorBuilder<uint8_t> builder(client, {height, width, ch});
    auto tensor_data = builder.data();

    // 将图像数据复制到 Tensor 中
    for (int row = 0; row < height; ++row) {
        for (int col = 0; col < width; ++col) {
            cv::Vec3b pixel = image.at<cv::Vec3b>(row, col);
            tensor_data[row * width * 3 + col * 3 + 0] = pixel[0]; // Blue
            tensor_data[row * width * 3 + col * 3 + 1] = pixel[1]; // Green
            tensor_data[row * width * 3 + col * 3 + 2] = pixel[2]; // Red
        }
    }

    // 封存 Tensor 并持久化到 Vineyard
    auto sealed = std::dynamic_pointer_cast<Tensor<uint8_t>>(builder.Seal(client));
    VINEYARD_CHECK_OK(client.Persist(sealed->id()));

    ObjectID id = sealed->id();
    LOG(INFO) << "Successfully sealed, ObjectID: " << ObjectIDToString(id);

    // // 验证 Tensor 数据
    // auto tensor = sealed->ArrowTensor();
    // CHECK_EQ(tensor->shape().size(), 2);
    // CHECK_EQ(tensor->shape()[0], height);
    // CHECK_EQ(tensor->shape()[1], width);

    // LOG(INFO) << "Passed tensor tests...";

    // 断开与 Vineyard 的连接
    client.Disconnect();

    return 0;
}
