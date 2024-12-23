#include "redoxi_inference_rknn/RknnModelInference.hpp"
#include <filesystem>
#include <spdlog/spdlog.h>

namespace rdx_rknn = redoxi_works::inference::rknn;
namespace rdx = redoxi_works;
namespace fs = std::filesystem;
namespace common_keys = rdx::inference::common_config_keys;
namespace common_device_types = rdx::inference::common_device_types;

fs::path model_path = "/data/code/psf_ros2_ws/tmp/models/rknn/yolov8n-pose-fp-bs1.rknn";

int main(int argc, char **argv)
{
    rdx_rknn::RknnModelInference model;

    // create init params and fill it
    spdlog::info("Creating init params");
    auto init_params = std::dynamic_pointer_cast<rdx_rknn::RknnModelConfig>(model.create_init_params());
    init_params->set_string(common_keys::ModelPath, model_path.string());
    init_params->set_string(common_keys::DeviceType, common_device_types::RKNPU);

    // open the model
    spdlog::info("Opening the model");
    auto ret = model.open(init_params);
    spdlog::info("Model opened with return code: {}", ret);

    // create inference inout data
    // auto inference_inout_data = model.create_inference_inout_data();

    spdlog::info("End of test_rknn_infer");
    return 0;
}
