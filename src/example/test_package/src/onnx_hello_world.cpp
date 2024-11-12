#include <onnxruntime_cxx_api.h>
#include <spdlog/spdlog.h>
#include <string>
#include <memory>

// these can be set by cmake
#define TEST_ONNX_MODEL_PATH "/soft/workspace/code/psf_ros2_ws/tmp/models/yolov8n-pose-640.onnx"
// #define TEST_ONNX_PROVIDER_NAME "TensorrtExecutionProvider"
#define TEST_ONNX_PROVIDER_NAME "CUDAExecutionProvider"

void list_onnx_providers()
{
    auto providers = Ort::GetAvailableProviders();
    for (const auto &provider : providers) {
        spdlog::info("Provider: {}", provider);
    }
}

int main(int argc, char **argv)
{
    const std::string ExecutorName = TEST_ONNX_PROVIDER_NAME;
    const std::string ModelPath = TEST_ONNX_MODEL_PATH;

    Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "onnx_hello_world");
    Ort::SessionOptions session_options;
    session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);

    std::shared_ptr<Ort::Session> session;
    try {
        session = std::make_shared<Ort::Session>(env, ModelPath.c_str(), session_options);
    } catch (const Ort::Exception &e) {
        spdlog::error("Error creating session: {}", e.what());
        return 1;
    }

    Ort::MemoryInfo memory_info("Cpu", OrtDeviceAllocator, 0, OrtMemTypeCPU);

    return 0;
}