#include <onnxruntime_cxx_api.h>
#include <spdlog/spdlog.h>
#include <string>
#include <memory>
#include <map>
#include <chrono>
#include <algorithm>
#include <numeric>
#include <optional>

// these can be set by cmake
// #define TEST_ONNX_MODEL_PATH "/soft/workspace/code/psf_ros2_ws/tmp/models/yolov8n-pose-640.onnx"
#define TEST_ONNX_MODEL_PATH "/soft/workspace/code/psf_ros2_ws/tmp/models/yolov8m-pose-dynbatch.onnx"
// #define TEST_ONNX_PROVIDER_NAME "TensorrtExecutionProvider"
#define TEST_ONNX_PROVIDER_NAME "CUDAExecutionProvider"

enum class OnnxExecutionProviderType {
    CPU,
    CUDA,
    TensorRT
};

struct OnnxPortInfo {
    std::string name;
    ONNXTensorElementDataType dtype;
    std::string dtype_str; // string representation of dtype
    std::vector<int64_t> shape;
    bool is_input = false;

    std::string to_string() const
    {
        return fmt::format("{}: {} {} {}", name, dtype_str, fmt::join(shape, ", "), is_input ? "input" : "output");
    }

    //! Define comparison operators for OnnxPortInfo based on the name
    bool operator<(const OnnxPortInfo &other) const
    {
        return name < other.name;
    }

    bool operator==(const OnnxPortInfo &other) const
    {
        return name == other.name;
    }
};

struct OnnxPortData {
    std::shared_ptr<std::vector<float>> data;
    std::shared_ptr<Ort::Value> tensor;
    std::shared_ptr<Ort::MemoryInfo> memory_info;
};

struct OnnxRuntimeData {
    std::shared_ptr<Ort::Session> session;
    std::map<OnnxPortInfo, OnnxPortData> tensor_caches;
    std::shared_ptr<Ort::IoBinding> io_binding;
};

std::vector<std::string>
    list_onnx_providers();
std::shared_ptr<Ort::Session> create_onnx_session(const std::string &model_path,
                                                  OnnxExecutionProviderType provider_type,
                                                  Ort::Env &env);
std::vector<OnnxPortInfo> list_onnx_model_info(const Ort::Session &session);
std::shared_ptr<OnnxRuntimeData> create_onnx_runtime_data(
    std::shared_ptr<Ort::Session> session, std::optional<int> num_batch = std::nullopt);

std::string onnx_element_type_to_string(ONNXTensorElementDataType dtype);
int onnx_inference_cpu();
int onnx_inference_cuda();

int main(int argc, char **argv)
{
    const std::string ModelPath = TEST_ONNX_MODEL_PATH;
    int num_batch = 10;
    Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "onnx_hello_world");

    auto session = create_onnx_session(ModelPath, OnnxExecutionProviderType::CUDA, env);
    auto ports = list_onnx_model_info(*session);
    for (const auto &port : ports) {
        spdlog::info("Port: {}", port.to_string());
    }

    auto runtime_data = create_onnx_runtime_data(session, num_batch);

    // Run the model 1000 times with randomized input
    auto tensor_caches = runtime_data->tensor_caches;
    try {
        spdlog::info("Running inference 1000 times with randomized input");
        for (int i = 0; i < 1000; ++i) {
            for (auto &entry : tensor_caches) {
                if (entry.first.is_input) {
                    std::generate(entry.second.data->begin(), entry.second.data->end(), []() { return static_cast<float>(rand()) / RAND_MAX; });
                    runtime_data->io_binding->BindInput(entry.first.name.c_str(), *(entry.second.tensor));
                }
            }
            auto start_time = std::chrono::high_resolution_clock::now();

            //! Synchronize inputs
            spdlog::info("Synchronizing inputs");
            runtime_data->io_binding->SynchronizeInputs();
            //! Run the model
            spdlog::info("Running the model");
            session->Run(Ort::RunOptions{nullptr}, *runtime_data->io_binding);
            //! Synchronize outputs
            spdlog::info("Synchronizing outputs");
            runtime_data->io_binding->SynchronizeOutputs();

            auto end_time = std::chrono::high_resolution_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();
            auto average_time_per_sample = elapsed / num_batch;

            //! Print first element of each output tensor
            spdlog::info("Getting output values");
            auto output_values = runtime_data->io_binding->GetOutputValues();
            auto output_names = runtime_data->io_binding->GetOutputNames();
            for (size_t i = 0; i < output_names.size(); ++i) {
                spdlog::info("Getting output '{}'", output_names[i]);
                const float *data = output_values[i].GetTensorData<float>();
                spdlog::info("Output '{}' first element: {}", output_names[i], data[0]);
            }
            spdlog::info("\033[33mIteration {}: Inference completed in {} us (Average per sample: {} us)\033[0m", i + 1, elapsed, average_time_per_sample);
        }
    } catch (const Ort::Exception &e) {
        spdlog::error("Error during inference: {}", e.what());
        return 1;
    }

    return 0;
}

std::shared_ptr<OnnxRuntimeData> create_onnx_runtime_data(
    std::shared_ptr<Ort::Session> session, std::optional<int> num_batch)
{
    spdlog::info("Creating OnnxRuntimeData object");
    auto runtime_data = std::make_shared<OnnxRuntimeData>();
    runtime_data->session = session;

    spdlog::info("Listing all input and output ports");
    auto ports = list_onnx_model_info(*session);

    spdlog::info("Creating ONNX tensors to bind to ports");
    for (const auto &port : ports) {
        if (port.is_input) {
            spdlog::info("Pre-allocating space for input port: {}", port.name);
            int batch_size = (port.shape[0] == -1) ? num_batch.value_or(1) : port.shape[0];
            int total_size = batch_size * std::accumulate(port.shape.begin() + 1, port.shape.end(), 1, std::multiplies<int64_t>());
            auto input_data = std::make_shared<std::vector<float>>(total_size);
            auto memory_info = std::make_shared<Ort::MemoryInfo>(Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault));

            std::vector<int64_t> shape(port.shape.begin(), port.shape.end());
            shape[0] = batch_size;

            auto tensor = std::make_shared<Ort::Value>(Ort::Value::CreateTensor<float>(
                *memory_info, input_data->data(), input_data->size(), shape.data(), shape.size()));
            runtime_data->tensor_caches[port] = {input_data, tensor, memory_info};
        } else {
            spdlog::info("No pre-allocation for output port: {}", port.name);
            auto memory_info = std::make_shared<Ort::MemoryInfo>(
                Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault));
            runtime_data->tensor_caches[port] = {nullptr, nullptr, memory_info};
        }
    }

    spdlog::info("Binding the tensors to the session");
    runtime_data->io_binding = std::make_shared<Ort::IoBinding>(*session);
    for (const auto &entry : runtime_data->tensor_caches) {
        const auto &port = entry.first;
        const auto &data = entry.second;
        if (port.is_input) {
            spdlog::info("Binding input tensor for port: {}", port.name);
            runtime_data->io_binding->BindInput(port.name.c_str(), *(data.tensor));
        } else {
            spdlog::info("Binding output tensor for port: {}", port.name);
            runtime_data->io_binding->BindOutput(port.name.c_str(), *(data.memory_info));
        }
    }
    return runtime_data;
}

std::shared_ptr<Ort::Session> create_onnx_session(const std::string &model_path,
                                                  OnnxExecutionProviderType provider_type,
                                                  Ort::Env &env)
{
    //! Create an ONNX session and load the model based on the provider type
    Ort::SessionOptions session_options;
    session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);

    if (provider_type == OnnxExecutionProviderType::CUDA) {
        spdlog::info("Configuring CUDA provider options");
        OrtCUDAProviderOptions cuda_options;
        session_options.AppendExecutionProvider_CUDA(cuda_options);
    } else if (provider_type == OnnxExecutionProviderType::CPU) {
        spdlog::info("Using CPU Execution Provider");
        // No additional configuration needed for CPU
    } else if (provider_type == OnnxExecutionProviderType::TensorRT) {
        //! Configuring TensorRT Execution Provider
        spdlog::info("Configuring TensorRT Execution Provider");
        OrtTensorRTProviderOptions trt_options;
        trt_options.device_id = 0;
        trt_options.trt_max_partition_iterations = 10;
        session_options.AppendExecutionProvider_TensorRT(trt_options);
    } else {
        spdlog::error("Unknown provider type: {}", static_cast<int>(provider_type));
        return nullptr;
    }

    std::shared_ptr<Ort::Session> session;
    try {
        spdlog::info("Creating session with model path: {}", model_path);
        session = std::make_shared<Ort::Session>(env, model_path.c_str(), session_options);
    } catch (const Ort::Exception &e) {
        spdlog::error("Error creating session: {}", e.what());
        return nullptr;
    }

    return session;
}

int onnx_inference_cuda()
{
    const std::string ModelPath = TEST_ONNX_MODEL_PATH;
    int num_batch = 10;

    spdlog::info("Initializing ONNX environment with logging level: WARNING");
    Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "onnx_hello_world");

    spdlog::info("Setting up session options");
    Ort::SessionOptions session_options;
    spdlog::info("Disabling graph optimizations");
    session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);

    spdlog::info("Configuring CUDA provider options");
    OrtCUDAProviderOptions cuda_options;
    spdlog::info("Appending CUDA execution provider to session options");
    session_options.AppendExecutionProvider_CUDA(cuda_options);

    std::shared_ptr<Ort::Session> session;
    try {
        spdlog::info("Creating session with CUDA Execution Provider");
        session = std::make_shared<Ort::Session>(env, ModelPath.c_str(), session_options);
    } catch (const Ort::Exception &e) {
        spdlog::error("Error creating session: {}", e.what());
        return 1;
    }

    auto ports = list_onnx_model_info(*session);
    spdlog::info("Listing all input and output ports:");
    for (const auto &port : ports) {
        spdlog::info("Port: {}", port.to_string());
    }

    Ort::IoBinding io_binding(*session);

    // Bind input and output tensors
    std::map<OnnxPortInfo, OnnxPortData> tensor_caches;
    for (const auto &port : ports) {
        if (port.is_input) {
            spdlog::info("Creating input tensor for port: {}; Port shape: {}", port.name, fmt::join(port.shape, ", "));
            auto input_tensor_values = std::make_shared<std::vector<float>>(port.shape[0] * port.shape[1] * port.shape[2] * port.shape[3]);
            auto memory_info = std::make_shared<Ort::MemoryInfo>(Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault));
            auto input_tensor = std::make_shared<Ort::Value>(Ort::Value::CreateTensor<float>(*memory_info,
                                                                                             input_tensor_values->data(),
                                                                                             input_tensor_values->size(),
                                                                                             port.shape.data(),
                                                                                             port.shape.size()));

            OnnxPortData port_data{input_tensor_values, input_tensor, memory_info};
            tensor_caches[port] = std::move(port_data);

            spdlog::info("Binding input tensor for port: {}", port.name);
            io_binding.BindInput(port.name.c_str(), *input_tensor);
        } else {
            spdlog::info("Creating output tensor for port: {}; Port shape: {}", port.name, fmt::join(port.shape, ", "));
            auto output_tensor_values = std::make_shared<std::vector<float>>(port.shape[0] * port.shape[1] * port.shape[2] * port.shape[3]);
            auto memory_info = std::make_shared<Ort::MemoryInfo>(Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault));
            auto output_tensor = std::make_shared<Ort::Value>(Ort::Value::CreateTensor<float>(*memory_info,
                                                                                              output_tensor_values->data(),
                                                                                              output_tensor_values->size(),
                                                                                              port.shape.data(),
                                                                                              port.shape.size()));

            OnnxPortData port_data{output_tensor_values, output_tensor, memory_info};
            tensor_caches[port] = std::move(port_data);

            spdlog::info("Binding output tensor for port: {}", port.name);
            io_binding.BindOutput(port.name.c_str(), *output_tensor);
        }
    }

    // Run the model 1000 times with randomized input
    try {
        spdlog::info("Running inference 1000 times with randomized input");
        for (int i = 0; i < 1000; ++i) {
            for (auto &entry : tensor_caches) {
                if (entry.first.is_input) {
                    std::generate(entry.second.data->begin(), entry.second.data->end(), []() { return static_cast<float>(rand()) / RAND_MAX; });
                }
            }
            auto start_time = std::chrono::high_resolution_clock::now();
            session->Run(Ort::RunOptions{nullptr}, io_binding);
            auto end_time = std::chrono::high_resolution_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();
            spdlog::info("\033[33mIteration {}: Inference completed in {} us\033[0m", i + 1, elapsed);
        }
    } catch (const Ort::Exception &e) {
        spdlog::error("Error during inference: {}", e.what());
        return 1;
    }

    return 0;
}

int onnx_inference_cpu()
{
    const std::string ExecutorName = TEST_ONNX_PROVIDER_NAME;
    const std::string ModelPath = TEST_ONNX_MODEL_PATH;

    Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "onnx_hello_world");
    Ort::SessionOptions session_options;
    session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);

    std::shared_ptr<Ort::Session> session;
    try {
        spdlog::info("Creating session");
        session = std::make_shared<Ort::Session>(env, ModelPath.c_str(), session_options);
    } catch (const Ort::Exception &e) {
        spdlog::error("Error creating session: {}", e.what());
        return 1;
    }

    auto ports = list_onnx_model_info(*session);
    spdlog::info("Listing all input and output ports:");
    for (const auto &port : ports) {
        spdlog::info("Port: {}", port.to_string());
    }

    Ort::IoBinding io_binding(*session);

    // Bind input tensors
    std::map<OnnxPortInfo, OnnxPortData> tensor_caches;
    for (const auto &port : ports) {
        if (port.is_input) {
            spdlog::info("Creating input tensor for port: {}; Port shape: {}", port.name, fmt::join(port.shape, ", "));
            auto input_tensor_values = std::make_shared<std::vector<float>>(port.shape[0] * port.shape[1] * port.shape[2] * port.shape[3], 0.5f);
            auto memory_info = std::make_shared<Ort::MemoryInfo>(Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault));
            auto input_tensor = std::make_shared<Ort::Value>(Ort::Value::CreateTensor<float>(*memory_info,
                                                                                             input_tensor_values->data(),
                                                                                             input_tensor_values->size(),
                                                                                             port.shape.data(),
                                                                                             port.shape.size()));

            OnnxPortData port_data{input_tensor_values, input_tensor, memory_info};
            tensor_caches[port] = std::move(port_data);

            spdlog::info("Binding input tensor for port: {}", port.name);
            io_binding.BindInput(port.name.c_str(), *input_tensor);
        }
    }

    // Bind output tensors
    spdlog::info("Binding output tensors");
    for (const auto &port : ports) {
        if (!port.is_input) {
            std::shared_ptr<Ort::MemoryInfo> output_mem_info = std::make_shared<Ort::MemoryInfo>(
                Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault));
            spdlog::info("Creating output tensor for port: {}; Port shape: {}", port.name, fmt::join(port.shape, ", "));
            auto output_tensor_values = std::make_shared<std::vector<float>>(port.shape[0] * port.shape[1] * port.shape[2] * port.shape[3], 0.0f);
            auto output_tensor = std::make_shared<Ort::Value>(Ort::Value::CreateTensor<float>(*output_mem_info,
                                                                                              output_tensor_values->data(),
                                                                                              output_tensor_values->size(),
                                                                                              port.shape.data(),
                                                                                              port.shape.size()));
            OnnxPortData port_data{output_tensor_values, output_tensor, output_mem_info};
            tensor_caches[port] = std::move(port_data);

            spdlog::info("Binding output tensor for port: {}", port.name);
            io_binding.BindOutput(port.name.c_str(), *output_tensor);
        }
    }

    // Run 100 iterations of inference with I/O binding
    for (int iteration = 0; iteration < 1000; ++iteration) {
        spdlog::info("Running inference iteration {}", iteration + 1);

        // Update input tensors with random data
        for (auto &entry : tensor_caches) {
            if (entry.first.is_input) {
                auto &input_tensor_values = entry.second.data;
                std::generate(input_tensor_values->begin(), input_tensor_values->end(), []() { return static_cast<float>(rand()) / RAND_MAX; });
            }
        }

        auto start_time = std::chrono::high_resolution_clock::now();
        try {
            session->Run(Ort::RunOptions{nullptr}, io_binding);
        } catch (const Ort::Exception &e) {
            spdlog::error("ONNX Runtime error: {}", e.what());
            return 1;
        }
        auto end_time = std::chrono::high_resolution_clock::now();
        auto inference_duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
        spdlog::info("\033[33mInference iteration {} took {} ms\033[0m", iteration + 1, inference_duration);

        // Retrieve and process outputs
        spdlog::info("Retrieving outputs for iteration {}", iteration + 1);
        auto output_tensors = io_binding.GetOutputValues();
        for (size_t i = 0; i < output_tensors.size(); ++i) {
            auto &output_tensor = output_tensors[i];
            float *floatarr = output_tensor.GetTensorMutableData<float>();
            size_t num_elements = output_tensor.GetTensorTypeAndShapeInfo().GetElementCount();
            float sum = std::accumulate(floatarr, floatarr + num_elements, 0.0f);
            float mean = sum / num_elements;
            spdlog::info("Output {}: Mean value = {}", i, mean);
        }
    }

    return 0;
}

std::vector<std::string> list_onnx_providers()
{
    auto providers = Ort::GetAvailableProviders();
    for (const auto &provider : providers) {
        spdlog::info("Provider: {}", provider);
    }
    return providers;
}

std::vector<OnnxPortInfo> list_onnx_model_info(const Ort::Session &session)
{
    std::vector<OnnxPortInfo> ports;
    size_t num_input_nodes = session.GetInputCount();

    for (size_t i = 0; i < num_input_nodes; ++i) {
        OnnxPortInfo port_info;
        auto input_name = session.GetInputNameAllocated(i, Ort::AllocatorWithDefaultOptions());
        port_info.name = input_name.get();
        port_info.is_input = true;

        auto input_type_info = session.GetInputTypeInfo(i);
        auto input_tensor_info = input_type_info.GetTensorTypeAndShapeInfo();
        port_info.dtype = input_tensor_info.GetElementType();
        port_info.dtype_str = onnx_element_type_to_string(port_info.dtype);

        auto input_shape = input_tensor_info.GetShape();
        port_info.shape = input_shape;

        ports.push_back(port_info);
    }

    size_t num_output_nodes = session.GetOutputCount();

    for (size_t i = 0; i < num_output_nodes; ++i) {
        OnnxPortInfo port_info;
        auto output_name = session.GetOutputNameAllocated(i, Ort::AllocatorWithDefaultOptions());
        port_info.name = output_name.get();
        port_info.is_input = false;

        auto output_type_info = session.GetOutputTypeInfo(i);
        auto output_tensor_info = output_type_info.GetTensorTypeAndShapeInfo();
        port_info.dtype = output_tensor_info.GetElementType();
        port_info.dtype_str = onnx_element_type_to_string(port_info.dtype);

        auto output_shape = output_tensor_info.GetShape();
        port_info.shape = output_shape;

        ports.push_back(port_info);
    }
    return ports;
}

std::string onnx_element_type_to_string(ONNXTensorElementDataType dtype)
{
    switch (dtype) {
        case ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT:
            return "float";
        case ONNX_TENSOR_ELEMENT_DATA_TYPE_UINT8:
            return "uint8";
        case ONNX_TENSOR_ELEMENT_DATA_TYPE_INT8:
            return "int8";
        case ONNX_TENSOR_ELEMENT_DATA_TYPE_UINT16:
            return "uint16";
        case ONNX_TENSOR_ELEMENT_DATA_TYPE_INT16:
            return "int16";
        case ONNX_TENSOR_ELEMENT_DATA_TYPE_INT32:
            return "int32";
        case ONNX_TENSOR_ELEMENT_DATA_TYPE_INT64:
            return "int64";
        case ONNX_TENSOR_ELEMENT_DATA_TYPE_STRING:
            return "string";
        case ONNX_TENSOR_ELEMENT_DATA_TYPE_BOOL:
            return "bool";
        case ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT16:
            return "float16";
        case ONNX_TENSOR_ELEMENT_DATA_TYPE_DOUBLE:
            return "double";
        case ONNX_TENSOR_ELEMENT_DATA_TYPE_UINT32:
            return "uint32";
        case ONNX_TENSOR_ELEMENT_DATA_TYPE_UINT64:
            return "uint64";
        case ONNX_TENSOR_ELEMENT_DATA_TYPE_COMPLEX64:
            return "complex64";
        case ONNX_TENSOR_ELEMENT_DATA_TYPE_COMPLEX128:
            return "complex128";
        case ONNX_TENSOR_ELEMENT_DATA_TYPE_BFLOAT16:
            return "bfloat16";
        default:
            return "unknown";
    }
}