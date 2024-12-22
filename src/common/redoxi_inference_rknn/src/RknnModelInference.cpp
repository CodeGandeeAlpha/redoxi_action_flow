#include <redoxi_inference_rknn/RknnModelInference.hpp>
#include <redoxi_basic_cpp/logging/ros_logging.hpp>
#include <redoxi_inference_rknn/rknn_utils.hpp>
#include <filesystem>
#include <algorithm>
#include <numeric>

namespace redoxi_works::inference::rknn
{

uint32_t InferenceContextInitConfig::to_init_flags() const
{
    uint32_t flags = 0;
    if (rkopt_collect_performance_stats) {
        flags |= RKNN_FLAG_COLLECT_PERF_MASK;
    }
    if (rkopt_allocate_memory_outside) {
        flags |= RKNN_FLAG_MEM_ALLOC_OUTSIDE;
    }
    if (rkopt_shared_weight) {
        flags |= RKNN_FLAG_SHARE_WEIGHT_MEM;
    }
    if (rkopt_load_model_info_only) {
        flags |= RKNN_FLAG_COLLECT_MODEL_INFO_ONLY;
    }
    if (rkopt_use_gpu_for_unsupported_npu_ops) {
        flags |= RKNN_FLAG_EXECUTE_FALLBACK_PRIOR_DEVICE_GPU;
    }
    if (rkopt_internel_tensor_use_sram) {
        flags |= RKNN_FLAG_ENABLE_SRAM;
    }
    if (rkopt_internal_tensor_share_sram) {
        flags |= RKNN_FLAG_SHARE_SRAM;
    }
    if (rkopt_manual_flush_input_cache) {
        flags |= RKNN_FLAG_DISABLE_FLUSH_INPUT_MEM_CACHE;
    }
    if (rkopt_manual_flush_output_cache) {
        flags |= RKNN_FLAG_DISABLE_FLUSH_OUTPUT_MEM_CACHE;
    }
    if (rkopt_allow_allocate_device_memory_without_context) {
        flags |= RKNN_MEM_FLAG_ALLOC_NO_CONTEXT;
    }
    flags |= _more_init_flags;
    return flags;
}

RknnModelInference::KeyValueStore_t::Ptr RknnModelInference::create_init_params()
{
    return std::make_shared<RknnModelConfig>();
}

RknnModelInference::InOutData_t::Ptr RknnModelInference::create_inference_inout_data()
{
    return std::make_shared<RknnInferenceInOutData>();
}

RknnModelInference::KeyValueStore_t::ConstPtr RknnModelInference::get_model_metadata() const
{
    return m_config;
}

RknnModelInference::KeyValueStore_t::ConstPtr RknnModelInference::get_inference_metadata() const
{
    return m_config;
}


RknnModelInference::ModelPortInfo_t::ConstPtrMap RknnModelInference::get_input_port_infos() const
{
    ModelPortInfo_t::ConstPtrMap input_ports;
    for (const auto &port : m_input_ports) {
        input_ports.insert(port);
    }
    return input_ports;
}

RknnModelInference::ModelPortInfo_t::PtrMap RknnModelInference::_get_input_port_infos()
{
    RknnModelInference::ModelPortInfo_t::PtrMap output;
    for (const auto &port : m_input_ports) {
        output.insert(port);
    }
    return output;
}

RknnModelInference::ModelPortInfo_t::ConstPtrMap RknnModelInference::get_output_port_infos() const
{
    ModelPortInfo_t::ConstPtrMap output_ports;
    for (const auto &port : m_output_ports) {
        output_ports.insert(port);
    }
    return output_ports;
}

RknnModelInference::ModelPortInfo_t::PtrMap RknnModelInference::_get_output_port_infos()
{
    RknnModelInference::ModelPortInfo_t::PtrMap output;
    for (const auto &port : m_output_ports) {
        output.insert(port);
    }
    return output;
}

int RknnModelInference::do_inference(InferenceInOutData::Ptr inout_data)
{
    // auto _inout_data = std::dynamic_pointer_cast<RknnInferenceInOutData>(inout_data);
    auto num_inputs = m_input_ports.size();

    // create rknn input structures
    std::vector<rknn_input> inputs(num_inputs);
    size_t input_index = 0;
    for (auto it = m_input_ports.begin(); it != m_input_ports.end(); ++it, ++input_index) {

        // TODO: now only supports uint8 HWC images
        inputs[input_index].type = RKNN_TENSOR_UINT8;
        inputs[input_index].fmt = RKNN_TENSOR_NHWC;

        auto port_info = it->second;
        inputs[input_index].index = port_info->get_index();
        auto port_data = inout_data->get_input_port_data(port_info->get_name());
        uint8_t *data = nullptr;
        auto ret = port_data->get_tensor_data(&data);
        if (ret != 0) {
            RDX_RAISE_ERROR("[f={}] Failed to get tensor data for port: {}", __func__, port_info->get_name());
            return ret;
        }
        size_t data_size = std::accumulate(port_data->get_shape().begin(), port_data->get_shape().end(), 1, std::multiplies<int64_t>());

        inputs[input_index].buf = data;
        inputs[input_index].size = data_size;
    }

    // make sure we have a valid context
}

int RknnModelInference::open(KeyValueStore::Ptr params)
{
    auto config = std::dynamic_pointer_cast<RknnModelConfig>(params);
    if (config == nullptr) {
        // wrong type of params
        return -1;
    }

    auto model_path = config->model_path;

    // check if the model path is valid
    if (model_path.empty()) {
        RDX_RAISE_ERROR("[f={}] Model path is empty", __func__);
        return -1;
    }
    if (!std::filesystem::exists(model_path)) {
        RDX_RAISE_ERROR("[f={}] Model path does not exist: {}", __func__, model_path);
        return -1;
    }

    auto device_index = config->device_index;
    auto core_mask = config->core_mask;
    // create the environment and session
    InferenceContextInitConfig infer_config;
    m_context = create_inference_context(model_path, infer_config);

    // set cores
    {
        int ret = 0;
        if (device_index == std::numeric_limits<int64_t>::max()) {
            ret = rknn_set_core_mask(*m_context, rknn_core_mask::RKNN_NPU_CORE_AUTO);
        } else {
            if (core_mask == 1 << 0) {
                ret = rknn_set_core_mask(*m_context, rknn_core_mask::RKNN_NPU_CORE_0);
            } else if (core_mask == 1 << 1) {
                ret = rknn_set_core_mask(*m_context, rknn_core_mask::RKNN_NPU_CORE_1);
            } else if (core_mask == 1 << 2) {
                ret = rknn_set_core_mask(*m_context, rknn_core_mask::RKNN_NPU_CORE_2);
            } else if (core_mask == ((1 << 1) | (1 << 2))) {
                ret = rknn_set_core_mask(*m_context, rknn_core_mask::RKNN_NPU_CORE_0_1);
            } else if (core_mask == ((1 << 1) | (1 << 2) | (1 << 3))) {
                ret = rknn_set_core_mask(*m_context, rknn_core_mask::RKNN_NPU_CORE_0_1_2);
            } else {
                RDX_RAISE_ERROR("[f={}] Invalid core mask: {}", __func__, core_mask);
                return -1;
            }
        }
        HANDLE_RKNN_ERROR(ret);
    }

    // get all input and output port infos
    m_input_ports = get_input_port_infos(*m_context);
    RDX_INFO_DEV(nullptr, __func__, false, "Got {} input ports", m_input_ports.size());
    {
        for (const auto &[port_name, port_info] : m_input_ports) {
            RDX_INFO_DEV(nullptr, __func__, false, "Input Port: {}", port_info->to_description());
        }
    }

    m_output_ports = get_output_port_infos(*m_context);
    RDX_INFO_DEV(nullptr, __func__, false, "Got {} output ports", m_output_ports.size());
    {
        for (const auto &[port_name, port_info] : m_output_ports) {
            RDX_INFO_DEV(nullptr, __func__, false, "Output Port: {}", port_info->to_description());
        }
    }

    RDX_INFO_DEV(nullptr, __func__, false, "{}", "Initialization completed");
    m_config = config;
    return 0;
}

bool RknnModelInference::is_open() const
{
    return m_context != nullptr;
}

int RknnModelInference::close()
{
    m_context.reset();
    return 0;
}

std::shared_ptr<RknnModelInference::InferenceContext_t> RknnModelInference::create_inference_context(
    const std::string &model_path,
    const InferenceContextInitConfig &init_config)
{
    auto ctx = std::shared_ptr<InferenceContext_t>(
        new InferenceContext_t(0),
        [](InferenceContext_t *context) {
            if (context && *context) {
                //! Destroy the rknn context to release resources if it is not null and non zero
                rknn_destroy(*context);
            }
            delete context;
        });

    // load model
    auto ret = rknn_init(ctx.get(), (void *)model_path.c_str(), 0,
                         init_config.to_init_flags(), init_config.extended_init_info.get());
    HANDLE_RKNN_ERROR(ret);
    return ctx;
}

RknnModelPortInfo::PtrMap RknnModelInference::get_input_port_infos(
    const InferenceContext_t &context)
{
    RknnModelPortInfo::PtrMap input_ports;
    rknn_input_output_num io_num;
    rknn_query(context, RKNN_QUERY_IN_OUT_NUM, &io_num, sizeof(io_num));

    std::vector<rknn_tensor_attr> input_attrs(io_num.n_input);
    for (uint32_t i = 0; i < io_num.n_input; i++) {
        rknn_query(context, RKNN_QUERY_INPUT_ATTR, &input_attrs[i], sizeof(rknn_tensor_attr));
        auto &attr = input_attrs[i];
        auto port_info = std::make_shared<RknnModelPortInfo>();

        port_info->m_name = attr.name;
        port_info->m_dtype = attr.type;
        port_info->m_dtype_str = rknn_tensor_type_to_string(attr.type);
        port_info->m_shape = std::vector<int64_t>(attr.dims, attr.dims + attr.n_dims);
        port_info->m_is_input = true;
        port_info->m_index = attr.index;
        port_info->m_rknn_attr = attr;
        input_ports.insert(std::make_pair(port_info->m_name, port_info));
    }
    return input_ports;
}

RknnModelPortInfo::PtrMap RknnModelInference::get_output_port_infos(
    const InferenceContext_t &context)
{
    RknnModelPortInfo::PtrMap output_ports;
    rknn_input_output_num io_num;
    rknn_query(context, RKNN_QUERY_IN_OUT_NUM, &io_num, sizeof(io_num));

    std::vector<rknn_tensor_attr> output_attrs(io_num.n_output);
    for (uint32_t i = 0; i < io_num.n_output; i++) {
        rknn_query(context, RKNN_QUERY_OUTPUT_ATTR, &output_attrs[i], sizeof(rknn_tensor_attr));
        auto &attr = output_attrs[i];
        auto port_info = std::make_shared<RknnModelPortInfo>();

        port_info->m_name = attr.name;
        port_info->m_dtype = attr.type;
        port_info->m_dtype_str = rknn_tensor_type_to_string(attr.type);
        port_info->m_shape = std::vector<int64_t>(attr.dims, attr.dims + attr.n_dims);
        port_info->m_is_input = false;
        port_info->m_index = attr.index;
        port_info->m_rknn_attr = attr;
        output_ports.insert(std::make_pair(port_info->m_name, port_info));
    }
    return output_ports;
}

std::string RknnModelInference::rknn_tensor_type_to_string(RknnTensorType_t dtype)
{
    switch (dtype) {
        case RKNN_TENSOR_FLOAT32:
            return "float32";
        case RKNN_TENSOR_FLOAT16:
            return "float16";
        case RKNN_TENSOR_INT8:
            return "int8";
        case RKNN_TENSOR_UINT8:
            return "uint8";
        case RKNN_TENSOR_INT16:
            return "int16";
        case RKNN_TENSOR_UINT16:
            return "uint16";
        case RKNN_TENSOR_INT32:
            return "int32";
        case RKNN_TENSOR_UINT32:
            return "uint32";
        case RKNN_TENSOR_INT64:
            return "int64";
        case RKNN_TENSOR_BOOL:
            return "bool";
        default:
            return "unknown";
    }
}

} // namespace redoxi_works::inference::rknn
