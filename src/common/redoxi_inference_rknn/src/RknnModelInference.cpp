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
    auto output = std::make_shared<RknnInferenceInOutData>();
    output->init(this);
    return output;
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
    // make sure we have a valid context
    if (m_context == nullptr || *m_context == 0) {
        RDX_RAISE_ERROR("[f={}()] RKNN Model is not initialized (no context)", __func__);
        return -1;
    }

    // auto _inout_data = std::dynamic_pointer_cast<RknnInferenceInOutData>(inout_data);
    auto num_inputs = m_input_ports.size();

    // create rknn input structures
    RDX_INFO_DEV(nullptr, __func__, false, "Filling {} inputs into context", num_inputs);
    std::vector<rknn_input> inputs(num_inputs);
    for (auto it = m_input_ports.begin(); it != m_input_ports.end(); ++it) {
        auto input_index = it->second->get_index();
        RDX_INFO_DEV(nullptr, __func__, false, "Filling input[{}] into context, name={}", input_index, it->second->get_name());

        // clear the input structure with memset
        auto &inp = inputs[input_index];
        memset(&inp, 0, sizeof(rknn_input));

        auto port_info = it->second;
        inp.index = port_info->get_index();
        auto port_data = inout_data->get_input_port_data(port_info->get_name());
        auto shape = port_data->get_shape();
        auto num_elements = std::accumulate(shape.begin(), shape.end(), 1, std::multiplies<int64_t>());

        // negative num_elements is not allowed
        if (num_elements < 0) {
            RDX_RAISE_ERROR("[f={}()] Negative num_elements is not allowed", __func__);
            return -1;
        }

        if (port_data->get_dtype_str() == "uint8") {
            // uint8 case, you do not need to process the image in this case
            // assuming the image is in (N)HWC format
            RDX_INFO_DEV(nullptr, __func__, false, "Setting input[{}] type={}",
                         input_index, port_data->get_dtype_str());
            inp.type = RKNN_TENSOR_UINT8;
            inp.fmt = RKNN_TENSOR_NHWC;
            uint8_t *data = nullptr;
            auto ret = port_data->get_tensor_data(&data);
            if (ret != 0) {
                RDX_RAISE_ERROR("[f={}()] Failed to get tensor data for port: {}", __func__, port_info->get_name());
                return ret;
            }
            size_t num_bytes = num_elements * sizeof(uint8_t);
            inp.buf = data;
            inp.size = num_bytes;
        } else if (port_data->get_dtype_str() == "float32") {
            // float32 case, you need to process the image in this case by scaling the image to [0, 1]
            // assuming the image is in (N)HWC format
            RDX_INFO_DEV(nullptr, __func__, false, "Setting input[{}] type={}",
                         input_index, port_data->get_dtype_str());
            inp.type = RKNN_TENSOR_FLOAT32;
            inp.fmt = RKNN_TENSOR_NCHW;
            float *data = nullptr;
            auto ret = port_data->get_tensor_data(&data);
            if (ret != 0) {
                RDX_RAISE_ERROR("[f={}()] Failed to get tensor data for port: {}", __func__, port_info->get_name());
                return ret;
            }
            auto num_bytes = num_elements * sizeof(float);
            inp.buf = data;
            inp.size = num_bytes;
        } else {
            RDX_RAISE_ERROR("[f={}()] Unsupported data type: {}", __func__, port_data->get_dtype_str());
            return -1;
        }
    }

    // set input into context
    RDX_INFO_DEV(nullptr, __func__, false, "Setting {} inputs into context", inputs.size());
    {
        auto ret = rknn_inputs_set(*m_context, inputs.size(), inputs.data());
        HANDLE_RKNN_ERROR(ret);
    }

    // run inference
    RDX_INFO_DEV(nullptr, __func__, false, "{}", "Running inference");
    {
        auto ret = rknn_run(*m_context, nullptr);
        HANDLE_RKNN_ERROR(ret);
    }

    // gets output
    RDX_INFO_DEV(nullptr, __func__, false, "Getting {} outputs from context", m_output_ports.size());
    auto num_outputs = m_output_ports.size();
    std::vector<rknn_output> outputs(num_outputs);
    {
        for (auto &out : outputs) {
            memset(&out, 0, sizeof(rknn_output));
            out.want_float = 1; // we want float output
        }
        auto ret = rknn_outputs_get(*m_context, outputs.size(), outputs.data(), nullptr);
        HANDLE_RKNN_ERROR(ret);
    }

    // copy the outputs to the inout data
    RDX_INFO_DEV(nullptr, __func__, false, "Copying {} outputs to inout data", num_outputs);
    for (const auto &port_pair : m_output_ports) {
        auto port_info = port_pair.second;
        auto port_name = port_pair.first;
        auto port_index = port_info->get_index();

        RDX_INFO_DEV(nullptr, __func__, false, "Copying output with index {}, port name={}, size={}",
                     port_index, port_name, outputs[port_index].size);

        auto port_data = inout_data->get_output_port_data(port_name);
        auto shape = port_data->get_shape();
        {
            // sanity check
            auto num_expected_output_elements = std::accumulate(shape.begin(), shape.end(), 1, std::multiplies<int64_t>());
            if (outputs[port_index].size != num_expected_output_elements * sizeof(float)) {
                RDX_RAISE_ERROR("[f={}()] Output size mismatch for port: {}", __func__, port_name);
                return -1;
            }
        }
        port_data->set_tensor_data((float *)outputs[port_index].buf, shape);
    }

    // release the outputs
    rknn_outputs_release(*m_context, outputs.size(), outputs.data());
    return 0;
}

int RknnModelInference::open(KeyValueStore::Ptr params)
{
    auto config = std::dynamic_pointer_cast<RknnModelConfig>(params);
    if (config == nullptr) {
        // wrong type of params
        return -1;
    }

    auto model_path = config->model_path;

    // we can do this because rknn context is uint64_t
    auto duplicate_context = (InferenceContext_t)config->duplicate_context;

    InferenceContextInitConfig infer_config;
    if (duplicate_context == 0) {
        // check if the model path is valid
        if (model_path.empty()) {
            RDX_RAISE_ERROR("[f={}()] Model path is empty", __func__);
            return -1;
        }
        if (!std::filesystem::exists(model_path)) {
            RDX_RAISE_ERROR("[f={}()] Model path does not exist: {}", __func__, model_path);
            return -1;
        }
        m_context = create_inference_context(model_path, infer_config);
    } else {
        // TODO: weight sharing by context duplication is not yet tested
        // duplicate the context
        m_context = create_inference_context(duplicate_context);
    }

    // set cores
    auto device_index = config->device_index;
    auto core_mask = config->core_mask;
    {
        int ret = 0;
        if (device_index == RknnModelConfig::DeviceIndexUseAnyCore) {
            ret = rknn_set_core_mask(*m_context, rknn_core_mask::RKNN_NPU_CORE_AUTO);
        } else if (device_index == RknnModelConfig::DeviceIndexUseAllCores) {
            ret = rknn_set_core_mask(*m_context, rknn_core_mask::RKNN_NPU_CORE_ALL);
        } else if (device_index == RknnModelConfig::DeviceIndexUseCoreMask) {
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
                RDX_RAISE_ERROR("[f={}()] Invalid core mask: {}", __func__, core_mask);
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

    // TODO: handle dynamic input shape
    // for now, just fix the shape for dynamic input ports
    {
        for (const auto &[port_name, port_info] : m_input_ports) {
            auto attr = port_info->get_default_tensor_attributes();
            RDX_INFO_DEV(nullptr, __func__, false, "Setting input[{}] to fixed shape=({})", port_info->get_index(),
                         fmt::join(std::vector<int64_t>(attr.dims, attr.dims + attr.n_dims), ","));
            rknn_set_input_shape(*m_context, &attr);
        }
    }

    m_output_ports = get_output_port_infos(*m_context);
    RDX_INFO_DEV(nullptr, __func__, false, "Got {} output ports", m_output_ports.size());
    {
        for (const auto &[port_name, port_info] : m_output_ports) {
            RDX_INFO_DEV(nullptr, __func__, false, "Output Port: {}", port_info->to_description());
        }
    }

    // FIXME: this is not correct for rknn, because rknn enumerates all the supported shapes,
    // rather than setting -1 to indicate dynamic shape
    // we do not allow dynamic input shapes (-1 in dimensions)
    // for (const auto &port : m_input_ports) {
    //     auto shape = port.second->get_shape();
    //     if (std::find(shape.begin(), shape.end(), -1) != shape.end()) {
    //         RDX_RAISE_ERROR("[f={}()] Dynamic input shape is not allowed for port: {}", __func__, port.first);
    //         return -1;
    //     }
    // }

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
    InferenceContext_t &duplicate_context)
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
    auto ret = rknn_dup_context(&duplicate_context, ctx.get());
    HANDLE_RKNN_ERROR(ret);
    return ctx;
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
    auto init_flags = init_config.to_init_flags();
    RDX_INFO_DEV(nullptr, __func__, "opening model with init_flags={}", init_flags);
    auto ret = rknn_init(ctx.get(), (void *)model_path.c_str(), 0,
                         init_flags, init_config.extended_init_info.get());
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
    RDX_INFO_DEV(nullptr, __func__, false, "found {} inputs", io_num.n_input);
    for (uint32_t i = 0; i < io_num.n_input; i++) {
        auto &attr = input_attrs[i];
        memset(&attr, 0, sizeof(rknn_tensor_attr));
        attr.index = i;

        rknn_query(context, RKNN_QUERY_INPUT_ATTR, &input_attrs[i], sizeof(rknn_tensor_attr));

        auto port_info = std::make_shared<RknnModelPortInfo>();
        port_info->m_name = attr.name;

        // rknn will convert the type to appropriate type, so the input type on user side can be whatever
        port_info->m_dtype = RknnModelInference::DefaultInputTensorType;
        port_info->m_dtype_str = rknn_tensor_type_to_string(RknnModelInference::DefaultInputTensorType);

        port_info->m_shape = std::vector<int64_t>(attr.dims, attr.dims + attr.n_dims);
        port_info->m_is_input = true;
        port_info->m_index = attr.index;
        port_info->context = &context;

        {
            // handle dynamic input range, you need to query for them explicitly
            rknn_input_range input_range;
            memset(&input_range, 0, sizeof(rknn_input_range));
            input_range.index = i;
            rknn_query(context, RKNN_QUERY_INPUT_DYNAMIC_RANGE, &input_range, sizeof(rknn_input_range));

            for (uint32_t k = 0; k < input_range.shape_number; k++) {
                std::vector<int64_t> dyn_shape(input_range.dyn_range[k], input_range.dyn_range[k] + input_range.n_dims);
                RDX_INFO_DEV(nullptr, __func__, false, "supported shape[{}]=({})", k, fmt::join(dyn_shape, ","));
                port_info->supported_shapes.push_back(dyn_shape);
            }
        }

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
    RDX_INFO_DEV(nullptr, __func__, false, "found {} outputs", io_num.n_output);
    for (uint32_t i = 0; i < io_num.n_output; i++) {
        // RDX_INFO_DEV(nullptr, __func__, false, "reading meta info for output[{}]", i);
        auto &attr = output_attrs[i];
        memset(&attr, 0, sizeof(rknn_tensor_attr));
        attr.index = i;

        rknn_query(context, RKNN_QUERY_OUTPUT_ATTR, &attr, sizeof(rknn_tensor_attr));
        auto port_info = std::make_shared<RknnModelPortInfo>();

        port_info->m_name = attr.name;

        // for output, rknn always returns float32
        port_info->m_dtype = RknnModelPortInfo::DataType_t::RKNN_TENSOR_FLOAT32;
        port_info->m_dtype_str = "float32";

        port_info->m_shape = std::vector<int64_t>(attr.dims, attr.dims + attr.n_dims);
        port_info->m_is_input = false;
        port_info->m_index = attr.index;

        output_ports.insert(std::make_pair(port_info->m_name, port_info));
    }
    return output_ports;
}


} // namespace redoxi_works::inference::rknn

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
    redoxi_works::inference::rknn::RknnModelInference,
    redoxi_works::inference::RedoxiModelInference)

PLUGINLIB_EXPORT_CLASS(
    redoxi_works::inference::rknn::RknnModelConfig,
    redoxi_works::inference::KeyValueStore)

PLUGINLIB_EXPORT_CLASS(
    redoxi_works::inference::rknn::RknnPortData,
    redoxi_works::inference::ModelPortData)

PLUGINLIB_EXPORT_CLASS(
    redoxi_works::inference::rknn::RknnModelPortInfo,
    redoxi_works::inference::ModelPortInfo)

PLUGINLIB_EXPORT_CLASS(
    redoxi_works::inference::rknn::RknnInferenceInOutData,
    redoxi_works::inference::InferenceInOutData)