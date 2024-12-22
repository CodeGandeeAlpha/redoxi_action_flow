#include <redoxi_inference_rknn/RknnModelInference.hpp>
#include <redoxi_basic_cpp/logging/ros_logging.hpp>
#include <redoxi_inference_rknn/rknn_utils.hpp>

namespace redoxi_works::inference::rknn
{
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
    (void)inout_data;
    throw std::runtime_error("Not implemented");
}

int RknnModelInference::open(KeyValueStore::Ptr params)
{
    (void)params;
    throw std::runtime_error("Not implemented");
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
                         init_config.init_flags, init_config.init_ext.get());
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
