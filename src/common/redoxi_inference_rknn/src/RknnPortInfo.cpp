#include <redoxi_inference_rknn/RknnPortInfo.hpp>
#include <redoxi_inference_rknn/rknn_utils.hpp>

namespace redoxi_works::inference::rknn
{
std::vector<int64_t> RknnModelPortInfo::get_current_shape() const
{
    auto attr = get_current_tensor_attributes();
    return std::vector<int64_t>(attr.dims, attr.dims + attr.n_dims);
}

std::vector<int64_t> RknnModelPortInfo::get_native_shape() const
{
    auto attr = get_native_tensor_attributes();
    return std::vector<int64_t>(attr.dims, attr.dims + attr.n_dims);
}

std::vector<int64_t> RknnModelPortInfo::get_default_shape() const
{
    auto attr = get_default_tensor_attributes();
    return std::vector<int64_t>(attr.dims, attr.dims + attr.n_dims);
}

std::string RknnModelPortInfo::get_intrinsic_dtype_str() const
{
    auto attr = get_default_tensor_attributes();
    return std::string(rknn_tensor_type_to_string(attr.type));
}

RknnModelPortInfo::TensorAttributes_t RknnModelPortInfo::get_native_tensor_attributes() const
{
    if (context == nullptr || *context == 0) {
        RDX_RAISE_ERROR("[f={}()] Invalid context", __func__);
    }

    rknn_tensor_attr attr;
    memset(&attr, 0, sizeof(rknn_tensor_attr));
    attr.index = m_index;
    if (m_is_input) {
        rknn_query(*context, RKNN_QUERY_NATIVE_INPUT_ATTR, &attr, sizeof(attr));
    } else {
        rknn_query(*context, RKNN_QUERY_NATIVE_OUTPUT_ATTR, &attr, sizeof(attr));
    }
    return attr;
}

RknnModelPortInfo::TensorAttributes_t RknnModelPortInfo::get_current_tensor_attributes() const
{
    if (context == nullptr || *context == 0) {
        RDX_RAISE_ERROR("[f={}()] Invalid context", __func__);
    }

    rknn_tensor_attr attr;
    memset(&attr, 0, sizeof(rknn_tensor_attr));
    attr.index = m_index;

    if (m_is_input) {
        rknn_query(*context, RKNN_QUERY_CURRENT_INPUT_ATTR, &attr, sizeof(attr));
    } else {
        rknn_query(*context, RKNN_QUERY_CURRENT_OUTPUT_ATTR, &attr, sizeof(attr));
    }
    return attr;
}

RknnModelPortInfo::TensorAttributes_t RknnModelPortInfo::get_default_tensor_attributes() const
{
    if (context == nullptr || *context == 0) {
        RDX_RAISE_ERROR("[f={}()] Invalid context", __func__);
    }

    rknn_tensor_attr attr;
    memset(&attr, 0, sizeof(rknn_tensor_attr));
    attr.index = m_index;
    if (m_is_input) {
        rknn_query(*context, RKNN_QUERY_INPUT_ATTR, &attr, sizeof(attr));
    } else {
        rknn_query(*context, RKNN_QUERY_OUTPUT_ATTR, &attr, sizeof(attr));
    }
    return attr;
}
} // namespace redoxi_works::inference::rknn
