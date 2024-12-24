#pragma once

#include <rknn_api.h>
#include <redoxi_basic_cpp/logging/ros_logging.hpp>
#include <string_view>

namespace redoxi_works::inference::rknn
{

// check the error code, if it is an error, log it and raise error
#define HANDLE_RKNN_ERROR(ret)                                                                                      \
    do {                                                                                                            \
        switch (ret) {                                                                                              \
            case RKNN_SUCC:                                                                                         \
                break;                                                                                              \
            case RKNN_ERR_FAIL:                                                                                     \
                RDX_RAISE_ERROR("[f={}()] Unknown error, error code: {}", __func__, ret);                           \
            case RKNN_ERR_TIMEOUT:                                                                                  \
                RDX_INFO_DEV(nullptr, __func__, "Timeout error, error code: {}", ret);                              \
            case RKNN_ERR_DEVICE_UNAVAILABLE:                                                                       \
                RDX_RAISE_ERROR("[f={}()] Device unavailable, error code: {}", __func__, ret);                      \
            case RKNN_ERR_PARAM_INVALID:                                                                            \
                RDX_RAISE_ERROR("[f={}()] Invalid parameter, error code: {}", __func__, ret);                       \
            case RKNN_ERR_MODEL_INVALID:                                                                            \
                RDX_RAISE_ERROR("[f={}()] Invalid model, error code: {}", __func__, ret);                           \
            case RKNN_ERR_CTX_INVALID:                                                                              \
                RDX_RAISE_ERROR("[f={}()] Invalid context, error code: {}", __func__, ret);                         \
            case RKNN_ERR_INPUT_INVALID:                                                                            \
                RDX_RAISE_ERROR("[f={}()] Invalid model input, error code: {}", __func__, ret);                     \
            case RKNN_ERR_OUTPUT_INVALID:                                                                           \
                RDX_RAISE_ERROR("[f={}()] Invalid model output, error code: {}", __func__, ret);                    \
            case RKNN_ERR_DEVICE_UNMATCH:                                                                           \
                RDX_RAISE_ERROR("[f={}()] Device unmatch, error code: {}", __func__, ret);                          \
            case RKNN_ERR_INCOMPATILE_OPTIMIZATION_LEVEL_VERSION:                                                   \
                RDX_RAISE_ERROR("[f={}()] Incompatible optimization level version, error code: {}", __func__, ret); \
            case RKNN_ERR_TARGET_PLATFORM_UNMATCH:                                                                  \
                RDX_RAISE_ERROR("[f={}()] Target platform unmatch, error code: {}", __func__, ret);                 \
            default:                                                                                                \
                RDX_RAISE_ERROR("[f={}()] Unknown error, error code: {}", __func__, ret);                           \
        }                                                                                                           \
    } while (0)

inline constexpr std::string_view rknn_tensor_type_to_string(rknn_tensor_type dtype)
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
