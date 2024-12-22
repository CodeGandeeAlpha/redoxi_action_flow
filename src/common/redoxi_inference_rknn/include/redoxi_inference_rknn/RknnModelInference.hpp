#pragma once

#include <redoxi_inference_rknn/RknnModelConfig.hpp>
#include <redoxi_inference_rknn/RknnPortInfo.hpp>
#include <redoxi_inference_rknn/RknnInferenceInOutData.hpp>
#include <rknn_api.h>

namespace redoxi_works::inference::rknn
{

/**
 * @brief Configuration for initializing the RKNN inference context
 */
struct InferenceContextInitConfig {
    //! NPU cores to use, default enables all cores (0,1,2)
    std::vector<int> use_npu_cores = {0, 1, 2};

    //! @name RKNN API Options
    //! @{

    //! Whether to collect performance stats during inference
    bool rkopt_collect_performance_stats = false;

    //! Whether to allow users to allocate memory themselves
    bool rkopt_allocate_memory_outside = false;

    //! Whether to share weights between different models (requires special model export handling)
    bool rkopt_shared_weight = false;

    //! Whether to only load model info without model data (can query ports but not run inference)
    bool rkopt_load_model_info_only = false;

    //! Whether to use GPU for NPU ops not supported by the NPU
    bool rkopt_use_gpu_for_unsupported_npu_ops = false;

    //! Whether to use SRAM for internal tensors instead of DDR
    bool rkopt_internel_tensor_use_sram = false;

    //! Whether to share SRAM between models (all must use SRAM for internal tensors)
    bool rkopt_internal_tensor_share_sram = false;

    //! Whether to manually flush input cache instead of auto flush
    bool rkopt_manual_flush_input_cache = false;

    //! Whether to manually flush output cache instead of auto flush
    bool rkopt_manual_flush_output_cache = false;

    //! Whether to allow device memory allocation without context
    bool rkopt_allow_allocate_device_memory_without_context = false;
    //! @}

    //! Additional initialization flags for rknn_init
    uint32_t _more_init_flags = 0;

    /**
     * @brief Extended RKNN API initialization info required for certain options:
     * - rkopt_shared_weight (RKNN_FLAG_SHARE_WEIGHT_MEM): Need source model context
     * - rkopt_allow_allocate_device_memory_without_context (RKNN_MEM_FLAG_ALLOC_NO_CONTEXT): Need model buffer
     */
    std::shared_ptr<rknn_init_extend> extended_init_info;

    //! Convert the RKNN API options to initialization flags
    uint32_t to_init_flags() const;
};

//! Inference model using rk3588 npu
class RknnModelInference : public RedoxiModelInference
{
  public:
    // inference context of underlying api
    // e.g. rknn_context, onnx session, etc.
    using InferenceContext_t = rknn_context;
    using RknnTensorType_t = rknn_tensor_type;
    using InitConfig_t = RknnModelConfig;

    RknnModelInference();
    virtual ~RknnModelInference() = default;

    //! Create a key value store
    virtual KeyValueStore::Ptr create_init_params() override;

    //! Create an inference inout data object
    virtual InferenceInOutData::Ptr create_inference_inout_data() override;

    //! Get model metadata, related to the model itself
    virtual KeyValueStore::ConstPtr get_model_metadata() const override;

    //! Get inference metadata, related to the runtime environment
    virtual KeyValueStore::ConstPtr get_inference_metadata() const override;

    //! Get information of ports
    virtual ModelPortInfo::ConstPtrMap get_input_port_infos() const override;

    //! Get information of input ports
    virtual ModelPortInfo::PtrMap _get_input_port_infos();

    //! Get information of output ports
    virtual ModelPortInfo::ConstPtrMap get_output_port_infos() const override;

    //! Get information of output ports
    virtual ModelPortInfo::PtrMap _get_output_port_infos();

    //! Inference
    virtual int do_inference(InferenceInOutData::Ptr inout_data) override;

    //! Initialize the model inference, load model and other resources
    virtual int open(KeyValueStore::Ptr params) override;

    //! Check if the model inference is open, ready for inference
    virtual bool is_open() const override;

    //! Close the model inference, release inference resources
    virtual int close() override;

  public:
    auto get_inference_context() const
    {
        return m_context;
    }

  private:
    std::shared_ptr<RknnModelConfig> m_config;
    RknnModelPortInfo::PtrMap m_input_ports;
    RknnModelPortInfo::PtrMap m_output_ports;
    std::shared_ptr<InferenceContext_t> m_context;

  private:
    /**
     * @brief Create an ONNX session and load the model based on the provider type.
     *
     * @param model_path The file path to the ONNX model.
     * @param init_config The configuration for initializing the inference context
     * @return A shared pointer to the created Ort::Session.
     */
    static std::shared_ptr<InferenceContext_t> create_inference_context(
        const std::string &model_path,
        const InferenceContextInitConfig &init_config);

    /**
     * @brief Get information about the input ports of the ONNX model.
     *
     * @param session A shared pointer to the Ort::Session.
     * @return A map of input port names to their corresponding OnnxModelPortInfo pointers.
     */
    static RknnModelPortInfo::PtrMap get_input_port_infos(
        const InferenceContext_t &context);

    /**
     * @brief Get information about the output ports of the ONNX model.
     *
     * @param session A shared pointer to the Ort::Session.
     * @return A map of output port names to their corresponding OnnxModelPortInfo pointers.
     */
    static RknnModelPortInfo::PtrMap get_output_port_infos(
        const InferenceContext_t &context);

    /**
     * @brief Convert an ONNX tensor element data type to its string representation.
     *
     * @param dtype The ONNX tensor element data type.
     * @return A string representation of the data type.
     */
    static std::string rknn_tensor_type_to_string(RknnTensorType_t dtype);
};
} // namespace redoxi_works::inference::rknn
