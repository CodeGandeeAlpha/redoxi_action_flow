#pragma once

#include <redoxi_inference/redoxi_inference.hpp>
#include <redoxi_inference_rknn/RknnModelConfig.hpp>
#include <redoxi_inference_rknn/RknnPortInfo.hpp>
#include <redoxi_inference_rknn/RknnInferenceInOutData.hpp>
#include <rknn_api.h>

namespace redoxi_works::inference::rknn
{

struct InferenceContextInitConfig {
    std::vector<int> use_npu_cores = {0, 1, 2}; // default enable all cores
    uint32_t init_flags = 0;                    // other flags for rknn_init
};

//! ONNX model inference
//! IMPORTANT: the current implementation assumes input and output shape do not change frequently
//! otherwise the performance will be very slow because io binding will be recreated every time when the shape is changed
class RknnModelInference : public RedoxiModelInference
{
  public:
    // inference context of underlying api
    // e.g. rknn_context, onnx session, etc.
    using InferenceContext_t = rknn_context;
    using RknnTensorType_t = rknn_tensor_type;

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
    //! Initialize the port information
    void _init_all_ports();

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
    static std::string rknn_element_type_to_string(RknnTensorType_t dtype);
};
} // namespace redoxi_works::inference::rknn
