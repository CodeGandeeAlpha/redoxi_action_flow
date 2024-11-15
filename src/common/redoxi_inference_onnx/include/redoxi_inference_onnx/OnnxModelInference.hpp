#pragma once

#include <redoxi_inference_onnx/redoxi_inference_onnx.hpp>
#include <redoxi_inference_onnx/OnnxModelConfig.hpp>
#include <redoxi_inference_onnx/OnnxPortInfo.hpp>
#include <redoxi_inference_onnx/OnnxInferenceInOutData.hpp>
#include <onnxruntime_cxx_api.h>

namespace redoxi_works::inference::onnx
{
class OnnxModelInference : public RedoxiModelInference
{
  public:
    OnnxModelInference();
    virtual ~OnnxModelInference() = default;

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
    auto get_onnx_session() const
    {
        return m_session;
    }

  private:
    //! Initialize the port information
    void _init_all_ports();

  private:
    std::shared_ptr<Ort::Session> m_session;
    std::shared_ptr<Ort::Env> m_env;
    std::shared_ptr<OnnxModelConfig> m_config;
    OnnxModelPortInfo::PtrMap m_input_ports;
    OnnxModelPortInfo::PtrMap m_output_ports;

  private:
    /**
     * @brief Create an ONNX session and load the model based on the provider type.
     *
     * @param model_path The file path to the ONNX model.
     * @param provider_type The type of execution provider (e.g., CUDA, CPU, TensorRT),
     * available options are defined in onnx_ep_names namespace
     * @param env The ONNX runtime environment.
     * @return A shared pointer to the created Ort::Session.
     */
    static std::shared_ptr<Ort::Session> create_onnx_session(
        const std::string &model_path,
        const std::string &provider_type,
        Ort::Env &env);

    /**
     * @brief Get information about the input ports of the ONNX model.
     *
     * @param session A shared pointer to the Ort::Session.
     * @return A map of input port names to their corresponding OnnxModelPortInfo pointers.
     */
    static OnnxModelPortInfo::PtrMap get_input_port_infos(
        const Ort::Session &session);

    /**
     * @brief Get information about the output ports of the ONNX model.
     *
     * @param session A shared pointer to the Ort::Session.
     * @return A map of output port names to their corresponding OnnxModelPortInfo pointers.
     */
    static OnnxModelPortInfo::PtrMap get_output_port_infos(
        const Ort::Session &session);

    /**
     * @brief Convert an ONNX tensor element data type to its string representation.
     *
     * @param dtype The ONNX tensor element data type.
     * @return A string representation of the data type.
     */
    static std::string onnx_element_type_to_string(ONNXTensorElementDataType dtype);
};
} // namespace redoxi_works::inference::onnx
