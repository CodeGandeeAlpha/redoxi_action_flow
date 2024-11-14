#pragma once

#include <redoxi_inference_onnx/redoxi_inference_onnx.hpp>
#include <redoxi_inference_onnx/OnnxModelConfig.hpp>
#include <redoxi_inference_onnx/OnnxPortInfo.hpp>
#include <onnxruntime_cxx_api.h>

namespace redoxi_works::inference::onnx
{
class OnnxModelInference : public RedoxiModelInference
{
  public:
    //! A struct to hold the information of an inference port
    struct InferencePort {
        // original port info
        OnnxModelPortInfo::Ptr info_original;

        // configured port info, where some dynamic properties are set
        OnnxModelPortInfo::Ptr info_configured;

        // data for input/output port, with shapes matched to the configured shape
        ModelPortData::Ptr data;

        // memory info for io binding
        std::shared_ptr<Ort::MemoryInfo> ort_memory_info;

        // whether the port has been bound to an io binding
        bool has_io_binding = false;
    };

    OnnxModelInference();
    virtual ~OnnxModelInference() = default;

    //! Create a key value store
    virtual KeyValueStore::Ptr create_init_params() override;

    //! Configure an input port, set the shape of the port
    //! For dynamic dimensions, set the dimension to -1, or a specific value if it is static
    //! @note this will invalidate the data in the corresponding input port, it may be destroyed and recreated
    //! @return 0 if success, -1 if failed (the model does not support this shape)
    virtual int configure_input_port(
        const std::string &port_name,
        std::vector<int64_t> shape) override;

    //! Get the data of an input port
    virtual ModelPortData::Ptr get_input_port_data(const std::string &port_name) override;

    //! Get the data of an output port
    virtual ModelPortData::Ptr get_output_port_data(const std::string &port_name) override;

    //! Get model metadata, related to the model itself
    virtual KeyValueStore::ConstPtr get_model_metadata() const override;

    //! Get inference metadata, related to the runtime environment
    virtual KeyValueStore::ConstPtr get_inference_metadata() const override;

    //! Get information of ports
    virtual ModelPortInfo::ConstPtrMap get_input_port_infos() const override;

    //! Get information of output ports
    virtual ModelPortInfo::ConstPtrMap get_output_port_infos() const override;

    //! Notify the model that all input data are set
    //! Call this before inference, after all input data are set
    //! If you don't call this, the input data might not be up-to-date
    virtual int notify_input_ready() override;

    //! Inference
    virtual int do_inference() override;

    //! Initialize the model inference, load model and other resources
    virtual int init(KeyValueStore::Ptr params) override;

    //! Open the model inference, get ready for inference
    virtual int open() override;

    //! Check if the model inference is open, ready for inference
    virtual bool is_open() const override;

    //! Close the model inference, release inference resources
    virtual int close() override;

  private:
    //! Initialize the port information
    void _init_all_ports();

  private:
    std::shared_ptr<Ort::Session> m_session;
    std::shared_ptr<Ort::Env> m_env;
    std::shared_ptr<OnnxModelConfig> m_config;
    std::shared_ptr<Ort::IoBinding> m_io_binding;
    std::map<std::string, InferencePort> m_input_ports;
    std::map<std::string, InferencePort> m_output_ports;

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
        const std::shared_ptr<Ort::Session> &session);

    /**
     * @brief Get information about the output ports of the ONNX model.
     *
     * @param session A shared pointer to the Ort::Session.
     * @return A map of output port names to their corresponding OnnxModelPortInfo pointers.
     */
    static OnnxModelPortInfo::PtrMap get_output_port_infos(
        const std::shared_ptr<Ort::Session> &session);

    /**
     * @brief Convert an ONNX tensor element data type to its string representation.
     *
     * @param dtype The ONNX tensor element data type.
     * @return A string representation of the data type.
     */
    static std::string onnx_element_type_to_string(ONNXTensorElementDataType dtype);
};
} // namespace redoxi_works::inference::onnx
