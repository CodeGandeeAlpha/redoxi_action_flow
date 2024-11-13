#pragma once

#include <redoxi_inference_onnx/redoxi_inference_onnx.hpp>
#include <redoxi_inference_onnx/OnnxModelConfig.hpp>
#include <onnxruntime_cxx_api.h>

namespace redoxi_works::inference::onnx
{

class OnnxModelInference : public RedoxiModelInference
{
public:
  OnnxModelInference();
  virtual ~OnnxModelInference() = default;

  //! Create a key value store
  virtual KeyValueStore::Ptr create_key_value_store() override;

  //! Create model port data, with optional batch size if the input port supports dynamic batch size
  virtual ModelPortData::Ptr create_model_port_data(
    const std::string& port_name, 
    std::optional<int> batch_size = std::nullopt) override;

  //! Get model metadata, related to the model itself
  virtual KeyValueStore::ConstPtr get_model_metadata() const override;

  //! Get inference metadata, related to the runtime environment
  virtual KeyValueStore::ConstPtr get_inference_metadata() const override;

  //! Get information of ports
  virtual ModelPortInfo::ConstPtrMap get_port_infos() const override;

  //! Set input data
  virtual int set_input_data(const std::string& port_name, ModelPortData::Ptr data) override;

  //! Get output data
  virtual int get_output_data(ModelPortData::Ptr output_data, const std::string& port_name) override;

  //! Notify the model that all input data are set
  //! Call this before inference, after all input data are set
  //! If you don't call this, the input data might not be up-to-date
  virtual int notify_input_ready() override;

  //! Inference
  virtual int do_inference() override;

  //! Get output data
  virtual ModelPortData::Ptr get_output_data(const std::string& port_name) override;

  //! Initialize the model inference, load model and other resources
  virtual int init(KeyValueStore::Ptr params) override;

  //! Open the model inference, get ready for inference
  virtual int open() override;

  //! Check if the model inference is open, ready for inference
  virtual bool is_open() const override;

  //! Close the model inference, release inference resources
  virtual int close() override;

private:
  std::shared_ptr<Ort::Session> m_session;
  std::shared_ptr<Ort::Env> m_env;
  std::shared_ptr<OnnxModelConfig> m_config;
  ModelPortInfo::PtrMap m_port_infos;

private:
  static std::shared_ptr<Ort::Session> create_onnx_session(
    const std::string &model_path,
    const std::string &provider_type,
    Ort::Env &env);
};
}  // namespace redoxi_works::inference::onnx
