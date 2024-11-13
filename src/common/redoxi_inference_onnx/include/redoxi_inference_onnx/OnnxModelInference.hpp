#pragma once

#include <redoxi_inference_onnx/redoxi_inference_onnx.hpp>

namespace redoxi_works::inference::onnx
{

class OnnxModelInference : public RedoxiModelInference
{
public:
  OnnxModelInference();
  virtual ~OnnxModelInference();

  //! Create a key value store
  virtual std::shared_ptr<KeyValueStore> create_key_value_store() override;

  //! Create model port data, with optional batch size if the input port supports dynamic batch size
  virtual std::shared_ptr<ModelPortData> create_model_port_data(
    const std::string& port_name, 
    std::optional<int> batch_size = std::nullopt) override;

  //! Get model metadata, related to the model itself
  virtual std::shared_ptr<const KeyValueStore> get_model_metadata() const override;

  //! Get inference metadata, related to the runtime environment
  virtual std::shared_ptr<const KeyValueStore> get_inference_metadata() const override;

  //! Get information of input ports
  virtual std::vector<ModelPortInfo> get_input_port_infos() const override;

  //! Get information of output ports
  virtual std::vector<ModelPortInfo> get_output_port_infos() const override;

  //! Set input data
  virtual int set_input_data(const std::string& port_name, std::shared_ptr<ModelPortData> data) override;

  //! Get output data
  virtual int get_output_data(ModelPortData* output_data, const std::string& port_name) override;

  //! Notify the model that all input data are set
  //! Call this before inference, after all input data are set
  //! If you don't call this, the input data might not be up-to-date
  virtual int notify_input_ready() override;

  //! Inference
  virtual int do_inference() override;

  //! Get output data
  virtual std::shared_ptr<ModelPortData> get_output_data(const std::string& port_name) override;

  //! Initialize the model inference, load model and other resources
  virtual int init(std::shared_ptr<KeyValueStore> params) override;

  //! Open the model inference, get ready for inference
  virtual int open() override;

  //! Check if the model inference is open, ready for inference
  virtual bool is_open() const override;

  //! Close the model inference, release inference resources
  virtual int close() override;
};
}  // namespace redoxi_works::inference::onnx
