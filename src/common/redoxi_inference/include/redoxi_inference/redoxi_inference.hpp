#ifndef REDOXI_INFERENCE__REDOXI_INFERENCE_HPP_
#define REDOXI_INFERENCE__REDOXI_INFERENCE_HPP_

#include "redoxi_inference/visibility_control.h"
#include "redoxi_inference/redoxi_tensor_def.hpp"
#include <optional>
#include <array>
#include <memory>


namespace redoxi_works::inference
{
struct KeyValueStore {
    virtual ~KeyValueStore() = default;
    using RawPtr = KeyValueStore *;

    //! Get a string value from the connect params
    virtual int get_string(std::string *output, const std::string &key) const = 0;

    //! Set a string value in the connect params
    //! @return 0 if success, -1 if failed
    virtual int set_string(const std::string &key, const std::string &value) = 0;

    //! Get a int64_t value from the connect params
    virtual int get_int(int64_t *output, const std::string &key) const = 0;

    //! Set a int64_t value in the connect params
    //! @return 0 if success, -1 if failed
    virtual int set_int(const std::string &key, int64_t value) = 0;

    //! Get a double value from the connect params
    virtual int get_double(double *output, const std::string &key) const = 0;

    //! Set a double value in the connect params
    //! @return 0 if success, -1 if failed
    virtual int set_double(const std::string &key, double value) = 0;

    //! Check if the connect params has a key
    virtual bool has_key(const std::string &key) const = 0;
};

struct ModelPortInfo
{
    // name of the port, unique for each port
    std::string name;

    // string representation of dtype, such as "float32", "int64", "float16", ...
    // following numpy dtype string
    std::string dtype_str;

    // expected shape of the tensor
    // -1 for dynamic dimension
    // following pytorch shape convention, [N,C,H,W]
    std::vector<int64_t> shape;

    // true if this is an input port, false if it is an output port
    bool is_input = false;
};

class ModelPortData
{
public:
  ModelPortData() = default;
  virtual ~ModelPortData() = default;

  // get information of the port
  virtual const ModelPortInfo &get_port_info() const = 0;

  /**
   * @brief Set data by tensor.
   * 
   * The shape must match the port info shape. If the data type or shape has a problem,
   * it will throw an std::invalid_argument exception.
   * 
   * @param tensor A shared pointer to a Tensor_4d_f32 object.
   * @return int Returns 0 on success, or an error code on failure.
   * @throws std::invalid_argument if the data type or shape is incorrect.
   */
  virtual int set_by_tensor(std::shared_ptr<Tensor_4d_f32> tensor) = 0;

  /**
   * @brief Set data by tensor.
   * 
   * The shape must match the port info shape. If the data type or shape has a problem,
   * it will throw an std::invalid_argument exception.
   * 
   * @param tensor A shared pointer to a Tensor_4d_u8 object.
   * @return int Returns 0 on success, or an error code on failure.
   * @throws std::invalid_argument if the data type or shape is incorrect.
   */
  virtual int set_by_tensor(std::shared_ptr<Tensor_4d_u8> tensor) = 0;

  /**
   * @brief Get the data as a tensor.
   * 
   * The shape must match the port info shape. If the data type or shape has a problem,
   * it will throw an std::invalid_argument exception.
   * 
   * @param output_tensor A pointer to a shared pointer that will hold the Tensor_4d_f32 object.
   * @return int Returns 0 on success, or an error code on failure.
   * @throws std::invalid_argument if the data type or shape is incorrect.
   */
  virtual int get_as_tensor(std::shared_ptr<Tensor_4d_f32>* output_tensor) const = 0;

  /**
   * @brief Get the data as a tensor.
   * 
   * The shape must match the port info shape. If the data type or shape has a problem,
   * it will throw an std::invalid_argument exception.
   * 
   * @param output_tensor A pointer to a shared pointer that will hold the Tensor_4d_u8 object.
   * @return int Returns 0 on success, or an error code on failure.
   * @throws std::invalid_argument if the data type or shape is incorrect.
   */
  virtual int get_as_tensor(std::shared_ptr<Tensor_4d_u8>* output_tensor) const = 0;

  // get expected shape of the tensor given a batch size, where the dimensions that can be dynamic are -1
  virtual std::array<int, 4> get_expected_shape_4d(std::optional<int> batch_size = std::nullopt) const = 0;
};

class RedoxiModelInference
{
public:
  RedoxiModelInference() = default;
  virtual ~RedoxiModelInference() = default;

  // create a key value store
  virtual std::shared_ptr<KeyValueStore> create_key_value_store() = 0;

  // create model port data, with optional batch size if the input port supports dynamic batch size
  virtual std::shared_ptr<ModelPortData> create_model_port_data(
    const std::string& port_name, 
    std::optional<int> batch_size = std::nullopt) = 0;

  // initialize the model inference, load model and other resources
  virtual int init(std::shared_ptr<KeyValueStore> params) = 0;

  // open the model inference, get ready for inference
  virtual int open() = 0;

  // check if the model inference is open, ready for inference
  virtual bool is_open() const = 0;

  // close the model inference, release inference resources
  virtual int close() = 0;

  // get model metadata, related to the model itself
  virtual std::shared_ptr<const KeyValueStore> get_model_metadata() const = 0;

  // get inference metadata, related to the runtime environment
  virtual std::shared_ptr<const KeyValueStore> get_inference_metadata() const = 0;

  // get information of input ports
  virtual std::vector<ModelPortInfo> get_input_port_infos() const = 0;

  // get information of output ports
  virtual std::vector<ModelPortInfo> get_output_port_infos() const = 0;

  // set input data
  virtual int set_input_data(const std::string& port_name, std::shared_ptr<ModelPortData> data) = 0;

  // get output data
  virtual int get_output_data(ModelPortData* output_data, const std::string& port_name) = 0;

  // notify the model that all input data are set
  // call this before inference, after all input data are set
  // if you don't call this, the input data might not up-to-date
  virtual int notify_input_ready() = 0;

  // inference
  virtual int do_inference() = 0;

  // get output data
  virtual std::shared_ptr<ModelPortData> get_output_data(const std::string& port_name)
  {
    auto data = create_model_port_data(port_name);
    auto ret = get_output_data(data.get(), port_name);
    if (ret != 0)
    {
      return nullptr;
    }
    return data;
  }


};

}  // namespace redoxi_works::inference

#endif  // REDOXI_INFERENCE__REDOXI_INFERENCE_HPP_
