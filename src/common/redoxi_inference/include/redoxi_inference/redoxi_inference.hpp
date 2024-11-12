#ifndef REDOXI_INFERENCE__REDOXI_INFERENCE_HPP_
#define REDOXI_INFERENCE__REDOXI_INFERENCE_HPP_

#include "redoxi_inference/visibility_control.h"
#include <opencv2/opencv.hpp>
#include <optional>

namespace redoxi_works
{

namespace inference
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
    
    // dtype as in opencv convention, like CV_32FC1, CV_8UC1, ...
    // always uses 1 channel
    int dtype_opencv = CV_32FC1;

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

  // convert from cv::Mat to the data type of the port
  // it will not copy the data, so you need to ensure the data is valid during inference
  // return 0 if success, -1 if failed
  virtual int set_by_cvmat(const cv::Mat &img) = 0;

  // load data from multiple cv::Mat, as a batch, they must be in the same shape
  // return 0 if success, -1 if failed
  virtual int set_by_cvmat(const std::vector<cv::Mat> &imgs) = 0;

  // get the data as cv::Mat
  // return 0 if success, -1 if failed
  virtual std::vector<cv::Mat> get_as_cvmat() const = 0;
};

class RedoxiModelInference
{
public:
  RedoxiModelInference() = default;
  virtual ~RedoxiModelInference() = default;

  // create a key value store
  static std::shared_ptr<KeyValueStore> create_key_value_store();

  // load model based on metadata
  virtual int load_model(std::shared_ptr<const KeyValueStore> metadata) = 0;

  // unload model
  virtual int unload_model() = 0;

  // do we have model loaded?
  virtual bool has_loaded_model() const = 0;

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

  // get output data
  virtual int get_output_data(ModelPortData* output_data, const std::string& port_name) = 0;

  // create model port data, with optional batch size if the input port supports dynamic batch size
  static std::shared_ptr<ModelPortData> create_model_port_data(
    const std::string& port_name, 
    std::optional<int> batch_size = std::nullopt);
};

}  // namespace inference

}  // namespace redoxi_works

#endif  // REDOXI_INFERENCE__REDOXI_INFERENCE_HPP_
