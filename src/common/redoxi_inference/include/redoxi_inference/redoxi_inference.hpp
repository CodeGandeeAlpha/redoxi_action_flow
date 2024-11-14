#ifndef REDOXI_INFERENCE__REDOXI_INFERENCE_HPP_
#define REDOXI_INFERENCE__REDOXI_INFERENCE_HPP_

#include "redoxi_inference/visibility_control.h"
#include "redoxi_inference/redoxi_tensor_def.hpp"
#include <optional>
#include <array>
#include <memory>
#include <map>


namespace redoxi_works::inference
{

//! Check if a specific shape is compatible with a general shape.
//! Specific shape set fixed dimensions for dynamic dimensions in the general shape, but not the other way around
//! - if general_shape[k]>0, then specific_shape[k]==general_shape[k]
//! - if general_shape[k]==-1, then specific_shape[k] can be -1 or positive
//! - in any case, specific_shape[k]!=0
//! @param specific_shape The shape to check
//! @param general_shape The general shape to compare against, where -1 means dynamic dimension
//! @return True if the specific shape is compatible with the general shape, false otherwise
inline bool is_shape_compatible(const std::vector<int64_t> &specific_shape,
                                const std::vector<int64_t> &general_shape)
{
    if (specific_shape.size() != general_shape.size()) {
        return false;
    }

    // check each dimension
    for (size_t i = 0; i < specific_shape.size(); ++i) {
        if (general_shape[i] > 0) {
            // specific shape must match general shape, if general shape is fixed
            bool ok = specific_shape[i] == general_shape[i];
            if (!ok) {
                return false;
            }
        } else if (general_shape[i] == -1) {
            // specific shape can be dynamic or match general shape, if general shape is dynamic
            bool ok = specific_shape[i] == -1 || specific_shape[i] > 0;
            if (!ok) {
                return false;
            }
        } else {
            return false; // general_shape[i] should not be 0 or negative other than -1
        }
    }

    // all dimensions are compatible
    return true;
}

struct KeyValueStore {
    using Ptr = std::shared_ptr<KeyValueStore>;
    using ConstPtr = std::shared_ptr<const KeyValueStore>;

    virtual ~KeyValueStore() = default;
    using RawPtr = KeyValueStore *;

    struct KeyInfo {
        std::string key;         // key name
        std::string dtype;       // data type, such as "string", "int64", "double", ...
        std::string description; // description of the key
    };

    //! Get all keys and their information
    virtual const std::vector<KeyInfo> &get_all_keys() const = 0;

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

class ModelPortInfo
{
  public:
    using Ptr = std::shared_ptr<ModelPortInfo>;
    using PtrMap = std::map<std::string, Ptr>;
    using ConstPtr = std::shared_ptr<const ModelPortInfo>;
    using ConstPtrMap = std::map<std::string, ConstPtr>;

    virtual ~ModelPortInfo() = default;

    //! Get the name of the port
    virtual std::string get_name() const
    {
        return m_name;
    }

    //! Get the data type of the port
    //! @return The data type of the port
    virtual std::string get_dtype_str() const
    {
        return m_dtype_str;
    }

    //! Get the expected shape of the tensor
    //! @return The expected shape of the tensor, -1 for dynamic dimension
    virtual std::vector<int64_t> get_shape() const
    {
        return m_shape;
    }

    //! Get if the port is an input port
    //! @return True if the port is an input port, false if it is an output port
    virtual bool is_input() const
    {
        return m_is_input;
    }

    //! Get the expected shape of the tensor given a batch size, where the dimensions that can be dynamic are -1
    //! @param batch_size The batch size, optional
    //! @return The expected shape of the tensor in 4d format, typically [N,C,H,W], depends on the model
    //! If the port cannot deal with 4d tensor (e.g., required 5d tensor or more), return std::nullopt
    // virtual std::optional<std::array<int, 4>> get_shape_4d(std::optional<int> batch_size = std::nullopt) const = 0;
  protected:
    // this class is intended to be subclassed
    // do not create it directly
    ModelPortInfo() = default;

    std::string m_name;
    std::string m_dtype_str;
    std::vector<int64_t> m_shape;
    bool m_is_input{false};
};

class ModelPortData
{
  public:
    using Ptr = std::shared_ptr<ModelPortData>;
    using ConstPtr = std::shared_ptr<const ModelPortData>;

    ModelPortData() = default;
    virtual ~ModelPortData() = default;

    // get information of the port, note that there may be dynamic shape for the data
    // so you need to use get_shape() to get the actual shape of the data
    virtual ModelPortInfo::ConstPtr get_port_info() const = 0;

    //! Set data by tensor, for float type. Data will be copied into the tensor
    virtual int set_tensor_data(const float *data, std::vector<int64_t> shape) = 0;

    //! Set data by tensor, for uint8_t type. Data will be copied into the tensor
    virtual int set_tensor_data(const uint8_t *data, std::vector<int64_t> shape) = 0;

    //! Get the shape of the tensor, you need this to interpret the data you get from get_as_tensor()
    //! @return The shape of the tensor
    virtual std::vector<int64_t> get_shape() const = 0;

    //! Get the data type of the tensor, you need this to interpret the data you get from get_tensor_data()
    //! @return The data type of the tensor
    virtual std::string get_dtype_str() const = 0;

    //! Get the raw data pointer of the tensor, then you can interpret the data by its shape
    virtual int get_tensor_data(const float **output_tensor) const = 0;

    //! Get the raw data pointer of the tensor, then you can interpret the data by its shape
    //! you can also modify the data by this pointer
    //! @return 0 if success, -1 if failed
    virtual int get_tensor_data(float **output_tensor) = 0;

    //! Get the raw data pointer of the tensor, then you can interpret the data by its shape
    //! @return 0 if success, -1 if failed
    virtual int get_tensor_data(const uint8_t **output_tensor) const = 0;

    //! Get the raw data pointer of the tensor, then you can interpret the data by its shape
    //! you can also modify the data by this pointer
    //! @return 0 if success, -1 if failed
    virtual int get_tensor_data(uint8_t **output_tensor) = 0;
};

//! A class to hold the input and output data of a model inference, like onnx io_binding
//! for those input ports that support dynamic size, you can configure a fixed size for them
//! in this class, so that input data memory can be reused
class RedoxiModelInference;
class InferenceInOutData
{
  public:
    using Ptr = std::shared_ptr<InferenceInOutData>;
    using ConstPtr = std::shared_ptr<const InferenceInOutData>;

    InferenceInOutData() = default;
    virtual ~InferenceInOutData() = default;

    // configure an input port, set the shape of the port
    // for dynamic dimensions, set the dimension to -1, or a specific value if it is static
    // @return 0 if success, -1 if failed (the model does not support this shape)
    virtual int configure_input_port(
        const std::string &port_name,
        std::vector<int64_t> shape) = 0;

    // get the information of a configured input port
    virtual ModelPortInfo::ConstPtr get_input_port_info(const std::string &port_name) const = 0;

    // get the information of a configured output port
    virtual ModelPortInfo::ConstPtr get_output_port_info(const std::string &port_name) const = 0;

    // get the data of a configured input port, where the data must be filled before inference
    // the object will be invalidated after configure_input_port() is called,
    // in which case you need to call get_input_port_data() again
    virtual ModelPortData::Ptr get_input_port_data(const std::string &port_name) = 0;

    // get the data of a configured output port, the data is filled by the model after inference
    virtual ModelPortData::Ptr get_output_port_data(const std::string &port_name) = 0;

    // notify the inferencer that all input port configuration and data are updated
    // call this before inference, otherwise input data may not be used
    virtual void notify_input_update() = 0;

    // get the inferencer that this inout data belongs to
    virtual RedoxiModelInference *get_owner() = 0;

    // get the inferencer that this inout data belongs to, const version
    virtual const RedoxiModelInference *get_owner() const = 0;
};

class RedoxiModelInference
{
  public:
    RedoxiModelInference() = default;
    virtual ~RedoxiModelInference() = default;

    // create a key value store for initialization parameters
    virtual KeyValueStore::Ptr create_init_params() = 0;

    // create an inference inout data object
    virtual InferenceInOutData::Ptr create_inference_inout_data() = 0;

    // get the information of all input ports, in original model definition
    virtual ModelPortInfo::ConstPtrMap get_input_port_infos() const = 0;

    // get the information of all output ports, in original model definition
    virtual ModelPortInfo::ConstPtrMap get_output_port_infos() const = 0;

    // initialize the model inference, load model and other resources
    virtual int init(KeyValueStore::Ptr params) = 0;

    // open the model inference, get ready for inference
    virtual int open() = 0;

    // check if the model inference is open, ready for inference
    virtual bool is_open() const = 0;

    // close the model inference, release inference resources
    virtual int close() = 0;

    // get model metadata, related to the model itself
    virtual KeyValueStore::ConstPtr get_model_metadata() const = 0;

    // get inference metadata, related to the runtime environment
    virtual KeyValueStore::ConstPtr get_inference_metadata() const = 0;

    // inference
    virtual int do_inference(InferenceInOutData::Ptr inout_data) = 0;
};

} // namespace redoxi_works::inference

#endif // REDOXI_INFERENCE__REDOXI_INFERENCE_HPP_
