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

    // get information of the port
    virtual ModelPortInfo::ConstPtr get_port_info() const = 0;

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
    virtual int get_as_tensor(std::shared_ptr<Tensor_4d_f32> *output_tensor) const = 0;

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
    virtual int get_as_tensor(std::shared_ptr<Tensor_4d_u8> *output_tensor) const = 0;
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
