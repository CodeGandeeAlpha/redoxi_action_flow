#pragma once

#include <redoxi_inference_onnx/redoxi_inference_onnx.hpp>
#include <redoxi_inference_onnx/OnnxPortInfo.hpp>
#include <variant>
#include <algorithm>
#include <fmt/core.h>

namespace redoxi_works::inference::onnx
{

//! A class to hold the tensor data and the corresponding onnx tensor
template <typename T>
struct MappedTensorData {
    std::shared_ptr<std::vector<T>> data;
    // std::shared_ptr<Ort::Value> onnx_tensor;
    std::shared_ptr<Ort::MemoryInfo> onnx_memory_info;
    std::vector<int64_t> shape;

    MappedTensorData() = default;
    MappedTensorData(const std::vector<int64_t> &shape)
    {
        init(shape);
    }

    virtual ~MappedTensorData() = default;

    //! Check if any dimension is dynamic
    bool has_dynamic_dims() const
    {
        return std::any_of(shape.begin(), shape.end(), [](int64_t dim) { return dim < 0; });
    }

    //! Check if data is allocated
    bool has_data() const
    {
        return data != nullptr;
    }

    //! Initialize all internals based on the shape
    //! If you change the shape, you need to call this function again
    //! For dynamic dimensions, data will not be allocated
    void init(const std::vector<int64_t> &shape)
    {
        if (shape.empty()) {
            // no shape, nothing to do
            return;
        }
        this->shape = shape;
        this->onnx_memory_info = std::make_shared<Ort::MemoryInfo>(
            Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeDefault));

        // dynamic dimensions cannot be preallocated, just skip
        if (has_dynamic_dims()) {
            this->data.reset();
            // this->onnx_tensor.reset();
            return;
        }

        // allocate data
        auto num_elements = std::accumulate(shape.begin(), shape.end(), 1, std::multiplies<int64_t>());
        this->data = std::make_shared<std::vector<T>>(num_elements);
        // this->onnx_tensor = std::make_shared<Ort::Value>(Ort::Value::CreateTensor<T>(
        //     *onnx_memory_info, data->data(), data->size(),
        //     shape.data(), shape.size()));
        RDX_INFO_DEV(nullptr, __func__, false, "Tensor data initialized, shape={}, num_elements={}",
                     fmt::join(shape, ", "), num_elements);
    }
};

using MappedTensorData_f32 = MappedTensorData<float>;
using MappedTensorData_u8 = MappedTensorData<uint8_t>;

class OnnxModelInference;
class OnnxInferenceInOutData;

//! A class to hold the tensor data ready to be used with a port
class OnnxPortData : public ModelPortData
{
    friend class OnnxModelInference;
    friend class OnnxInferenceInOutData;

  public:
    using Ptr = std::shared_ptr<OnnxPortData>;
    using ConstPtr = std::shared_ptr<const OnnxPortData>;

  public:
    // get information of the port
    virtual ModelPortInfo::ConstPtr get_port_info() const override;
    virtual int set_tensor_data(const float *data, std::vector<int64_t> shape,
                                std::optional<TensorFormat> fmt = std::nullopt) override;
    virtual int set_tensor_data(const uint8_t *data, std::vector<int64_t> shape,
                                std::optional<TensorFormat> fmt = std::nullopt) override;
    virtual int get_tensor_data(const float **output_tensor) const override;
    virtual int get_tensor_data(float **output_tensor) override;
    virtual int get_tensor_data(const uint8_t **output_tensor) const override;
    virtual int get_tensor_data(uint8_t **output_tensor) override;
    virtual std::vector<int64_t> get_shape() const override;
    virtual bool has_tensor_data() const override;
    virtual std::string get_dtype_str() const override;

    //! Set the port info and shape, the shape should be more specific than the port shape
    //! If shape is provided, it will check if the shape is compatible with the port shape
    //! If not, it will raise an error
    virtual int init(OnnxModelPortInfo::Ptr port_info,
                     std::optional<std::vector<int64_t>> shape = std::nullopt);

  protected:
    //! Set the shape
    //! If the shape is not compatible with the port shape, it will raise an error
    //! If the shape is compatible, it will allocate the data
    //! Note that if dynamic dimensions are present in shape, it is considered ok, but data will not be allocated
    virtual int _set_shape_and_allocate_data(const std::vector<int64_t> &shape);

    //! Initialize all internals based on the port info and shape
    //! If you change the shape, you need to call this function again
    virtual int _allocate_data();

  protected:
    OnnxModelPortInfo::Ptr m_port_info;
    std::variant<MappedTensorData_f32,
                 MappedTensorData_u8>
        m_tensor_data;
    std::vector<int64_t> m_shape;

  public:
    // callback function to notify shape changed
    // will be called after the shape is changed and data is allocated
    std::function<void(const std::vector<int64_t> &new_shape, const std::vector<int64_t> &old_shape)> on_shape_changed;
};

} // namespace redoxi_works::inference::onnx
