#pragma once

#include <redoxi_basic_cpp/logging/ros_logging.hpp>
#include <redoxi_inference_rknn/RknnPortInfo.hpp>
#include <variant>
#include <algorithm>
#include <numeric>
#include <fmt/core.h>
#include <fmt/format.h>

namespace redoxi_works::inference::rknn
{

//! A class to hold the tensor data and the corresponding onnx tensor
template <typename T>
struct MappedTensorData {
    std::shared_ptr<std::vector<T>> data;
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

        // dynamic dimensions cannot be preallocated, just skip
        if (has_dynamic_dims()) {
            this->data.reset();
            return;
        }


        // allocate data
        auto num_elements = std::accumulate(shape.begin(), shape.end(), 1, std::multiplies<int64_t>());
        this->data = std::make_shared<std::vector<T>>(num_elements);
        RDX_INFO_DEV(nullptr, __func__, false, "Tensor data initialized, shape={}, num_elements={}",
                     fmt::join(shape, ", "), num_elements);
    }
};

using MappedTensorData_f32 = MappedTensorData<float>;
using MappedTensorData_u8 = MappedTensorData<uint8_t>;

class RknnModelInference;
class RknnInferenceInOutData;

//! A class to hold the tensor data ready to be used with a port
class RknnPortData : public ModelPortData
{
    friend class RknnModelInference;
    friend class RknnInferenceInOutData;

  public:
    using Ptr = std::shared_ptr<RknnPortData>;
    using ConstPtr = std::shared_ptr<const RknnPortData>;

  public:
    virtual ModelPortInfo::ConstPtr get_port_info() const override;
    virtual int set_tensor_data(const float *data, std::vector<int64_t> shape) override;
    virtual int set_tensor_data(const uint8_t *data, std::vector<int64_t> shape) override;
    virtual int get_tensor_data(const float **output_tensor) const override;
    virtual int get_tensor_data(float **output_tensor) override;
    virtual int get_tensor_data(const uint8_t **output_tensor) const override;
    virtual int get_tensor_data(uint8_t **output_tensor) override;
    virtual std::vector<int64_t> get_shape() const override;
    virtual bool has_tensor_data() const override;
    virtual std::string get_dtype_str() const override;
    virtual int init(RknnModelPortInfo::Ptr port_info, std::optional<std::vector<int64_t>> shape = std::nullopt);

  protected:
    virtual int _set_shape_and_allocate_data(const std::vector<int64_t> &shape);
    virtual int _allocate_data();

  protected:
    RknnModelPortInfo::Ptr m_port_info;
    std::variant<MappedTensorData_f32,
                 MappedTensorData_u8>
        m_tensor_data;
    std::vector<int64_t> m_shape;

  public:
    std::function<void(const std::vector<int64_t> &new_shape, const std::vector<int64_t> &old_shape)> on_shape_changed;
};

} // namespace redoxi_works::inference::rknn
