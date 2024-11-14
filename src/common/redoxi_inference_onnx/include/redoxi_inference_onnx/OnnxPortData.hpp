#pragma once

#include <redoxi_inference_onnx/redoxi_inference_onnx.hpp>
#include <redoxi_inference_onnx/OnnxPortInfo.hpp>

namespace redoxi_works::inference::onnx
{

class OnnxModelInference;
class OnnxPortData : public ModelPortData
{
    friend class OnnxModelInference;

  public:
    OnnxPortData();
    virtual ~OnnxPortData();

    // get information of the port
    virtual ModelPortInfo::ConstPtr get_port_info() const override;

    //! Set data by tensor.
    //! The shape must match the port info shape. If the data type or shape has a problem,
    //! it will throw an std::invalid_argument exception.
    //! @param tensor A shared pointer to a Tensor_4d_f32 object.
    //! @return int Returns 0 on success, or an error code on failure.
    //! @throws std::invalid_argument if the data type or shape is incorrect.
    virtual int set_by_tensor(std::shared_ptr<Tensor_4d_f32> tensor) override;

    //! Set data by tensor.
    //! The shape must match the port info shape. If the data type or shape has a problem,
    //! it will throw an std::invalid_argument exception.
    //! @param tensor A shared pointer to a Tensor_4d_u8 object.
    //! @return int Returns 0 on success, or an error code on failure.
    //! @throws std::invalid_argument if the data type or shape is incorrect.
    virtual int set_by_tensor(std::shared_ptr<Tensor_4d_u8> tensor) override;

    //! Get the data as a tensor.
    //! The shape must match the port info shape. If the data type or shape has a problem,
    //! it will throw an std::invalid_argument exception.
    //! @param output_tensor A pointer to a shared pointer that will hold the Tensor_4d_f32 object.
    //! @return int Returns 0 on success, or an error code on failure.
    //! @throws std::invalid_argument if the data type or shape is incorrect.
    virtual int get_as_tensor(std::shared_ptr<Tensor_4d_f32> *output_tensor) const override;

    //! Get the data as a tensor.
    //! The shape must match the port info shape. If the data type or shape has a problem,
    //! it will throw an std::invalid_argument exception.
    //! @param output_tensor A pointer to a shared pointer that will hold the Tensor_4d_u8 object.
    //! @return int Returns 0 on success, or an error code on failure.
    //! @throws std::invalid_argument if the data type or shape is incorrect.
    virtual int get_as_tensor(std::shared_ptr<Tensor_4d_u8> *output_tensor) const override;

  private:
    OnnxModelPortInfo::Ptr m_port_info;
};

} // namespace redoxi_works::inference::onnx
