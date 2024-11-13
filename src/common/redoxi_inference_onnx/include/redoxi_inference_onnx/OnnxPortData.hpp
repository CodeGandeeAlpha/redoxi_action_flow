#pragma once

#include <redoxi_inference_onnx/redoxi_inference_onnx.hpp>

namespace redoxi_works::inference::onnx
{

class OnnxPortData : public ModelPortData
{
public:
  OnnxPortData();
  virtual ~OnnxPortData();

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
  virtual int get_as_tensor(std::shared_ptr<Tensor_4d_f32>* output_tensor) const override;

  //! Get the data as a tensor.
  //! The shape must match the port info shape. If the data type or shape has a problem,
  //! it will throw an std::invalid_argument exception.
  //! @param output_tensor A pointer to a shared pointer that will hold the Tensor_4d_u8 object.
  //! @return int Returns 0 on success, or an error code on failure.
  //! @throws std::invalid_argument if the data type or shape is incorrect.
  virtual int get_as_tensor(std::shared_ptr<Tensor_4d_u8>* output_tensor) const override;

  //! Get expected shape of the tensor given a batch size, where the dimensions that can be dynamic are -1
  //! @param batch_size Optional batch size for dynamic dimensions.
  //! @return std::array<int, 4> The expected shape of the tensor.
  virtual std::array<int, 4> get_expected_shape_4d(std::optional<int> batch_size = std::nullopt) const override;
};

}  // namespace redoxi_works::inference::onnx
