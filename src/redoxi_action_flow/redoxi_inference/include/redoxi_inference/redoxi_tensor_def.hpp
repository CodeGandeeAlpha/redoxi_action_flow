#pragma once

#include <Eigen/Dense>
#include <unsupported/Eigen/CXX11/Tensor>

namespace redoxi_works::inference
{
template <int Ndim, typename ElementType>
using Tensor_nd_dtype = Eigen::Tensor<ElementType, Ndim, Eigen::RowMajor>;

// fixed tensor, row major
using Tensor_5d_f32 = Tensor_nd_dtype<5, float>;
using Tensor_4d_f32 = Tensor_nd_dtype<4, float>;
using Tensor_3d_f32 = Tensor_nd_dtype<3, float>;
using Tensor_2d_f32 = Tensor_nd_dtype<2, float>;
using Tensor_1d_f32 = Tensor_nd_dtype<1, float>;
using TensorView_5d_f32 = Eigen::TensorMap<Tensor_5d_f32>;
using TensorView_4d_f32 = Eigen::TensorMap<Tensor_4d_f32>;
using TensorView_3d_f32 = Eigen::TensorMap<Tensor_3d_f32>;
using TensorView_2d_f32 = Eigen::TensorMap<Tensor_2d_f32>;
using TensorView_1d_f32 = Eigen::TensorMap<Tensor_1d_f32>;

// fixed tensor, row major
using Tensor_5d_f16 = Tensor_nd_dtype<5, Eigen::half>;
using Tensor_4d_f16 = Tensor_nd_dtype<4, Eigen::half>;
using Tensor_3d_f16 = Tensor_nd_dtype<3, Eigen::half>;
using Tensor_2d_f16 = Tensor_nd_dtype<2, Eigen::half>;
using Tensor_1d_f16 = Tensor_nd_dtype<1, Eigen::half>;
using TensorView_5d_f16 = Eigen::TensorMap<Tensor_5d_f16>;
using TensorView_4d_f16 = Eigen::TensorMap<Tensor_4d_f16>;
using TensorView_3d_f16 = Eigen::TensorMap<Tensor_3d_f16>;
using TensorView_2d_f16 = Eigen::TensorMap<Tensor_2d_f16>;
using TensorView_1d_f16 = Eigen::TensorMap<Tensor_1d_f16>;

// fixed tensor, row major
using Tensor_5d_i32 = Tensor_nd_dtype<5, int32_t>;
using Tensor_4d_i32 = Tensor_nd_dtype<4, int32_t>;
using Tensor_3d_i32 = Tensor_nd_dtype<3, int32_t>;
using Tensor_2d_i32 = Tensor_nd_dtype<2, int32_t>;
using Tensor_1d_i32 = Tensor_nd_dtype<1, int32_t>;
using TensorView_5d_i32 = Eigen::TensorMap<Tensor_5d_i32>;
using TensorView_4d_i32 = Eigen::TensorMap<Tensor_4d_i32>;
using TensorView_3d_i32 = Eigen::TensorMap<Tensor_3d_i32>;
using TensorView_2d_i32 = Eigen::TensorMap<Tensor_2d_i32>;
using TensorView_1d_i32 = Eigen::TensorMap<Tensor_1d_i32>;

// fixed tensor, row major
using Tensor_5d_u8 = Tensor_nd_dtype<5, uint8_t>;
using Tensor_4d_u8 = Tensor_nd_dtype<4, uint8_t>;
using Tensor_3d_u8 = Tensor_nd_dtype<3, uint8_t>;
using Tensor_2d_u8 = Tensor_nd_dtype<2, uint8_t>;
using Tensor_1d_u8 = Tensor_nd_dtype<1, uint8_t>;
using TensorView_5d_u8 = Eigen::TensorMap<Tensor_5d_u8>;
using TensorView_4d_u8 = Eigen::TensorMap<Tensor_4d_u8>;
using TensorView_3d_u8 = Eigen::TensorMap<Tensor_3d_u8>;
using TensorView_2d_u8 = Eigen::TensorMap<Tensor_2d_u8>;
using TensorView_1d_u8 = Eigen::TensorMap<Tensor_1d_u8>;
} // namespace redoxi_works::inference