#pragma once

#include <redoxi_common_cpp/redoxi_common_cpp.hpp>
#include <opencv2/opencv.hpp>
#include <json_struct/json_struct.h>

JS_OBJ_EXT(cv::Size, width, height);

namespace redoxi_works
{

} // namespace redoxi_works

namespace JS
{


//! Type handler for cv::Size to be used with json_struct
// template <>
// struct TypeHandler<cv::Size> {
//     //! Serialize cv::Size to JSON
//     static inline JS::Error to(const cv::Size &value, ParseContext &context)
//     {
//         context.token.
//         return JS::Error::NoError;
//     }

//     //! Deserialize cv::Size from JSON
//     static inline JS::Error from(cv::Size &value, const JS::Deserializer &js)
//     {
//         js.read("width", value.width);
//         js.read("height", value.height);
//         return JS::Error::NoError;
//     }
// };
} // namespace JS