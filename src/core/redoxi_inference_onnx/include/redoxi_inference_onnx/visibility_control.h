#ifndef REDOXI_INFERENCE_ONNX__VISIBILITY_CONTROL_H_
#define REDOXI_INFERENCE_ONNX__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define REDOXI_INFERENCE_ONNX_EXPORT __attribute__ ((dllexport))
    #define REDOXI_INFERENCE_ONNX_IMPORT __attribute__ ((dllimport))
  #else
    #define REDOXI_INFERENCE_ONNX_EXPORT __declspec(dllexport)
    #define REDOXI_INFERENCE_ONNX_IMPORT __declspec(dllimport)
  #endif
  #ifdef REDOXI_INFERENCE_ONNX_BUILDING_LIBRARY
    #define REDOXI_INFERENCE_ONNX_PUBLIC REDOXI_INFERENCE_ONNX_EXPORT
  #else
    #define REDOXI_INFERENCE_ONNX_PUBLIC REDOXI_INFERENCE_ONNX_IMPORT
  #endif
  #define REDOXI_INFERENCE_ONNX_PUBLIC_TYPE REDOXI_INFERENCE_ONNX_PUBLIC
  #define REDOXI_INFERENCE_ONNX_LOCAL
#else
  #define REDOXI_INFERENCE_ONNX_EXPORT __attribute__ ((visibility("default")))
  #define REDOXI_INFERENCE_ONNX_IMPORT
  #if __GNUC__ >= 4
    #define REDOXI_INFERENCE_ONNX_PUBLIC __attribute__ ((visibility("default")))
    #define REDOXI_INFERENCE_ONNX_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define REDOXI_INFERENCE_ONNX_PUBLIC
    #define REDOXI_INFERENCE_ONNX_LOCAL
  #endif
  #define REDOXI_INFERENCE_ONNX_PUBLIC_TYPE
#endif

#endif  // REDOXI_INFERENCE_ONNX__VISIBILITY_CONTROL_H_
