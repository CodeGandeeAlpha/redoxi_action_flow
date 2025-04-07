#ifndef REDOXI_DNN_MODELS__VISIBILITY_CONTROL_H_
#define REDOXI_DNN_MODELS__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define REDOXI_DNN_MODELS_EXPORT __attribute__ ((dllexport))
    #define REDOXI_DNN_MODELS_IMPORT __attribute__ ((dllimport))
  #else
    #define REDOXI_DNN_MODELS_EXPORT __declspec(dllexport)
    #define REDOXI_DNN_MODELS_IMPORT __declspec(dllimport)
  #endif
  #ifdef REDOXI_DNN_MODELS_BUILDING_LIBRARY
    #define REDOXI_DNN_MODELS_PUBLIC REDOXI_DNN_MODELS_EXPORT
  #else
    #define REDOXI_DNN_MODELS_PUBLIC REDOXI_DNN_MODELS_IMPORT
  #endif
  #define REDOXI_DNN_MODELS_PUBLIC_TYPE REDOXI_DNN_MODELS_PUBLIC
  #define REDOXI_DNN_MODELS_LOCAL
#else
  #define REDOXI_DNN_MODELS_EXPORT __attribute__ ((visibility("default")))
  #define REDOXI_DNN_MODELS_IMPORT
  #if __GNUC__ >= 4
    #define REDOXI_DNN_MODELS_PUBLIC __attribute__ ((visibility("default")))
    #define REDOXI_DNN_MODELS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define REDOXI_DNN_MODELS_PUBLIC
    #define REDOXI_DNN_MODELS_LOCAL
  #endif
  #define REDOXI_DNN_MODELS_PUBLIC_TYPE
#endif

#endif  // REDOXI_DNN_MODELS__VISIBILITY_CONTROL_H_
