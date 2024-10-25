#ifndef REDOXI_SAMPLES_LIB__VISIBILITY_CONTROL_H_
#define REDOXI_SAMPLES_LIB__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define REDOXI_SAMPLES_LIB_EXPORT __attribute__ ((dllexport))
    #define REDOXI_SAMPLES_LIB_IMPORT __attribute__ ((dllimport))
  #else
    #define REDOXI_SAMPLES_LIB_EXPORT __declspec(dllexport)
    #define REDOXI_SAMPLES_LIB_IMPORT __declspec(dllimport)
  #endif
  #ifdef REDOXI_SAMPLES_LIB_BUILDING_LIBRARY
    #define REDOXI_SAMPLES_LIB_PUBLIC REDOXI_SAMPLES_LIB_EXPORT
  #else
    #define REDOXI_SAMPLES_LIB_PUBLIC REDOXI_SAMPLES_LIB_IMPORT
  #endif
  #define REDOXI_SAMPLES_LIB_PUBLIC_TYPE REDOXI_SAMPLES_LIB_PUBLIC
  #define REDOXI_SAMPLES_LIB_LOCAL
#else
  #define REDOXI_SAMPLES_LIB_EXPORT __attribute__ ((visibility("default")))
  #define REDOXI_SAMPLES_LIB_IMPORT
  #if __GNUC__ >= 4
    #define REDOXI_SAMPLES_LIB_PUBLIC __attribute__ ((visibility("default")))
    #define REDOXI_SAMPLES_LIB_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define REDOXI_SAMPLES_LIB_PUBLIC
    #define REDOXI_SAMPLES_LIB_LOCAL
  #endif
  #define REDOXI_SAMPLES_LIB_PUBLIC_TYPE
#endif

#endif  // REDOXI_SAMPLES_LIB__VISIBILITY_CONTROL_H_
