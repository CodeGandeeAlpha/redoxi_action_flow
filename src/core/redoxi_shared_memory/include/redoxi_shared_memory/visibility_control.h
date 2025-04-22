#ifndef REDOXI_SHARED_MEMORY__VISIBILITY_CONTROL_H_
#define REDOXI_SHARED_MEMORY__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define REDOXI_SHARED_MEMORY_EXPORT __attribute__ ((dllexport))
    #define REDOXI_SHARED_MEMORY_IMPORT __attribute__ ((dllimport))
  #else
    #define REDOXI_SHARED_MEMORY_EXPORT __declspec(dllexport)
    #define REDOXI_SHARED_MEMORY_IMPORT __declspec(dllimport)
  #endif
  #ifdef REDOXI_SHARED_MEMORY_BUILDING_LIBRARY
    #define REDOXI_SHARED_MEMORY_PUBLIC REDOXI_SHARED_MEMORY_EXPORT
  #else
    #define REDOXI_SHARED_MEMORY_PUBLIC REDOXI_SHARED_MEMORY_IMPORT
  #endif
  #define REDOXI_SHARED_MEMORY_PUBLIC_TYPE REDOXI_SHARED_MEMORY_PUBLIC
  #define REDOXI_SHARED_MEMORY_LOCAL
#else
  #define REDOXI_SHARED_MEMORY_EXPORT __attribute__ ((visibility("default")))
  #define REDOXI_SHARED_MEMORY_IMPORT
  #if __GNUC__ >= 4
    #define REDOXI_SHARED_MEMORY_PUBLIC __attribute__ ((visibility("default")))
    #define REDOXI_SHARED_MEMORY_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define REDOXI_SHARED_MEMORY_PUBLIC
    #define REDOXI_SHARED_MEMORY_LOCAL
  #endif
  #define REDOXI_SHARED_MEMORY_PUBLIC_TYPE
#endif

#endif  // REDOXI_SHARED_MEMORY__VISIBILITY_CONTROL_H_
