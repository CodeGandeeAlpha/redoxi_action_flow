#ifndef REDOXI_SHM_V6D__VISIBILITY_CONTROL_H_
#define REDOXI_SHM_V6D__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define REDOXI_SHM_V6D_EXPORT __attribute__ ((dllexport))
    #define REDOXI_SHM_V6D_IMPORT __attribute__ ((dllimport))
  #else
    #define REDOXI_SHM_V6D_EXPORT __declspec(dllexport)
    #define REDOXI_SHM_V6D_IMPORT __declspec(dllimport)
  #endif
  #ifdef REDOXI_SHM_V6D_BUILDING_LIBRARY
    #define REDOXI_SHM_V6D_PUBLIC REDOXI_SHM_V6D_EXPORT
  #else
    #define REDOXI_SHM_V6D_PUBLIC REDOXI_SHM_V6D_IMPORT
  #endif
  #define REDOXI_SHM_V6D_PUBLIC_TYPE REDOXI_SHM_V6D_PUBLIC
  #define REDOXI_SHM_V6D_LOCAL
#else
  #define REDOXI_SHM_V6D_EXPORT __attribute__ ((visibility("default")))
  #define REDOXI_SHM_V6D_IMPORT
  #if __GNUC__ >= 4
    #define REDOXI_SHM_V6D_PUBLIC __attribute__ ((visibility("default")))
    #define REDOXI_SHM_V6D_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define REDOXI_SHM_V6D_PUBLIC
    #define REDOXI_SHM_V6D_LOCAL
  #endif
  #define REDOXI_SHM_V6D_PUBLIC_TYPE
#endif

#endif  // REDOXI_SHM_V6D__VISIBILITY_CONTROL_H_
