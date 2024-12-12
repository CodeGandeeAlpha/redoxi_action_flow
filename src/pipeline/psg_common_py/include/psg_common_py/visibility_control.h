#ifndef PSG_COMMON_PY__VISIBILITY_CONTROL_H_
#define PSG_COMMON_PY__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define PSG_COMMON_PY_EXPORT __attribute__ ((dllexport))
    #define PSG_COMMON_PY_IMPORT __attribute__ ((dllimport))
  #else
    #define PSG_COMMON_PY_EXPORT __declspec(dllexport)
    #define PSG_COMMON_PY_IMPORT __declspec(dllimport)
  #endif
  #ifdef PSG_COMMON_PY_BUILDING_LIBRARY
    #define PSG_COMMON_PY_PUBLIC PSG_COMMON_PY_EXPORT
  #else
    #define PSG_COMMON_PY_PUBLIC PSG_COMMON_PY_IMPORT
  #endif
  #define PSG_COMMON_PY_PUBLIC_TYPE PSG_COMMON_PY_PUBLIC
  #define PSG_COMMON_PY_LOCAL
#else
  #define PSG_COMMON_PY_EXPORT __attribute__ ((visibility("default")))
  #define PSG_COMMON_PY_IMPORT
  #if __GNUC__ >= 4
    #define PSG_COMMON_PY_PUBLIC __attribute__ ((visibility("default")))
    #define PSG_COMMON_PY_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define PSG_COMMON_PY_PUBLIC
    #define PSG_COMMON_PY_LOCAL
  #endif
  #define PSG_COMMON_PY_PUBLIC_TYPE
#endif

#endif  // PSG_COMMON_PY__VISIBILITY_CONTROL_H_
