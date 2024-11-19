#ifndef PSG_COMMON__VISIBILITY_CONTROL_H_
#define PSG_COMMON__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define PSG_COMMON_EXPORT __attribute__ ((dllexport))
    #define PSG_COMMON_IMPORT __attribute__ ((dllimport))
  #else
    #define PSG_COMMON_EXPORT __declspec(dllexport)
    #define PSG_COMMON_IMPORT __declspec(dllimport)
  #endif
  #ifdef PSG_COMMON_BUILDING_LIBRARY
    #define PSG_COMMON_PUBLIC PSG_COMMON_EXPORT
  #else
    #define PSG_COMMON_PUBLIC PSG_COMMON_IMPORT
  #endif
  #define PSG_COMMON_PUBLIC_TYPE PSG_COMMON_PUBLIC
  #define PSG_COMMON_LOCAL
#else
  #define PSG_COMMON_EXPORT __attribute__ ((visibility("default")))
  #define PSG_COMMON_IMPORT
  #if __GNUC__ >= 4
    #define PSG_COMMON_PUBLIC __attribute__ ((visibility("default")))
    #define PSG_COMMON_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define PSG_COMMON_PUBLIC
    #define PSG_COMMON_LOCAL
  #endif
  #define PSG_COMMON_PUBLIC_TYPE
#endif

#endif  // PSG_COMMON__VISIBILITY_CONTROL_H_
