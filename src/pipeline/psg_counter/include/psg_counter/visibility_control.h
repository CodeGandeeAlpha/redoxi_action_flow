#ifndef PSG_COUNTER__VISIBILITY_CONTROL_H_
#define PSG_COUNTER__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define PSG_COUNTER_EXPORT __attribute__ ((dllexport))
    #define PSG_COUNTER_IMPORT __attribute__ ((dllimport))
  #else
    #define PSG_COUNTER_EXPORT __declspec(dllexport)
    #define PSG_COUNTER_IMPORT __declspec(dllimport)
  #endif
  #ifdef PSG_COUNTER_BUILDING_LIBRARY
    #define PSG_COUNTER_PUBLIC PSG_COUNTER_EXPORT
  #else
    #define PSG_COUNTER_PUBLIC PSG_COUNTER_IMPORT
  #endif
  #define PSG_COUNTER_PUBLIC_TYPE PSG_COUNTER_PUBLIC
  #define PSG_COUNTER_LOCAL
#else
  #define PSG_COUNTER_EXPORT __attribute__ ((visibility("default")))
  #define PSG_COUNTER_IMPORT
  #if __GNUC__ >= 4
    #define PSG_COUNTER_PUBLIC __attribute__ ((visibility("default")))
    #define PSG_COUNTER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define PSG_COUNTER_PUBLIC
    #define PSG_COUNTER_LOCAL
  #endif
  #define PSG_COUNTER_PUBLIC_TYPE
#endif

#endif  // PSG_COUNTER__VISIBILITY_CONTROL_H_
