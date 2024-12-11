#ifndef PSG_ALL_DETECTOR_CPP__VISIBILITY_CONTROL_H_
#define PSG_ALL_DETECTOR_CPP__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define PSG_ALL_DETECTOR_CPP_EXPORT __attribute__ ((dllexport))
    #define PSG_ALL_DETECTOR_CPP_IMPORT __attribute__ ((dllimport))
  #else
    #define PSG_ALL_DETECTOR_CPP_EXPORT __declspec(dllexport)
    #define PSG_ALL_DETECTOR_CPP_IMPORT __declspec(dllimport)
  #endif
  #ifdef PSG_ALL_DETECTOR_CPP_BUILDING_LIBRARY
    #define PSG_ALL_DETECTOR_CPP_PUBLIC PSG_ALL_DETECTOR_CPP_EXPORT
  #else
    #define PSG_ALL_DETECTOR_CPP_PUBLIC PSG_ALL_DETECTOR_CPP_IMPORT
  #endif
  #define PSG_ALL_DETECTOR_CPP_PUBLIC_TYPE PSG_ALL_DETECTOR_CPP_PUBLIC
  #define PSG_ALL_DETECTOR_CPP_LOCAL
#else
  #define PSG_ALL_DETECTOR_CPP_EXPORT __attribute__ ((visibility("default")))
  #define PSG_ALL_DETECTOR_CPP_IMPORT
  #if __GNUC__ >= 4
    #define PSG_ALL_DETECTOR_CPP_PUBLIC __attribute__ ((visibility("default")))
    #define PSG_ALL_DETECTOR_CPP_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define PSG_ALL_DETECTOR_CPP_PUBLIC
    #define PSG_ALL_DETECTOR_CPP_LOCAL
  #endif
  #define PSG_ALL_DETECTOR_CPP_PUBLIC_TYPE
#endif

#endif  // PSG_ALL_DETECTOR_CPP__VISIBILITY_CONTROL_H_
