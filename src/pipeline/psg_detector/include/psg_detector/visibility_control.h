#ifndef PSG_DETECTOR__VISIBILITY_CONTROL_H_
#define PSG_DETECTOR__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define PSG_DETECTOR_EXPORT __attribute__ ((dllexport))
    #define PSG_DETECTOR_IMPORT __attribute__ ((dllimport))
  #else
    #define PSG_DETECTOR_EXPORT __declspec(dllexport)
    #define PSG_DETECTOR_IMPORT __declspec(dllimport)
  #endif
  #ifdef PSG_DETECTOR_BUILDING_LIBRARY
    #define PSG_DETECTOR_PUBLIC PSG_DETECTOR_EXPORT
  #else
    #define PSG_DETECTOR_PUBLIC PSG_DETECTOR_IMPORT
  #endif
  #define PSG_DETECTOR_PUBLIC_TYPE PSG_DETECTOR_PUBLIC
  #define PSG_DETECTOR_LOCAL
#else
  #define PSG_DETECTOR_EXPORT __attribute__ ((visibility("default")))
  #define PSG_DETECTOR_IMPORT
  #if __GNUC__ >= 4
    #define PSG_DETECTOR_PUBLIC __attribute__ ((visibility("default")))
    #define PSG_DETECTOR_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define PSG_DETECTOR_PUBLIC
    #define PSG_DETECTOR_LOCAL
  #endif
  #define PSG_DETECTOR_PUBLIC_TYPE
#endif

#endif  // PSG_DETECTOR__VISIBILITY_CONTROL_H_
