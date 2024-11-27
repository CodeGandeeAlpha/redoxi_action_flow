#ifndef PSG_TRACKER__VISIBILITY_CONTROL_H_
#define PSG_TRACKER__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define PSG_TRACKER_EXPORT __attribute__ ((dllexport))
    #define PSG_TRACKER_IMPORT __attribute__ ((dllimport))
  #else
    #define PSG_TRACKER_EXPORT __declspec(dllexport)
    #define PSG_TRACKER_IMPORT __declspec(dllimport)
  #endif
  #ifdef PSG_TRACKER_BUILDING_LIBRARY
    #define PSG_TRACKER_PUBLIC PSG_TRACKER_EXPORT
  #else
    #define PSG_TRACKER_PUBLIC PSG_TRACKER_IMPORT
  #endif
  #define PSG_TRACKER_PUBLIC_TYPE PSG_TRACKER_PUBLIC
  #define PSG_TRACKER_LOCAL
#else
  #define PSG_TRACKER_EXPORT __attribute__ ((visibility("default")))
  #define PSG_TRACKER_IMPORT
  #if __GNUC__ >= 4
    #define PSG_TRACKER_PUBLIC __attribute__ ((visibility("default")))
    #define PSG_TRACKER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define PSG_TRACKER_PUBLIC
    #define PSG_TRACKER_LOCAL
  #endif
  #define PSG_TRACKER_PUBLIC_TYPE
#endif

#endif  // PSG_TRACKER__VISIBILITY_CONTROL_H_
