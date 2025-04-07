#ifndef UNIVERSAL_MOT_TRACKERS__VISIBILITY_CONTROL_H_
#define UNIVERSAL_MOT_TRACKERS__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define UNIVERSAL_MOT_TRACKERS_EXPORT __attribute__ ((dllexport))
    #define UNIVERSAL_MOT_TRACKERS_IMPORT __attribute__ ((dllimport))
  #else
    #define UNIVERSAL_MOT_TRACKERS_EXPORT __declspec(dllexport)
    #define UNIVERSAL_MOT_TRACKERS_IMPORT __declspec(dllimport)
  #endif
  #ifdef UNIVERSAL_MOT_TRACKERS_BUILDING_LIBRARY
    #define UNIVERSAL_MOT_TRACKERS_PUBLIC UNIVERSAL_MOT_TRACKERS_EXPORT
  #else
    #define UNIVERSAL_MOT_TRACKERS_PUBLIC UNIVERSAL_MOT_TRACKERS_IMPORT
  #endif
  #define UNIVERSAL_MOT_TRACKERS_PUBLIC_TYPE UNIVERSAL_MOT_TRACKERS_PUBLIC
  #define UNIVERSAL_MOT_TRACKERS_LOCAL
#else
  #define UNIVERSAL_MOT_TRACKERS_EXPORT __attribute__ ((visibility("default")))
  #define UNIVERSAL_MOT_TRACKERS_IMPORT
  #if __GNUC__ >= 4
    #define UNIVERSAL_MOT_TRACKERS_PUBLIC __attribute__ ((visibility("default")))
    #define UNIVERSAL_MOT_TRACKERS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define UNIVERSAL_MOT_TRACKERS_PUBLIC
    #define UNIVERSAL_MOT_TRACKERS_LOCAL
  #endif
  #define UNIVERSAL_MOT_TRACKERS_PUBLIC_TYPE
#endif

#endif  // UNIVERSAL_MOT_TRACKERS__VISIBILITY_CONTROL_H_
