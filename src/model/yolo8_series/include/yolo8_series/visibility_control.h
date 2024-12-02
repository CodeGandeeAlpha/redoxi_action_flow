#ifndef YOLO8_SERIES__VISIBILITY_CONTROL_H_
#define YOLO8_SERIES__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define YOLO8_SERIES_EXPORT __attribute__ ((dllexport))
    #define YOLO8_SERIES_IMPORT __attribute__ ((dllimport))
  #else
    #define YOLO8_SERIES_EXPORT __declspec(dllexport)
    #define YOLO8_SERIES_IMPORT __declspec(dllimport)
  #endif
  #ifdef YOLO8_SERIES_BUILDING_LIBRARY
    #define YOLO8_SERIES_PUBLIC YOLO8_SERIES_EXPORT
  #else
    #define YOLO8_SERIES_PUBLIC YOLO8_SERIES_IMPORT
  #endif
  #define YOLO8_SERIES_PUBLIC_TYPE YOLO8_SERIES_PUBLIC
  #define YOLO8_SERIES_LOCAL
#else
  #define YOLO8_SERIES_EXPORT __attribute__ ((visibility("default")))
  #define YOLO8_SERIES_IMPORT
  #if __GNUC__ >= 4
    #define YOLO8_SERIES_PUBLIC __attribute__ ((visibility("default")))
    #define YOLO8_SERIES_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define YOLO8_SERIES_PUBLIC
    #define YOLO8_SERIES_LOCAL
  #endif
  #define YOLO8_SERIES_PUBLIC_TYPE
#endif

#endif  // YOLO8_SERIES__VISIBILITY_CONTROL_H_
