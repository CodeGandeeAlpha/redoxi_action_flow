#ifndef VIDEO_READER_ORBBEC__VISIBILITY_CONTROL_H_
#define VIDEO_READER_ORBBEC__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define VIDEO_READER_ORBBEC_EXPORT __attribute__ ((dllexport))
    #define VIDEO_READER_ORBBEC_IMPORT __attribute__ ((dllimport))
  #else
    #define VIDEO_READER_ORBBEC_EXPORT __declspec(dllexport)
    #define VIDEO_READER_ORBBEC_IMPORT __declspec(dllimport)
  #endif
  #ifdef VIDEO_READER_ORBBEC_BUILDING_LIBRARY
    #define VIDEO_READER_ORBBEC_PUBLIC VIDEO_READER_ORBBEC_EXPORT
  #else
    #define VIDEO_READER_ORBBEC_PUBLIC VIDEO_READER_ORBBEC_IMPORT
  #endif
  #define VIDEO_READER_ORBBEC_PUBLIC_TYPE VIDEO_READER_ORBBEC_PUBLIC
  #define VIDEO_READER_ORBBEC_LOCAL
#else
  #define VIDEO_READER_ORBBEC_EXPORT __attribute__ ((visibility("default")))
  #define VIDEO_READER_ORBBEC_IMPORT
  #if __GNUC__ >= 4
    #define VIDEO_READER_ORBBEC_PUBLIC __attribute__ ((visibility("default")))
    #define VIDEO_READER_ORBBEC_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define VIDEO_READER_ORBBEC_PUBLIC
    #define VIDEO_READER_ORBBEC_LOCAL
  #endif
  #define VIDEO_READER_ORBBEC_PUBLIC_TYPE
#endif

#endif  // VIDEO_READER_ORBBEC__VISIBILITY_CONTROL_H_
