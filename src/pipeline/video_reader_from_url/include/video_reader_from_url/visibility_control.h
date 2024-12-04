#ifndef VIDEO_READER_FROM_URL__VISIBILITY_CONTROL_H_
#define VIDEO_READER_FROM_URL__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define VIDEO_READER_FROM_URL_EXPORT __attribute__ ((dllexport))
    #define VIDEO_READER_FROM_URL_IMPORT __attribute__ ((dllimport))
  #else
    #define VIDEO_READER_FROM_URL_EXPORT __declspec(dllexport)
    #define VIDEO_READER_FROM_URL_IMPORT __declspec(dllimport)
  #endif
  #ifdef VIDEO_READER_FROM_URL_BUILDING_LIBRARY
    #define VIDEO_READER_FROM_URL_PUBLIC VIDEO_READER_FROM_URL_EXPORT
  #else
    #define VIDEO_READER_FROM_URL_PUBLIC VIDEO_READER_FROM_URL_IMPORT
  #endif
  #define VIDEO_READER_FROM_URL_PUBLIC_TYPE VIDEO_READER_FROM_URL_PUBLIC
  #define VIDEO_READER_FROM_URL_LOCAL
#else
  #define VIDEO_READER_FROM_URL_EXPORT __attribute__ ((visibility("default")))
  #define VIDEO_READER_FROM_URL_IMPORT
  #if __GNUC__ >= 4
    #define VIDEO_READER_FROM_URL_PUBLIC __attribute__ ((visibility("default")))
    #define VIDEO_READER_FROM_URL_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define VIDEO_READER_FROM_URL_PUBLIC
    #define VIDEO_READER_FROM_URL_LOCAL
  #endif
  #define VIDEO_READER_FROM_URL_PUBLIC_TYPE
#endif

#endif  // VIDEO_READER_FROM_URL__VISIBILITY_CONTROL_H_
