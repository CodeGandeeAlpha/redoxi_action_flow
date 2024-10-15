#ifndef REDOXI_VIDEO_READER_BASE__VISIBILITY_CONTROL_H_
#define REDOXI_VIDEO_READER_BASE__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define REDOXI_VIDEO_READER_BASE_EXPORT __attribute__ ((dllexport))
    #define REDOXI_VIDEO_READER_BASE_IMPORT __attribute__ ((dllimport))
  #else
    #define REDOXI_VIDEO_READER_BASE_EXPORT __declspec(dllexport)
    #define REDOXI_VIDEO_READER_BASE_IMPORT __declspec(dllimport)
  #endif
  #ifdef REDOXI_VIDEO_READER_BASE_BUILDING_LIBRARY
    #define REDOXI_VIDEO_READER_BASE_PUBLIC REDOXI_VIDEO_READER_BASE_EXPORT
  #else
    #define REDOXI_VIDEO_READER_BASE_PUBLIC REDOXI_VIDEO_READER_BASE_IMPORT
  #endif
  #define REDOXI_VIDEO_READER_BASE_PUBLIC_TYPE REDOXI_VIDEO_READER_BASE_PUBLIC
  #define REDOXI_VIDEO_READER_BASE_LOCAL
#else
  #define REDOXI_VIDEO_READER_BASE_EXPORT __attribute__ ((visibility("default")))
  #define REDOXI_VIDEO_READER_BASE_IMPORT
  #if __GNUC__ >= 4
    #define REDOXI_VIDEO_READER_BASE_PUBLIC __attribute__ ((visibility("default")))
    #define REDOXI_VIDEO_READER_BASE_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define REDOXI_VIDEO_READER_BASE_PUBLIC
    #define REDOXI_VIDEO_READER_BASE_LOCAL
  #endif
  #define REDOXI_VIDEO_READER_BASE_PUBLIC_TYPE
#endif

#endif  // REDOXI_VIDEO_READER_BASE__VISIBILITY_CONTROL_H_
