#ifndef STREAM_WORKER__VISIBILITY_CONTROL_H_
#define STREAM_WORKER__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define STREAM_WORKER_EXPORT __attribute__ ((dllexport))
    #define STREAM_WORKER_IMPORT __attribute__ ((dllimport))
  #else
    #define STREAM_WORKER_EXPORT __declspec(dllexport)
    #define STREAM_WORKER_IMPORT __declspec(dllimport)
  #endif
  #ifdef STREAM_WORKER_BUILDING_LIBRARY
    #define STREAM_WORKER_PUBLIC STREAM_WORKER_EXPORT
  #else
    #define STREAM_WORKER_PUBLIC STREAM_WORKER_IMPORT
  #endif
  #define STREAM_WORKER_PUBLIC_TYPE STREAM_WORKER_PUBLIC
  #define STREAM_WORKER_LOCAL
#else
  #define STREAM_WORKER_EXPORT __attribute__ ((visibility("default")))
  #define STREAM_WORKER_IMPORT
  #if __GNUC__ >= 4
    #define STREAM_WORKER_PUBLIC __attribute__ ((visibility("default")))
    #define STREAM_WORKER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define STREAM_WORKER_PUBLIC
    #define STREAM_WORKER_LOCAL
  #endif
  #define STREAM_WORKER_PUBLIC_TYPE
#endif

#endif  // STREAM_WORKER__VISIBILITY_CONTROL_H_
