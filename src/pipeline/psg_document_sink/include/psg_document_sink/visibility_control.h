#ifndef PSG_DOCUMENT_SINK__VISIBILITY_CONTROL_H_
#define PSG_DOCUMENT_SINK__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define PSG_DOCUMENT_SINK_EXPORT __attribute__ ((dllexport))
    #define PSG_DOCUMENT_SINK_IMPORT __attribute__ ((dllimport))
  #else
    #define PSG_DOCUMENT_SINK_EXPORT __declspec(dllexport)
    #define PSG_DOCUMENT_SINK_IMPORT __declspec(dllimport)
  #endif
  #ifdef PSG_DOCUMENT_SINK_BUILDING_LIBRARY
    #define PSG_DOCUMENT_SINK_PUBLIC PSG_DOCUMENT_SINK_EXPORT
  #else
    #define PSG_DOCUMENT_SINK_PUBLIC PSG_DOCUMENT_SINK_IMPORT
  #endif
  #define PSG_DOCUMENT_SINK_PUBLIC_TYPE PSG_DOCUMENT_SINK_PUBLIC
  #define PSG_DOCUMENT_SINK_LOCAL
#else
  #define PSG_DOCUMENT_SINK_EXPORT __attribute__ ((visibility("default")))
  #define PSG_DOCUMENT_SINK_IMPORT
  #if __GNUC__ >= 4
    #define PSG_DOCUMENT_SINK_PUBLIC __attribute__ ((visibility("default")))
    #define PSG_DOCUMENT_SINK_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define PSG_DOCUMENT_SINK_PUBLIC
    #define PSG_DOCUMENT_SINK_LOCAL
  #endif
  #define PSG_DOCUMENT_SINK_PUBLIC_TYPE
#endif

#endif  // PSG_DOCUMENT_SINK__VISIBILITY_CONTROL_H_
