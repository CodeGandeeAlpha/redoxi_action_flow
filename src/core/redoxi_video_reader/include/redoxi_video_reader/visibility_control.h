#ifndef REDOXI_VIDEO_READER__VISIBILITY_CONTROL_H_
#define REDOXI_VIDEO_READER__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#    ifdef __GNUC__
#        define REDOXI_VIDEO_READER_EXPORT __attribute__((dllexport))
#        define REDOXI_VIDEO_READER_IMPORT __attribute__((dllimport))
#    else
#        define REDOXI_VIDEO_READER_EXPORT __declspec(dllexport)
#        define REDOXI_VIDEO_READER_IMPORT __declspec(dllimport)
#    endif
#    ifdef REDOXI_VIDEO_READER_BUILDING_LIBRARY
#        define REDOXI_VIDEO_READER_PUBLIC REDOXI_VIDEO_READER_EXPORT
#    else
#        define REDOXI_VIDEO_READER_PUBLIC REDOXI_VIDEO_READER_IMPORT
#    endif
#    define REDOXI_VIDEO_READER_PUBLIC_TYPE REDOXI_VIDEO_READER_PUBLIC
#    define REDOXI_VIDEO_READER_LOCAL
#else
#    define REDOXI_VIDEO_READER_EXPORT __attribute__((visibility("default")))
#    define REDOXI_VIDEO_READER_IMPORT
#    if __GNUC__ >= 4
#        define REDOXI_VIDEO_READER_PUBLIC __attribute__((visibility("default")))
#        define REDOXI_VIDEO_READER_LOCAL __attribute__((visibility("hidden")))
#    else
#        define REDOXI_VIDEO_READER_PUBLIC
#        define REDOXI_VIDEO_READER_LOCAL
#    endif
#    define REDOXI_VIDEO_READER_PUBLIC_TYPE
#endif

#endif // REDOXI_VIDEO_READER__VISIBILITY_CONTROL_H_
