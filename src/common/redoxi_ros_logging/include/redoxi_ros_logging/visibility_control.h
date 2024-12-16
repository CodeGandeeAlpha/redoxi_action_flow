#ifndef REDOXI_ROS_LOGGING__VISIBILITY_CONTROL_H_
#define REDOXI_ROS_LOGGING__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define REDOXI_ROS_LOGGING_EXPORT __attribute__ ((dllexport))
    #define REDOXI_ROS_LOGGING_IMPORT __attribute__ ((dllimport))
  #else
    #define REDOXI_ROS_LOGGING_EXPORT __declspec(dllexport)
    #define REDOXI_ROS_LOGGING_IMPORT __declspec(dllimport)
  #endif
  #ifdef REDOXI_ROS_LOGGING_BUILDING_LIBRARY
    #define REDOXI_ROS_LOGGING_PUBLIC REDOXI_ROS_LOGGING_EXPORT
  #else
    #define REDOXI_ROS_LOGGING_PUBLIC REDOXI_ROS_LOGGING_IMPORT
  #endif
  #define REDOXI_ROS_LOGGING_PUBLIC_TYPE REDOXI_ROS_LOGGING_PUBLIC
  #define REDOXI_ROS_LOGGING_LOCAL
#else
  #define REDOXI_ROS_LOGGING_EXPORT __attribute__ ((visibility("default")))
  #define REDOXI_ROS_LOGGING_IMPORT
  #if __GNUC__ >= 4
    #define REDOXI_ROS_LOGGING_PUBLIC __attribute__ ((visibility("default")))
    #define REDOXI_ROS_LOGGING_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define REDOXI_ROS_LOGGING_PUBLIC
    #define REDOXI_ROS_LOGGING_LOCAL
  #endif
  #define REDOXI_ROS_LOGGING_PUBLIC_TYPE
#endif

#endif  // REDOXI_ROS_LOGGING__VISIBILITY_CONTROL_H_
