#ifndef YOLO8_BODY_POSE_DETECTOR__VISIBILITY_CONTROL_H_
#define YOLO8_BODY_POSE_DETECTOR__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define YOLO8_BODY_POSE_DETECTOR_EXPORT __attribute__ ((dllexport))
    #define YOLO8_BODY_POSE_DETECTOR_IMPORT __attribute__ ((dllimport))
  #else
    #define YOLO8_BODY_POSE_DETECTOR_EXPORT __declspec(dllexport)
    #define YOLO8_BODY_POSE_DETECTOR_IMPORT __declspec(dllimport)
  #endif
  #ifdef YOLO8_BODY_POSE_DETECTOR_BUILDING_LIBRARY
    #define YOLO8_BODY_POSE_DETECTOR_PUBLIC YOLO8_BODY_POSE_DETECTOR_EXPORT
  #else
    #define YOLO8_BODY_POSE_DETECTOR_PUBLIC YOLO8_BODY_POSE_DETECTOR_IMPORT
  #endif
  #define YOLO8_BODY_POSE_DETECTOR_PUBLIC_TYPE YOLO8_BODY_POSE_DETECTOR_PUBLIC
  #define YOLO8_BODY_POSE_DETECTOR_LOCAL
#else
  #define YOLO8_BODY_POSE_DETECTOR_EXPORT __attribute__ ((visibility("default")))
  #define YOLO8_BODY_POSE_DETECTOR_IMPORT
  #if __GNUC__ >= 4
    #define YOLO8_BODY_POSE_DETECTOR_PUBLIC __attribute__ ((visibility("default")))
    #define YOLO8_BODY_POSE_DETECTOR_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define YOLO8_BODY_POSE_DETECTOR_PUBLIC
    #define YOLO8_BODY_POSE_DETECTOR_LOCAL
  #endif
  #define YOLO8_BODY_POSE_DETECTOR_PUBLIC_TYPE
#endif

#endif  // YOLO8_BODY_POSE_DETECTOR__VISIBILITY_CONTROL_H_
