#ifndef PSG_PERSON_GENERATOR__VISIBILITY_CONTROL_H_
#define PSG_PERSON_GENERATOR__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define PSG_PERSON_GENERATOR_EXPORT __attribute__ ((dllexport))
    #define PSG_PERSON_GENERATOR_IMPORT __attribute__ ((dllimport))
  #else
    #define PSG_PERSON_GENERATOR_EXPORT __declspec(dllexport)
    #define PSG_PERSON_GENERATOR_IMPORT __declspec(dllimport)
  #endif
  #ifdef PSG_PERSON_GENERATOR_BUILDING_LIBRARY
    #define PSG_PERSON_GENERATOR_PUBLIC PSG_PERSON_GENERATOR_EXPORT
  #else
    #define PSG_PERSON_GENERATOR_PUBLIC PSG_PERSON_GENERATOR_IMPORT
  #endif
  #define PSG_PERSON_GENERATOR_PUBLIC_TYPE PSG_PERSON_GENERATOR_PUBLIC
  #define PSG_PERSON_GENERATOR_LOCAL
#else
  #define PSG_PERSON_GENERATOR_EXPORT __attribute__ ((visibility("default")))
  #define PSG_PERSON_GENERATOR_IMPORT
  #if __GNUC__ >= 4
    #define PSG_PERSON_GENERATOR_PUBLIC __attribute__ ((visibility("default")))
    #define PSG_PERSON_GENERATOR_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define PSG_PERSON_GENERATOR_PUBLIC
    #define PSG_PERSON_GENERATOR_LOCAL
  #endif
  #define PSG_PERSON_GENERATOR_PUBLIC_TYPE
#endif

#endif  // PSG_PERSON_GENERATOR__VISIBILITY_CONTROL_H_
