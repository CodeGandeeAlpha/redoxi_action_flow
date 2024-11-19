#ifndef PSG_MASTER_NODE__VISIBILITY_CONTROL_H_
#define PSG_MASTER_NODE__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#    ifdef __GNUC__
#        define PSG_MASTER_NODE_EXPORT __attribute__((dllexport))
#        define PSG_MASTER_NODE_IMPORT __attribute__((dllimport))
#    else
#        define PSG_MASTER_NODE_EXPORT __declspec(dllexport)
#        define PSG_MASTER_NODE_IMPORT __declspec(dllimport)
#    endif
#    ifdef PSG_MASTER_NODE_BUILDING_LIBRARY
#        define PSG_MASTER_NODE_PUBLIC PSG_MASTER_NODE_EXPORT
#    else
#        define PSG_MASTER_NODE_PUBLIC PSG_MASTER_NODE_IMPORT
#    endif
#    define PSG_MASTER_NODE_PUBLIC_TYPE PSG_MASTER_NODE_PUBLIC
#    define PSG_MASTER_NODE_LOCAL
#else
#    define PSG_MASTER_NODE_EXPORT __attribute__((visibility("default")))
#    define PSG_MASTER_NODE_IMPORT
#    if __GNUC__ >= 4
#        define PSG_MASTER_NODE_PUBLIC __attribute__((visibility("default")))
#        define PSG_MASTER_NODE_LOCAL __attribute__((visibility("hidden")))
#    else
#        define PSG_MASTER_NODE_PUBLIC
#        define PSG_MASTER_NODE_LOCAL
#    endif
#    define PSG_MASTER_NODE_PUBLIC_TYPE
#endif

#endif // PSG_MASTER_NODE__VISIBILITY_CONTROL_H_
