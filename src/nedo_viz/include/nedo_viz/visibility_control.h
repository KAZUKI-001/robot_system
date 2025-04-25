#ifndef NEDO_VIZ__VISIBILITY_CONTROL_H_
#define NEDO_VIZ__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define NEDO_VIZ_EXPORT __attribute__ ((dllexport))
    #define NEDO_VIZ_IMPORT __attribute__ ((dllimport))
  #else
    #define NEDO_VIZ_EXPORT __declspec(dllexport)
    #define NEDO_VIZ_IMPORT __declspec(dllimport)
  #endif
  #ifdef NEDO_VIZ_BUILDING_LIBRARY
    #define NEDO_VIZ_PUBLIC NEDO_VIZ_EXPORT
  #else
    #define NEDO_VIZ_PUBLIC NEDO_VIZ_IMPORT
  #endif
  #define NEDO_VIZ_PUBLIC_TYPE NEDO_VIZ_PUBLIC
  #define NEDO_VIZ_LOCAL
#else
  #define NEDO_VIZ_EXPORT __attribute__ ((visibility("default")))
  #define NEDO_VIZ_IMPORT
  #if __GNUC__ >= 4
    #define NEDO_VIZ_PUBLIC __attribute__ ((visibility("default")))
    #define NEDO_VIZ_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define NEDO_VIZ_PUBLIC
    #define NEDO_VIZ_LOCAL
  #endif
  #define NEDO_VIZ_PUBLIC_TYPE
#endif

#endif  // NEDO_VIZ__VISIBILITY_CONTROL_H_
