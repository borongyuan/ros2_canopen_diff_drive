#ifndef DIFF_DRIVE__VISIBILITY_CONTROL_HPP_
#define DIFF_DRIVE__VISIBILITY_CONTROL_HPP_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define DIFF_DRIVE_EXPORT __attribute__((dllexport))
#define DIFF_DRIVE_IMPORT __attribute__((dllimport))
#else
#define DIFF_DRIVE_EXPORT __declspec(dllexport)
#define DIFF_DRIVE_IMPORT __declspec(dllimport)
#endif
#ifdef DIFF_DRIVE_BUILDING_DLL
#define DIFF_DRIVE_PUBLIC DIFF_DRIVE_EXPORT
#else
#define DIFF_DRIVE_PUBLIC DIFF_DRIVE_IMPORT
#endif
#define DIFF_DRIVE_PUBLIC_TYPE DIFF_DRIVE_PUBLIC
#define DIFF_DRIVE_LOCAL
#else
#define DIFF_DRIVE_EXPORT __attribute__((visibility("default")))
#define DIFF_DRIVE_IMPORT
#if __GNUC__ >= 4
#define DIFF_DRIVE_PUBLIC __attribute__((visibility("default")))
#define DIFF_DRIVE_LOCAL __attribute__((visibility("hidden")))
#else
#define DIFF_DRIVE_PUBLIC
#define DIFF_DRIVE_LOCAL
#endif
#define DIFF_DRIVE_PUBLIC_TYPE
#endif

#endif  // DIFF_DRIVE__VISIBILITY_CONTROL_HPP_
