#ifndef DUA_BTCPP_NODES__VISIBILITY_H_
#define DUA_BTCPP_NODES__VISIBILITY_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define DUA_BTCPP_NODES_EXPORT __attribute__ ((dllexport))
    #define DUA_BTCPP_NODES_IMPORT __attribute__ ((dllimport))
  #else
    #define DUA_BTCPP_NODES_EXPORT __declspec(dllexport)
    #define DUA_BTCPP_NODES_IMPORT __declspec(dllimport)
  #endif
  #ifdef DUA_BTCPP_NODES_BUILDING_LIBRARY
    #define DUA_BTCPP_NODES_PUBLIC DUA_BTCPP_NODES_EXPORT
  #else
    #define DUA_BTCPP_NODES_PUBLIC DUA_BTCPP_NODES_IMPORT
  #endif
  #define DUA_BTCPP_NODES_PUBLIC_TYPE DUA_BTCPP_NODES_PUBLIC
  #define DUA_BTCPP_NODES_LOCAL
#else
  #define DUA_BTCPP_NODES_EXPORT __attribute__ ((visibility("default")))
  #define DUA_BTCPP_NODES_IMPORT
  #if __GNUC__ >= 4
    #define DUA_BTCPP_NODES_PUBLIC __attribute__ ((visibility("default")))
    #define DUA_BTCPP_NODES_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define DUA_BTCPP_NODES_PUBLIC
    #define DUA_BTCPP_NODES_LOCAL
  #endif
  #define DUA_BTCPP_NODES_PUBLIC_TYPE
#endif

#endif  // DUA_BTCPP_NODES__VISIBILITY_H_
