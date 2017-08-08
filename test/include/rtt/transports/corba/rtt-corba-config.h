#ifndef RTT_CORBA_CONFIG_H
#define RTT_CORBA_CONFIG_H

#include "../../rtt-config.h"

#define RTT_CORBA_IMPLEMENTATION OMNIORB

/* #undef CORBA_IS_TAO */
#define CORBA_IS_OMNIORB 1

#if !defined( CORBA_IS_TAO ) && !defined( CORBA_IS_OMNIORB )
#  error "Configuration error: neither CORBA_IS_TAO nor CORBA_IS_OMNIORB is defined in rtt-corba-config.h"
#endif

/* #undef CORBA_TAO_HAS_MESSAGING */

//
// See: <http://gcc.gnu.org/wiki/Visibility>
//
#define RTT_GCC_HASVISIBILITY
#if defined(__GNUG__) && defined(RTT_GCC_HASVISIBILITY) && (defined(__unix__) || defined(__APPLE__))

# if defined(RTT_CORBA_DLL_EXPORT)
   // Use RTT_CORBA_API for normal function exporting
#  define RTT_CORBA_API    __attribute__((visibility("default")))

   // Use RTT_CORBA_EXPORT for static template class member variables
   // They must always be 'globally' visible.
#  define RTT_CORBA_EXPORT __attribute__((visibility("default")))

   // Use RTT_CORBA_HIDE to explicitly hide a symbol
#  define RTT_CORBA_HIDE   __attribute__((visibility("hidden")))

# else
#  define RTT_CORBA_API
#  define RTT_CORBA_EXPORT __attribute__((visibility("default")))
#  define RTT_CORBA_HIDE   __attribute__((visibility("hidden")))
# endif
#else
   // NOT GNU
# if defined( __MINGW__ ) || defined( WIN32 )
#  if defined(RTT_CORBA_DLL_EXPORT)
#   define RTT_CORBA_API    __declspec(dllexport)
#   define RTT_CORBA_EXPORT __declspec(dllexport)
#   define RTT_CORBA_HIDE   
#  else
#   define RTT_CORBA_API	 __declspec(dllimport)
#   define RTT_CORBA_EXPORT __declspec(dllexport)
#   define RTT_CORBA_HIDE 
#  endif
# else
#  define RTT_CORBA_API
#  define RTT_CORBA_EXPORT
#  define RTT_CORBA_HIDE
# endif
#endif

#endif

