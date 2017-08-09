#ifndef RTT_CONFIG_H
#define RTT_CONFIG_H

#define RTT_VERSION       2.8.3
#define RTT_VERSION_MAJOR 2
#define RTT_VERSION_MINOR 8
#define RTT_VERSION_PATCH 3

#define RTT_VERSION_GTE(major,minor,patch) \
    ((RTT_VERSION_MAJOR > major) || (RTT_VERSION_MAJOR == major && \
     (RTT_VERSION_MINOR > minor) || (RTT_VERSION_MINOR == minor && \
     (RTT_VERSION_PATCH >= patch))))

#define ORO_DISABLE_PORT_DATA_SCRIPTING

// if not defined, show an error
#ifndef OROCOS_TARGET
// OROCOS_TARGET is only used to include the file targets/OROCOS_TARGET
#error "You _must_ define OROCOS_TARGET to an installed target (for example: -DOROCOS_TARGET=gnulinux )"
#endif

#include "os/targets/rtt-target.h"

// Detect the CPU we are compiling for
#if defined( __GNUC__ )
#define OROBLD_GCC_VERSION (__GNUC__ * 10000 \
                             + __GNUC_MINOR__ * 100 \
			     + __GNUC_PATCHLEVEL__)
#endif
# if defined( __GNUC__ ) && defined( __i386__ )
#  define OROBLD_OS_ARCH_i386
# elif defined( __GNUC__ ) && defined( __x86_64__ )
#  define OROBLD_OS_ARCH_x86_64
# elif defined( __GNUC__ ) && (defined( __powerpc__ ) || defined( __PPC__ ) )
#  define OROBLD_OS_ARCH_ppc
# elif defined( __GNUC__ ) && (defined( __arm__ ) || defined( __ARM__ ) )
#  define OROBLD_OS_ARCH_arm
# elif defined( __GNUC__ ) && defined( __ia64__ )
#  error "ia64 Is not yet supported, contact the orocos-dev mailinglist for further actions."
#  define OROBLD_OS_ARCH_ia64
# elif defined( __MINGW32__ ) && defined( __i386__ )
#  define OROBLD_OS_ARCH_i386
# elif defined( WIN32 )
#  define OROBLD_OS_ARCH_i386
# else
#  error "Unknown Processor Architecture"
#  define OROBLD_OS_ARCH_unknown
# endif


//
// See: <http://gcc.gnu.org/wiki/Visibility>
//
#define RTT_GCC_HASVISIBILITY
#if defined(__GNUG__) && defined(RTT_GCC_HASVISIBILITY) && (defined(__unix__) || defined(__APPLE__))

# if defined(RTT_DLL_EXPORT)
   // Use RTT_API for normal function exporting
#  define RTT_API    __attribute__((visibility("default")))

   // Use RTT_EXPORT for static template class member variables
   // They must always be 'globally' visible.
#  define RTT_EXPORT __attribute__((visibility("default")))

   // Use RTT_HIDE to explicitly hide a symbol
#  define RTT_HIDE   __attribute__((visibility("hidden")))

# else
#  define RTT_API
#  define RTT_EXPORT __attribute__((visibility("default")))
#  define RTT_HIDE   __attribute__((visibility("hidden")))
# endif
#else
   // Win32 and NOT GNU
# if defined( WIN32 ) && !defined ( __MINGW32__ )
#  if defined(RTT_DLL_EXPORT)
#   define RTT_API    __declspec(dllexport)
#   define RTT_EXPORT __declspec(dllexport)
#   define RTT_HIDE   
#  else
#   define RTT_API	 __declspec(dllimport)
#   define RTT_EXPORT __declspec(dllexport)
#   define RTT_HIDE 
#  endif
# else
#  define RTT_API
#  define RTT_EXPORT
#  define RTT_HIDE
# endif
#endif

#endif