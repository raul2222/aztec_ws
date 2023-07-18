#ifndef LIBSYNEXENS3_SY3_MACROS_H
#define LIBSYNEXENS3_SY3_MACROS_H

#include<stdio.h>
#include<stdint.h>
#include <string.h>

#define  SY3_NAMESPACE  sy3


#if defined(_MSC_VER)  & defined(LIBSYNEXENS3_EXPORTS)
    #define SY3_IMPORT __declspec(dllimport)
    #define SY3_EXPORT __declspec(dllexport)
 
#elif __GNUC__ >= 4
    #define SY3_IMPORT __attribute__ ((visibility("default")))
    #define SY3_EXPORT __attribute__ ((visibility("default")))
    #define SY3_LOCAL  __attribute__ ((visibility("hidden")))
#else
    #define SY3_IMPORT
    #define SY3_EXPORT
#endif

// Ignore warnings about import/exports when deriving from std classes.
#ifdef _MSC_VER
  #pragma warning(disable: 4251)
  #pragma warning(disable: 4275)
#endif

#define filename_macros(x) strrchr(x, '\\') ? strrchr(x, '\\') + 1 : x

#define LOG_WARNING(format,...) printf("sy3_warning %-20s fun:%s(%-3d) " format "\n", filename_macros(__FILE__),__func__,__LINE__,##__VA_ARGS__)

//#define SY3_DEBUG
//#define DEBUG_DUMP 1
#define DEBUG_MARK

#define LOG_ERROR(format,...)   printf("sy3_error   %-20s fun:%s(%-3d) " format "\n", filename_macros(__FILE__),__func__, __LINE__,  ##__VA_ARGS__)


#ifdef SY3_DEBUG
	#define LOG_INFO(format,...)    printf("sy3_info    %-20s fun:%s(%-3d) " format "\n", filename_macros(__FILE__),__func__, __LINE__, ##__VA_ARGS__)
	#define LOG_DEBUG(format,...)   printf("sy3_debug   %-20s fun:%s(%-3d) " format "\n", filename_macros(__FILE__),__func__, __LINE__,##__VA_ARGS__)

	#define LOG_FATAL(format,...)	printf("sy3_fatal   %-20s fun:%s(%-3d) " format "\n", filename_macros(__FILE__),__func__,__LINE__, ##__VA_ARGS__)
#else
	#define LOG_INFO(format,...)     
	#define LOG_DEBUG(format,...)    
	#define LOG_FATAL(format,...)	 

#endif// DEBUG


#define  MARK_ENTER()

#ifdef DEBUG_MARK
	#define MARK_ENTER()  LOG_INFO("enter\n");
#endif


#endif


