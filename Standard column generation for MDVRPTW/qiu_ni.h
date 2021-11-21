//
// MATLAB Compiler: 6.3 (R2016b)
// Date: Fri Nov 22 17:17:43 2019
// Arguments: "-B" "macro_default" "-W" "cpplib:qiu_ni" "-T" "link:lib"
// "qiu_ni.m" 
//

#ifndef __qiu_ni_h
#define __qiu_ni_h 1

#if defined(__cplusplus) && !defined(mclmcrrt_h) && defined(__linux__)
#  pragma implementation "mclmcrrt.h"
#endif
#include "mclmcrrt.h"
#include "mclcppclass.h"
#ifdef __cplusplus
extern "C" {
#endif

#if defined(__SUNPRO_CC)
/* Solaris shared libraries use __global, rather than mapfiles
 * to define the API exported from a shared library. __global is
 * only necessary when building the library -- files including
 * this header file to use the library do not need the __global
 * declaration; hence the EXPORTING_<library> logic.
 */

#ifdef EXPORTING_qiu_ni
#define PUBLIC_qiu_ni_C_API __global
#else
#define PUBLIC_qiu_ni_C_API /* No import statement needed. */
#endif

#define LIB_qiu_ni_C_API PUBLIC_qiu_ni_C_API

#elif defined(_HPUX_SOURCE)

#ifdef EXPORTING_qiu_ni
#define PUBLIC_qiu_ni_C_API __declspec(dllexport)
#else
#define PUBLIC_qiu_ni_C_API __declspec(dllimport)
#endif

#define LIB_qiu_ni_C_API PUBLIC_qiu_ni_C_API


#else

#define LIB_qiu_ni_C_API

#endif

/* This symbol is defined in shared libraries. Define it here
 * (to nothing) in case this isn't a shared library. 
 */
#ifndef LIB_qiu_ni_C_API 
#define LIB_qiu_ni_C_API /* No special import/export declaration */
#endif

extern LIB_qiu_ni_C_API 
bool MW_CALL_CONV qiu_niInitializeWithHandlers(
       mclOutputHandlerFcn error_handler, 
       mclOutputHandlerFcn print_handler);

extern LIB_qiu_ni_C_API 
bool MW_CALL_CONV qiu_niInitialize(void);

extern LIB_qiu_ni_C_API 
void MW_CALL_CONV qiu_niTerminate(void);



extern LIB_qiu_ni_C_API 
void MW_CALL_CONV qiu_niPrintStackTrace(void);

extern LIB_qiu_ni_C_API 
bool MW_CALL_CONV mlxQiu_ni(int nlhs, mxArray *plhs[], int nrhs, mxArray *prhs[]);


#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

/* On Windows, use __declspec to control the exported API */
#if defined(_MSC_VER) || defined(__BORLANDC__)

#ifdef EXPORTING_qiu_ni
#define PUBLIC_qiu_ni_CPP_API __declspec(dllexport)
#else
#define PUBLIC_qiu_ni_CPP_API __declspec(dllimport)
#endif

#define LIB_qiu_ni_CPP_API PUBLIC_qiu_ni_CPP_API

#else

#if !defined(LIB_qiu_ni_CPP_API)
#if defined(LIB_qiu_ni_C_API)
#define LIB_qiu_ni_CPP_API LIB_qiu_ni_C_API
#else
#define LIB_qiu_ni_CPP_API /* empty! */ 
#endif
#endif

#endif

extern LIB_qiu_ni_CPP_API void MW_CALL_CONV qiu_ni(int nargout, mwArray& inv_B, const mwArray& B);

#endif
#endif
