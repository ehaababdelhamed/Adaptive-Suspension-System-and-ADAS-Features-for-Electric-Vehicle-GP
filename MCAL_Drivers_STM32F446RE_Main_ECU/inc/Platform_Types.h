 /******************************************************************************
 * Module: Common - Platform Abstraction
 *
 * File Name: Platform_Types.h
 *
 * Description: Platform Types - Specification of AUTOSAR CP R20-11
 *
 * Author: Ehab Mohamed Abdelhamed
 *******************************************************************************/

#ifndef PLATFORM_TYPES_H_
#define PLATFORM_TYPES_H_


/*************** CPU Type definition *******************/

#define CPU_TYPE_8     (8u)
#define CPU_TYPE_16    (16u)
#define CPU_TYPE_32    (32u)

/*************** Bit Ordering definition **************/
#define LSB_FIRST (0u)
#define MSB_FIRST (1u)

/*************** Byte Ordering definition **************/
#define LOW_BYTE_ORDER  (0u)
#define HIGH_BYTE_ORDER (1u)

/*************** Platform specification **************/
#define CPU_TYPE            CPU_TYPE_32
#define CPU_BIT_ORDER       LSB_FIRST
#define CPU_BYTE_ORDER      LOW_BYTE_FIRST

/*************** boolean values *********************/
#ifndef TRUE
#define TRUE  (1u)
#endif

#ifndef FALSE
#define FALSE  (0u)
#endif

/*************** NULL *********************/


#ifndef NULL
#define NULL  (void*)(0u)
#endif

/*************** Main Types  ************************/
typedef unsigned char boolean;

typedef signed char sint8;
typedef unsigned char uint8;
typedef signed short sint16;
typedef unsigned short uint16;
typedef signed long sint32;
typedef unsigned long uint32;
typedef signed long long sint64;
typedef unsigned long long uint64;
typedef float float32;
typedef double float64;

typedef void* VoidPtr;
typedef const void* ConstVoidPtr;

#endif /* PLATFORM_TYPES_H_ */
