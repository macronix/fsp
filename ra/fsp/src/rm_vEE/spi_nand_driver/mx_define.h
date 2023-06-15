/**
  ******************************************************************************
  * @file    mx_define.h
  * @author  MX Application Team
  * @date    10-July-2018
  * @brief   This file contains definitions for device operation status and data type.
  ******************************************************************************
  */

#ifndef MX_DEFINE_H
#define MX_DEFINE_H

#include "string.h"
#include "stdio.h"
#include "rm_vee_cfg.h"

enum DeviceStatus{
	 MXST_SUCCESS                 = 0L,
	 MXST_FAILURE                 = 1L,
	 MXST_TIMEOUT                 = 2L,
	 MXST_DEVICE_IS_STARTED       = 3L,
	 MXST_DEVICE_IS_STOPPED       = 4L,
	 MXST_ID_NOT_MATCH            = 5L,
	 MXST_DEVICE_BUSY             = 6L,	/* device is busy */
	 MXST_DEVICE_READY            = 7L,	/* device is ready */
	 MXST_DEVICE_PROGRAM_FAILED   = 10L,
	 MXST_DEVICE_ERASE_FAILED	  = 11L,
	 MXST_FLASH_QIO_NOT_ENABLE    = 12L,
	 MXST_FLASH_QIO_ENABLE        = 13L,
	 EBADMSG					  = 14L,  /* status about ECC  */
	 EINVAL						  = 15L,  /* status about ECC  */
	 MXST_BLOCK_IS_BAD			  = 16L,
	 MXST_BLOCK_ISNOT_BAD		  = 17L,
};

typedef unsigned char		u8;				/**< unsigned 8-bit */
typedef char				int8;			/**< signed 8-bit */
typedef unsigned short		u16;			/**< unsigned 16-bit */
typedef short				int16;			/**< signed 16-bit */
typedef unsigned long		u32;			/**< unsigned 32-bit */
typedef unsigned long long	u64;			/**< unsigned 64-bit */
typedef long				int32;			/**< signed 32-bit */
typedef float				Xfloat32;		/**< 32-bit floating point */
typedef double				Xfloat64;		/**< 64-bit double precision FP */
typedef unsigned long		Xboolean;		/**< boolean (XTRUE or XFALSE) */

#define TRUE		1U
#define FALSE		0U
//#define NULL		0

#endif
