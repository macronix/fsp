/**
  ******************************************************************************
  * @file    mxic_hc.h
  * @author  MX Application Team
  * @date    10-July-2018
  * @brief   Header for mxic_hc.c module.
  ******************************************************************************
  */


#ifndef MXIC_HC_H_
#define MXIC_HC_H_

#include "../spi_nand/mx_define.h"


typedef struct {
	u8 *InstrBufPtr;
	u8 LenCmd;
	u8 LenDummy;
	u8 LenAddr;
	u8 IsRd;
	u32 CurrFreq;
	u8 WrCmd;
	u8 RdCmd;
	u8 *SendBufferPtr;	 /**< Buffer to send (state) */
	u8 *RecvBufferPtr;	 /**< Buffer to receive (state) */
	int RequestedBytes;	 /**< Number of bytes to transfer (state) */
	int RemainingBytes;	 /**< Number of bytes left to transfer(state) */
	u32 IsBusy;		     /**< A transfer is in progress (state) */
	u8 TransFlag;
	u8 FlashProtocol;
	u8 PreambleEn;
	u8 DataPass;
	u8 HardwareMode;
	u8 PlaneSelectBit;

	spi_flash_instance_t * Controller;

} MxSpi;

enum FlashProtocol{
	PROT_1_1_1,
	PROT_1_1D_1D,
	PROT_1_1_2,
	PROT_1_1D_2D,
	PROT_1_2_2,
	PROT_1_2D_2D,
	PROT_1_1_4,
	PROT_1_1D_4D,
	PROT_1_4_4,
	PROT_1_4D_4D,
	PROT_4_4_4,
	PROT_4_4D_4D,
	PROT_8_8_8,
	PROT_8D_8D_8D,
};

enum CurrentMode{
	 MODE_SPI             =	0x00000001,
 	 MODE_SOPI 			  =	0x00000002,
 	 MODE_DOPI			  =	0x00000004,
	 MODE_OPI			  =	(MODE_SOPI | MODE_DOPI),
	 MODE_QPI			  =	0x00000008,
	 MODE_FS_READ		  =	0x00000010,
	 MODE_DUAL_READ		  =	0x00000020, //PROT_1_1_2
	 MODE_DUAL_IO_READ	  =	0x00000040, //PROT_1_2_2
	 MODE_QUAD_READ		  =	0x00000080, //PROT_1_1_4
	 MODE_QUAD_IO_READ	  =	0x00000100, //PROT_1_4_4
	 MODE_DUAL_WRITE	  =	0x00000200,
	 MODE_DUAL_IO_WRITE	  =	0x00000400,
	 MODE_QUAD_WRITE	  =	0x00000800,
	 MODE_QUAD_IO_WRITE	  =	0x00001000,
	 MODE_FSDT_READ		  =	0x00002000,
	 MODE_DUAL_IO_DT_READ =	0x00004000,
	 MODE_QUAD_IO_DT_READ =	0x00008000,
};

/********************************************************************************
 *                       Function Declaration                                   *
 ********************************************************************************/

#endif /* MXIC_HC_H_ */
