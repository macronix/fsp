/**
  ******************************************************************************
  * @file    spi.c
  * @author  MX Application Team
  * @date    10-July-2018
  * @brief   This file provides MxSpiFlashWrite and MxSpiFlashRead function.
  ******************************************************************************
  */


#include "../spi_nand_driver/spi.h"

#ifdef SPINAND
#ifndef SPI_XFER_PERF
#define EXTRA_SZ	30
#define RDWR_BUF_SZ 2048
 u8 ReadBuffer[EXTRA_SZ + RDWR_BUF_SZ], WriteBuffer[EXTRA_SZ + RDWR_BUF_SZ];
#endif

/*
 * Function:      MxAddr2Cmd
 * Arguments:	  Spi,     pointer to an MxSpi structure of transfer.
 *                Addr:    the address to put into the send data buffer.
 *                CmdBuf:  the data will be send to controller.
 * Return Value:  None.
 * Description:   This function put the value of address into the send data buffer. This address is stored after the command code.
 */
static void MxAddr2Cmd(MxSpi *Spi, u32 Addr, u8 *CmdBuf)
{
	int n;

	for (n = 1; n <= Spi->LenAddr; n++)
		CmdBuf[n] = Addr >> (Spi->LenAddr - n) * 8;
}

/*
 * Function:      SpiFlashWrite
 * Arguments:	  Spi,       pointer to an MxSpi structure of transfer
 * 				  Addr:      address to be written to
 * 				  ByteCount, number of byte to write
 * 			      WrBuf,     Pointer to a data buffer where the write data will be stored
 * 			      WrCmd:     write command code to be written to the flash
 * Return Value:  MXST_SUCCESS
 *                MXST_FAILURE
 * Description:   This function prepares the data to be written and put them into data buffer,
 *                then call MxPolledTransfer function to start a write data transfer.
 */
int MxSpiFlashWrite(MxSpi *Spi, u32 Addr, u32 ByteCount, u8 *WrBuf, u8 WrCmd)
{
	int Status;
	u32 LenInst;
	u32 WrSz;
	/*
	 * Setup the write command with the specified address and data for the flash
	 */
	Spi->IsRd = 0;
	Spi->LenCmd = 1;
	LenInst = Spi->LenCmd + Spi->LenAddr;

	WriteBuffer[0] = WrCmd ;

	do
	{
		WrSz = ByteCount > RDWR_BUF_SZ ? RDWR_BUF_SZ : ByteCount;
		MxAddr2Cmd(Spi, Addr, WriteBuffer);
		memcpy(WriteBuffer + LenInst, WrBuf, WrSz);

		R_QSPI_DirectWrite (Spi->Controller->p_ctrl, WriteBuffer, WrSz + LenInst, false);

		WrBuf += WrSz;
		Addr += WrSz;
		ByteCount -= WrSz;
	}
	while(ByteCount);

	return MXST_SUCCESS;
}

/*
 * Function:      SpiFlashRead
 * Arguments:	  Spi,       pointer to an MxSpi structure of transfer.
 * 				  Addr:      address to be read.
 * 				  ByteCount, number of byte to read.
 * 			      RdBuf:     pointer to a data buffer where the read data will be stored.
 * 			      RdCmd:     read command code to be written to the flash.
 * Return Value:  MXST_SUCCESS.
 *                MXST_FAILURE.
 * Description:   This function calls MxPolledTransfer function to start a read data transfer,
 *  			  then put the read data into data buffer.
 */
int MxSpiFlashRead(MxSpi *Spi, u32 Addr, u32 ByteCount, u8 *RdBuf, u8 RdCmd)
{
	u32 LenInst;
	/*
	 * Setup the read command with the specified address, data and dummy for the flash
	 */

	Spi->IsRd = 1;
	Spi->LenCmd = 1;
	LenInst = Spi->LenCmd + Spi->LenAddr + Spi->LenDummy;

	WriteBuffer[0] = RdCmd ;

	MxAddr2Cmd(Spi, Addr, WriteBuffer);
	if (Spi->LenDummy)
		WriteBuffer[1 + Spi->LenAddr] = 0x0;

	R_QSPI_DirectWrite (Spi->Controller->p_ctrl, WriteBuffer, LenInst, true);

	R_QSPI_DirectRead (Spi->Controller->p_ctrl, ReadBuffer, ByteCount);

	memcpy(RdBuf, ReadBuffer, ByteCount);

	return MXST_SUCCESS;
}
#else
int MxSpiFlashWrite(MxSpi *Spi, u32 Addr, u32 ByteCount, u8 *WrBuf, u8 WrCmd)
{
    return MXST_FAILURE;
}
int MxSpiFlashRead(MxSpi *Spi, u32 Addr, u32 ByteCount, u8 *RdBuf, u8 RdCmd)
{
    return MXST_FAILURE;
}
#endif

