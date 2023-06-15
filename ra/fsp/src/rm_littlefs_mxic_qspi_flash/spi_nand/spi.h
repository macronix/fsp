/**
  ******************************************************************************
  * @file    spi.h
  * @author  MX Application Team
  * @date    10-July-2018
  * @brief   Header for spi.c module.
  ******************************************************************************
  */


#ifndef SPI_H_
#define SPI_H_

#include "../spi_nand/mxic_hc.h"

int MxSpiFlashWrite(MxSpi *Spi, u32 Addr, u32 ByteCount, u8 *WrBuf, u8 WrCmd);
int MxSpiFlashRead(MxSpi *Spi, u32 Addr, u32 ByteCount, u8 *RdBuf, u8 RdCmd);

#endif /* SPI_H_ */
