/**
  ******************************************************************************
  * @file    spi_nand_cmd.c
  * @author  MX Application Team
  * @date    10-July-2018
  * @brief   This file provides MX35XX command function.
  ******************************************************************************
  */
#include "../spi_nand/spi_nand_cmd.h"

/*
 * Function:      CheckFlashStatus
 * Arguments:	  Mxic, pointer to an mxchip structure of SPI NAND device.
 * Return Value:  MXST_SUCCESS
 *                MXST_FAILURE
 *                MXST_DEVICE_BUSY
 *                MXST_DEVICE_READY
 *                MXST_DEVICE_PROGRAM_FAILED
 *                MXST_DEVICE_ERASE_FAILED
 * Description:   This function is for checking status register bit 7 ~ bit 0
 */
int MxCheckFlashStatus(MxChip *Mxic)
{
	int Status;
	u8 Sr;
	Status = Mx_GET_FEATURE(Mxic, FEATURES_ADDR_STATUS, &Sr);
	if (Status != MXST_SUCCESS)
		return Status;
	if(Sr & SR_OIP)
	{

		return MXST_DEVICE_BUSY;
	}
	else if(Sr & SR_PROGRAM_FAIL)
	{

		return MXST_DEVICE_PROGRAM_FAILED;
	}
	else if(Sr & SR_ERASE_FAIL)
	{

		return MXST_DEVICE_ERASE_FAILED;
	}
	else
		return MXST_DEVICE_READY;
}

/*
 * Function:      MxIsFlashQIO
 * Arguments:	  Mxic, pointer to an mxchip structure of SPI NAND device.
 * Return Value:  MXST_SUCCESS
 *                MXST_FAILURE
 *                MXST_FLASH_QIO_ENABLE
 *                MXST_FLASH_QIO_NOT_ENABLE
 * Description:   This function is for checking if flash Quad-IO mode is enable or not.
 */
int MxIsFlashQIOEnable(MxChip *Mxic)
{
	int Status;
	u8 ScurOtpReg;
	Status = Mx_GET_FEATURE(Mxic, FEATURES_ADDR_SECURE_OTP, &ScurOtpReg);
	if (Status != MXST_SUCCESS)
		return Status;
	if(ScurOtpReg & SECURE_OTP_QE)
		return MXST_FLASH_QIO_ENABLE;
	else
		return MXST_FLASH_QIO_NOT_ENABLE;
}

/*
 * Function:      MxIsOnDieEccEnable
 * Arguments:	  Mxic, pointer to an mxchip structure of SPI NAND device.
 * Return Value:  MXST_SUCCESS
 *                MXST_FAILURE
 *                TRUE
 *                FALSE
 * Description:   This function is for checking if on die ECC is enable or not.
 */
int MxIsOnDieEccEnable(MxChip *Mxic)
{
	int Status;
	u8 ScurOtpReg;
	Status = Mx_GET_FEATURE(Mxic, FEATURES_ADDR_SECURE_OTP, &ScurOtpReg);
	if (Status != MXST_SUCCESS)
		return Status;
	if(ScurOtpReg & SECURE_OTP_ECC_EN)
		return TRUE;
	else
		return FALSE;
}

/*
 * Function:      MxEnableOnDieEcc
 * Arguments:	  Mxic, pointer to an mxchip structure of SPI NAND device.
 * Return Value:  MXST_SUCCESS
 *                MXST_FAILURE
 * Description:   This function is for enabling on die ECC.
 */
int MxEnableOnDieEcc(MxChip *Mxic)
{
	int Status;
	u8 ScurOtpReg, ConfigReg;
	Status = Mx_GET_FEATURE(Mxic, FEATURES_ADDR_SECURE_OTP, &ScurOtpReg);
	if (Status != MXST_SUCCESS)
		return Status;

	ScurOtpReg |= SECURE_OTP_ECC_EN;

	Status = Mx_SET_FEATURE(Mxic, FEATURES_ADDR_SECURE_OTP, &ScurOtpReg);
	if (Status != MXST_SUCCESS)
		return Status;

	Status = Mx_GET_FEATURE(Mxic, FEATURES_ADDR_CONFIG, &ConfigReg);
	if (Status != MXST_SUCCESS)
		return Status;

	ConfigReg |= 0x60; //set BFT bits

	return Mx_SET_FEATURE(Mxic, FEATURES_ADDR_CONFIG, &ConfigReg);

}

/*
 * Function:      MxDisableOnDieEcc
 * Arguments:	  Mxic, pointer to an mxchip structure of SPI NAND device.
 * Return Value:  MXST_SUCCESS
 *                MXST_FAILURE
 * Description:   This function is for disabling on die ECC.
 */
int MxDisableOnDieEcc(MxChip *Mxic)
{
	int Status;
	u8 ScurOtpReg;
	Status = Mx_GET_FEATURE(Mxic, FEATURES_ADDR_SECURE_OTP, &ScurOtpReg);
	if (Status != MXST_SUCCESS)
		return Status;

	ScurOtpReg &= ~SECURE_OTP_ECC_EN;

	return Mx_SET_FEATURE(Mxic, FEATURES_ADDR_SECURE_OTP, &ScurOtpReg);
}

/*
 * Function:      MxWaitForFlashReady
 * Arguments:	  Mxic,       pointer to an mxchip structure of SPI NAND device.
 *                ExpectTime, expected time-out value of flash operations.
 * Return Value:  MXST_SUCCESS
 *                MXST_FAILURE
 *                MXST_TIMEOUT
 * Description:   If flash is ready return MXST_SUCCESS. If flash is time-out return MXST_TIMEOUT.
 */
int MxWaitForFlashReady(MxChip *Mxic,u32 ExpectTime)
{
	int32_t time_out          = (INT32_MAX);

	while(MxCheckFlashStatus(Mxic) == MXST_DEVICE_BUSY)
	{

	    --time_out;
		if(0 >= time_out)
		{
			return MXST_TIMEOUT;
		}
	}
	return MXST_SUCCESS;
}

/*
 * Function:      Mx_ECC_STAT_READ
 * Arguments:	  Mxic,    pointer to an mxchip structure of SPI NAND device.
 *                Buf:     pointer to a data buffer where the register value will be stored
 * Return Value:  MXST_SUCCESS
 *                MXST_FAILURE
 * Description:   This function is for checking ECC status
 */
int Mx_ECC_STAT_READ(MxChip *Mxic, u8 *Buf)
{
	u8 Cmd = MX_CMD_ECC_STAT_READ;
	MxSpi *Spi = Mxic->Priv;

	Spi->LenAddr = 0;
	Spi->LenDummy = 1;
	Spi->FlashProtocol = PROT_1_1_1;

	return MxSpiFlashRead(Spi, 0, 1, Buf, Cmd);
}

/*
 * Function:      Mx_ECC_STAT_READ
 * Arguments:	  Mxic,    pointer to an mxchip structure of SPI NAND device.
 *                Buf:     pointer to a data buffer where the register value will be stored
 * Return Value:  MXST_SUCCESS
 *                MXST_FAILURE
 * Description:   This function is for checking ECC status
 */
int Mx_ECC_WARNING(MxChip *Mxic, u8 *Buf)
{
	u8 Cmd = MX_CMD_ECC_WARNING;
	MxSpi *Spi = Mxic->Priv;

	Spi->LenAddr = 0;
	Spi->LenDummy = 1;
	Spi->FlashProtocol = PROT_1_1_1;

	return MxSpiFlashRead(Spi, 0, 6, Buf, Cmd);
}

/*
 * Function:      Mx_GET_FEATURE
 * Arguments:	  Mxic:    pointer to an mxchip structure of SPI NAND device.
 *                Addr:    get Feature Address
 *                Buf:     pointer to a data buffer where the feature register value will be stored
 * Return Value:  MXST_SUCCESS
 *                MXST_FAILURE
 * Description:   This function is for checking features settings.
 */
int Mx_GET_FEATURE(MxChip *Mxic, u32 Addr, u8 *Buf)
{
	u8 Cmd = MX_CMD_GET_FEATURE;
	MxSpi *Spi = Mxic->Priv;

	Spi->LenAddr = 1;
	Spi->LenDummy = 0;
	Spi->FlashProtocol = PROT_1_1_1;

	return MxSpiFlashRead(Spi, Addr, 1, Buf, Cmd);
}

/*
 * Function:      Mx_SET_FEATURE
 * Arguments:	  Mxic:    pointer to an mxchip structure of SPI NAND device.
 *                Addr:    set Feature Address
 *                Buf:     pointer to a data buffer where the set feature register value is stored
 * Return Value:  MXST_SUCCESS
 *                MXST_FAILURE
 * Description:   This function is for writing features settings.
 */
int Mx_SET_FEATURE(MxChip *Mxic, u32 Addr, u8 *Buf)
{
	u8 Cmd = MX_CMD_SET_FEATURE;
	MxSpi *Spi = Mxic->Priv;

	Spi->LenAddr = 1;
	Spi->LenDummy = 0;
	Spi->FlashProtocol = PROT_1_1_1;

	return MxSpiFlashWrite(Spi, Addr, 1, Buf, Cmd);
}

/*
 * Function:      MxRDID
 * Arguments:	  Mxic:      pointer to an mxchip structure of SPI NAND device.
 *                Buf:       data buffer to store the ID value
 * Return Value:  MXST_SUCCESS
 *                MXST_FAILURE
 * Description:   This function is to read the manufacturer ID of 1-byte and followed by Device ID of 2-byte.
 */
int Mx_RDID(MxChip *Mxic, u8 *Buf)
{
	u8 Cmd = MX_CMD_RDID;
	MxSpi *Spi = Mxic->Priv;

	Spi->LenAddr = 0;
	Spi->LenDummy = 1;
	Spi->FlashProtocol = PROT_1_1_1;

	return MxSpiFlashRead(Spi, 0, 2, Buf, Cmd);
}

/*
 * Function:      Mx_RD_ECC_STATUS
 * Arguments:	  Mxic:      pointer to an mxchip structure of SPI NAND device.
 *                Buf:       data buffer to store the ECC status value
 * Return Value:  MXST_SUCCESS
 *                MXST_FAILURE
 * Description:   This function is to read the value of ECC status.
 */
int Mx_RD_ECC_STATUS(MxChip *Mxic, u8 *Buf)
{
	u8 Cmd = MX_CMD_RD_ECC_STATUS;
	MxSpi *Spi = Mxic->Priv;

	Spi->LenAddr = 0;
	Spi->LenDummy = 1;
	Spi->FlashProtocol = PROT_1_1_1;

	return MxSpiFlashRead(Spi, 0, 1, Buf, Cmd);
}

/*
 * Function:      MxREAD
 * Arguments:	  Mxic:      pointer to an mxchip structure of SPI NAND device.
 *                Addr:      device address to read
 * Return Value:  MXST_SUCCESS
 *                MXST_FAILURE
 *                MXST_TIMEOUT
 * Description:   This function is for reading data from array to cache.
 */
int Mx_READ(MxChip *Mxic, u32 Addr)
{
	int Status;
	u8 Cmd = MX_CMD_READ;
	MxSpi *Spi = Mxic->Priv;

	Spi->LenAddr = 3;
	Spi->LenDummy = 0;
	Spi->FlashProtocol = PROT_1_1_1;

	u32 addr = Addr>>12;

	Status = MxSpiFlashWrite(Spi, addr, 0, 0, Cmd);
	if (Status != MXST_SUCCESS)
		return Status;

	return MxWaitForFlashReady(Mxic, NAND_FLASH_TIMEOUT_VALUE);
}

/*
 * Function:      Mx_READ_CACHE
 * Arguments:	  Mxic:      pointer to an mxchip structure of SPI NAND device.
 *                Addr:      device address to read
 *                ByteCount: number of bytes to read
 *                Buf:       pointer to a data buffer where the read data will be stored
 *                WrapBit:   define wrap bit and Plane select bit (only for 2Gb and 4Gb)
 * Return Value:  MXST_SUCCESS
 *                MXST_FAILURE
 * Description:   This function is for reading data out from cache on SO.
 */
int Mx_READ_CACHE(MxChip *Mxic, u32 Addr, u32 ByteCount, u8 *Buf, u8 WrapBit)
{
	u8 Cmd = 0x03;
	MxSpi *Spi = Mxic->Priv;

	Spi->LenAddr = 2;
	Spi->LenDummy = 1;
	Spi->FlashProtocol = PROT_1_1_1;

    u32 PlaneSel = (0x01 & (Addr >> 18));
    u32 addr = (Addr&0xFFF) | (WrapBit<<14) | (PlaneSel<<12);

	return MxSpiFlashRead(Spi, addr, ByteCount, Buf, Cmd);
}

/*
 * Function:      Mx_READ_CACHE2
 * Arguments:	  Mxic:      pointer to an mxchip structure of SPI NAND device.
 *                Addr:      device address to read
 *                ByteCount: number of bytes to read
 *                Buf:       pointer to a data buffer where the read data will be stored
 *                WrapBit:   define wrap bit and Plane select bit (only for 2Gb and 4Gb)
 * Return Value:  MXST_SUCCESS
 *                MXST_FAILURE
 * Description:   This function is for reading data out from cache on SI and SO.
 */
int Mx_READ_CACHE2(MxChip *Mxic, u32 Addr, u32 ByteCount, u8 *Buf, u8 WrapBit)
{
	u8 Cmd = MX_CMD_READ_CACHE2;
	MxSpi *Spi = Mxic->Priv;

	Spi->LenAddr = 2;
	Spi->LenDummy = 1;
	Spi->FlashProtocol = PROT_1_1_2;

    u32 PlaneSel = (0x01 & (Addr >> 18));
    u32 addr = (Addr&0xFFF) | (WrapBit<<14) | (PlaneSel<<12);

	return MxSpiFlashRead(Spi, addr, ByteCount, Buf, Cmd);
}

/*
 * Function:      Mx_READ_CACHE4
 * Arguments:	  Mxic:      pointer to an mxchip structure of SPI NAND device.
 *                Addr:      device address to read
 *                ByteCount: number of bytes to read
 *                Buf:       pointer to a data buffer where the read data will be stored
 *                WrapBit:   define wrap bit and Plane select bit (only for 2Gb and 4Gb)
 * Return Value:  MXST_SUCCESS
 *                MXST_FAILURE
 * Description:   This function is for reading data out from cache on on SI, SO, WP and HOLD.
 */
int Mx_READ_CACHE4(MxChip *Mxic, u32 Addr, u32 ByteCount, u8 *Buf, u8 WrapBit)
{

	if( MxIsFlashQIOEnable(Mxic) != MXST_FLASH_QIO_ENABLE )
		return MXST_FLASH_QIO_NOT_ENABLE;

	u8 Cmd = MX_CMD_READ_CACHE4;
	MxSpi *Spi = Mxic->Priv;

	Spi->LenAddr = 2;
	Spi->LenDummy = 1;
	Spi->FlashProtocol = PROT_1_1_4;

    u32 PlaneSel = (0x01 & (Addr >> 18));
    u32 addr = (Addr&0xFFF) | (WrapBit<<14) | (PlaneSel<<12);

	return MxSpiFlashRead(Spi, addr, ByteCount, Buf, Cmd);
}

/*
 * Function:      Mx_READ_CACHE_SEQUENTIAL
 * Arguments:	  Mxic: pointer to an mxchip structure of SPI NAND device.
 * Return Value:  MXST_SUCCESS
 *                MXST_FAILURE
 *                MXST_TIMEOUT
 * Description:   This function is for throughput enhancement by using the internal cache buffer.
 */
int Mx_READ_CACHE_SEQUENTIAL(MxChip *Mxic)
{
	int Status;
	u8 Cmd = MX_CMD_READ_CACHE_SEQUENTIAL;
	MxSpi *Spi = Mxic->Priv;

	Spi->LenAddr = 0;
	Spi->LenDummy = 0;
	Spi->FlashProtocol = PROT_1_1_1;

	Status = MxSpiFlashWrite(Spi, 0, 0, 0, Cmd);
	if (Status != MXST_SUCCESS)
		return Status;

	return MxWaitForFlashReady(Mxic, NAND_FLASH_TIMEOUT_VALUE);
}

/*
 * Function:      Mx_READ_CACHE_END
 * Arguments:	  Mxic: pointer to an mxchip structure of SPI NAND device.
 * Return Value:  MXST_SUCCESS
 *                MXST_FAILURE
 *                MXST_TIMEOUT
 * Description:   This function is for ending reading data from cache buffer.
 */
int Mx_READ_CACHE_END(MxChip *Mxic)
{
	int Status;
	u8 Cmd = MX_CMD_READ_CACHE_END;
	MxSpi *Spi = Mxic->Priv;

	Spi->LenAddr = 0;
	Spi->LenDummy = 0;
	Spi->FlashProtocol = PROT_1_1_1;

	Status = MxSpiFlashWrite(Spi, 0, 0, 0, Cmd);
	if (Status != MXST_SUCCESS)
		return Status;

	return MxWaitForFlashReady(Mxic, NAND_FLASH_TIMEOUT_VALUE);
}

/*
 * Function:      MxWREN
 * Arguments:	  Mxic: pointer to an mxchip structure of SPI NAND device.
 * Return Value:  MXST_SUCCESS
 *                MXST_FAILURE
 * Description:   This function is for setting Write Enable Latch bit.
 */
int Mx_WREN(MxChip *Mxic)
{	int Status;
    u8 Sr;
	u8 Cmd = MX_CMD_WREN;
	MxSpi *Spi = Mxic->Priv;

	Spi->LenAddr = 0;
	Spi->LenDummy = 0;
	Spi->FlashProtocol = PROT_1_1_1;

	 MxSpiFlashWrite(Mxic->Priv, 0, 0, 0, Cmd);

	Status = Mx_GET_FEATURE(Mxic, FEATURES_ADDR_STATUS, &Sr);
	if (Status != MXST_SUCCESS)
		return Status;

	Status = Mx_GET_FEATURE(Mxic, 0xA0, &Sr);
	if (Status != MXST_SUCCESS)
		return Status;

	Sr = 0x00;
	Status = Mx_SET_FEATURE(Mxic, 0xA0, &Sr);
	if (Status != MXST_SUCCESS)
		return Status;

	return 0;
}

/*
 * Function:      MxWRDI
 * Arguments:	  Mxic: pointer to an mxchip structure of SPI NAND device.
 * Return Value:  MXST_SUCCESS
 *                MXST_FAILURE
 * Description:   This function is to reset Write Enable Latch (WEL) bit.
 */
int Mx_WRDI(MxChip *Mxic)
{
	u8 Cmd = MX_CMD_WRDI;
	MxSpi *Spi = Mxic->Priv;

	Spi->LenAddr = 0;
	Spi->LenDummy = 0;
	Spi->FlashProtocol = PROT_1_1_1;

	return MxSpiFlashWrite(Mxic->Priv, 0, 0, 0, Cmd);
}

/*
 * Function:      Mx_PP_LOAD
 * Arguments:	  Mxic:      pointer to an mxchip structure of SPI NAND device.
 *                Addr:      device address to write
 *                ByteCount: number of bytes to write
 *                Buf:       pointer to a data buffer where the write data will be stored
 *                WrapBit:   define wrap bit and Plane select bit (only for 2Gb and 4Gb)
 * Return Value:  MXST_SUCCESS
 *                MXST_FAILURE
 * Description:   This function is for loading program data with cache reset first.
 */
int Mx_PP_LOAD(MxChip *Mxic, u32 Addr, u32 ByteCount, u8 *Buf, u8 WrapBit)
{
	int Status;
	u8 Cmd = MX_CMD_PP_LOAD;
	MxSpi *Spi = Mxic->Priv;

	Status = Mx_WREN(Mxic);
	if (Status != MXST_SUCCESS)
		return Status;

	Spi->LenAddr = 2;
	Spi->LenDummy = 0;
	Spi->FlashProtocol = PROT_1_1_1;

	u32 PlaneSel = (0x01 & (Addr >> 18));
	u32 addr = (Addr&0xFFF) | (WrapBit<<14) | (PlaneSel<<12);

	return MxSpiFlashWrite(Spi, addr, ByteCount, Buf, Cmd);
}

/*
 * Function:      Mx_PP_RAND_LOAD
 * Arguments:	  Mxic:      pointer to an mxchip structure of SPI NAND device.
 *                Addr:      device address to write
 *                ByteCount: number of bytes to write
 *                Buf:       pointer to a data buffer where the write data will be stored
 *                WrapBit:   define wrap bit and Plane select bit (only for 2Gb and 4Gb)
 * Return Value:  MXST_SUCCESS
 *                MXST_FAILURE
 * Description:   This function is for loading program data without cache reset first.
 */
int Mx_PP_RAND_LOAD(MxChip *Mxic, u32 Addr, u32 ByteCount, u8 *Buf, u8 WrapBit)
{
	int Status;
	u8 Cmd = MX_CMD_PP_RAND_LOAD;
	MxSpi *Spi = Mxic->Priv;

	Status = Mx_WREN(Mxic);
	if (Status != MXST_SUCCESS)
		return Status;

	Spi->LenAddr = 2;
	Spi->LenDummy = 0;
	Spi->FlashProtocol = PROT_1_1_1;

    u32 PlaneSel = (0x01 & (Addr >> 18));
    u32 addr = (Addr&0xFFF) | (WrapBit<<14) | (PlaneSel<<12);

	return MxSpiFlashWrite(Spi, addr, ByteCount, Buf, Cmd);
}

/*
 * Function:      Mx_4PP_LOAD
 * Arguments:	  Mxic:      pointer to an mxchip structure of SPI NAND device.
 *                Addr:      device address to write
 *                ByteCount: number of bytes to write
 *                Buf:       pointer to a data buffer where the write data will be stored
 *                WrapBit:   define wrap bit and Plane select bit (only for 2Gb and 4Gb)
 * Return Value:  MXST_SUCCESS
 *                MXST_FAILURE
 * Description:   This function is for loading program data with 4 data input.
 */
int Mx_4PP_LOAD(MxChip *Mxic, u32 Addr, u32 ByteCount, u8 *Buf, u8 WrapBit)
{
	int Status;
	u8 Cmd = MX_CMD_4PP_LOAD;
	MxSpi *Spi = Mxic->Priv;

	Status = Mx_WREN(Mxic);
	if (Status != MXST_SUCCESS)
		return Status;

	Spi->LenAddr = 2;
	Spi->LenDummy = 0;
	Spi->FlashProtocol = PROT_1_1_4;

    u32 PlaneSel = (0x01 & (Addr >> 18));
    u32 addr = (Addr&0xFFF) | (WrapBit<<14) | (PlaneSel<<12);

	return MxSpiFlashWrite(Spi, addr, ByteCount, Buf, Cmd);
}

/*
 * Function:      Mx_4PP_RAND_LOAD
 * Arguments:	  Mxic:      pointer to an mxchip structure of SPI NAND device.
 *                Addr:      device address to write
 *                ByteCount: number of bytes to write
 *                Buf:       pointer to a data buffer where the write data will be stored
 *                WrapBit:   define wrap bit and Plane select bit (only for 2Gb and 4Gb)
 * Return Value:  MXST_SUCCESS
 *                MXST_FAILURE
 * Description:   This function is for loading program random data with 4 data input.
 */
int Mx_4PP_RAND_LOAD(MxChip *Mxic, u32 Addr, u32 ByteCount, u8 *Buf, u8 WrapBit)
{
	int Status;
	u8 Cmd = MX_CMD_4PP_RAND_LOAD;
	MxSpi *Spi = Mxic->Priv;

	Status = Mx_WREN(Mxic);
	if (Status != MXST_SUCCESS)
		return Status;

	Spi->LenAddr = 2;
	Spi->LenDummy = 0;
	Spi->FlashProtocol = PROT_1_1_4;

    u32 PlaneSel = (0x01 & (Addr >> 18));
    u32 addr = (Addr&0xFFF) | (WrapBit<<14) | (PlaneSel<<12);

	return MxSpiFlashWrite(Spi, addr, ByteCount, Buf, Cmd);
}

/*
 * Function:      Mx_PROGRAM_EXEC
 * Arguments:	  Mxic: pointer to an mxchip structure of SPI NAND device.
 *                Addr: device address to write
 * Return Value:  MXST_SUCCESS
 *                MXST_FAILURE
 *                MXST_TIMEOUT
 * Description:   This function is for executing programming data to array.
 */
int Mx_PROGRAM_EXEC(MxChip *Mxic, u32 Addr)
{
	int Status;
	u8 Cmd = MX_CMD_PROGRAM_EXEC;
	MxSpi *Spi = Mxic->Priv;

	Spi->LenAddr = 3;
	Spi->LenDummy = 0;
	Spi->FlashProtocol = PROT_1_1_1;

	u32 addr = Addr>>12;

	Status = MxSpiFlashWrite(Spi, addr, 0, 0, Cmd);
	if (Status != MXST_SUCCESS)
		return Status;

	return MxWaitForFlashReady(Mxic, NAND_FLASH_TIMEOUT_VALUE);
}

/*
 * Function:      Mx_BE
 * Arguments:	  Mxic: pointer to an mxchip structure of SPI NAND device.
 *                Addr: device address to erase
 * Return Value:  MXST_SUCCESS
 *                MXST_FAILURE
 *                MXST_TIMEOUT
 * Description:   This function is for erasing the data.
 */
int Mx_BE(MxChip *Mxic, u32 Addr)
{
	int Status;
	u8 Cmd = MX_CMD_BE;
	MxSpi *Spi = Mxic->Priv;

	Status = Mx_WREN(Mxic);
	if (Status != MXST_SUCCESS)
		return Status;

	Spi->LenAddr = 3;
	Spi->LenDummy = 0;
	Spi->FlashProtocol = PROT_1_1_1;

	Status = MxSpiFlashWrite(Spi, Addr>>12, 0, 0, Cmd);
	if (Status != MXST_SUCCESS)
		return Status;

	return MxWaitForFlashReady(Mxic, NAND_FLASH_TIMEOUT_VALUE);
}

/*
 * Function:      Mx_RESET
 * Arguments:	  Mxic: pointer to an mxchip structure of SPI NAND device.
 * Return Value:  MXST_SUCCESS
 *                MXST_FAILURE
 * Description:   This function is for resetting the read/program/erase operation.
 */
int Mx_RESET(MxChip *Mxic)
{
	u8 Cmd = MX_CMD_RESET;
	MxSpi *Spi = Mxic->Priv;

	Spi->LenAddr = 0;
	Spi->LenDummy = 0;
	Spi->FlashProtocol = PROT_1_1_1;

	return MxSpiFlashWrite(Spi, 0, 0, 0, Cmd);
}
