/**
  ******************************************************************************
  * @file    spi_nand_Ops.c
  * @author  MX Application Team
  * @version V1.1.0
  * @date    10-July-2018
  * @brief   This file provides different device operation by calling MX35XX command function.
  ******************************************************************************
  */

#include "../spi_nand/spi_nand_ops.h"

#include "malloc.h"

#include "../spi_nand/bitops.h"
#include "../spi_nand/nand_bch.h"

/*
 * The instances to support the device drivers are global such that they
 * are initialized to zero each time the program runs. They could be local
 * but should at least be static so they are zeroed.
 */

#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))
#define INFO(_JedecId,_PageSz,_OobSz, _BlockSz, _ChipSz,  _CmdList, _SpclFlag)	\
	.Id = {																\
			(_JedecId >> 8) & 0xff,										\
			 _JedecId & 0xff											\
		  },															\
	.PageSize = (_PageSz),								   			    \
	.OobSize = (_OobSz),								   			    \
	.BlockSz = (_BlockSz),								   			    \
	.ChipSz = (_ChipSz), 												\
	.SpclFlag = (_SpclFlag),											\
    .CmdList = (_CmdList),


typedef struct {
	char *Name;
	u8 Id[SPI_NAND_FLASH_MAX_ID_LEN];
	u32 PageSize;
	u32 OobSize;
	u32 BlockSz;
	u32 ChipSz;
	u32 SpclFlag;
	u32 CmdList;
}MxFlashInfo;

static MxFlashInfo SpiFlashParamsTable[] = {
		{ "mx35lf2ge4ab", INFO(0xC222, 2048, 64, 2112 * 64, 2112 * 64 * 2048, 0x0001FE7F,0)},
		{ "mx35lf2g14ac", INFO(0xC220, 2048, 64, 2112 * 64, 2112 * 64 * 2048, 0x0001FFFB,0)},
		{ "mx31lf1ge4bc", INFO(0xC21E, 2048, 64, 2112 * 64, 2112 * 64 * 1024, 0x0001FFFB,0)},
};

/* Define default oob placement schemes for large and small page devices */
static struct NandEccLayout NandOob_8 = {
	.EccBytes = 3,
	.EccPos = {0, 1, 2},
	.OobFree = {
		{.Offset = 3,
		 .Length = 2},
		{.Offset = 6,
		 .Length = 2} }
};

static struct NandEccLayout NandOob_16 = {
	.EccBytes = 6,
	.EccPos = {0, 1, 2, 3, 6, 7},
	.OobFree = {
		{.Offset = 8,
		 . Length = 8} }
};

static struct NandEccLayout NandOob_64 = {
	.EccBytes = 24,
	.EccPos = {
		   40, 41, 42, 43, 44, 45, 46, 47,
		   48, 49, 50, 51, 52, 53, 54, 55,
		   56, 57, 58, 59, 60, 61, 62, 63},
	.OobFree = {
		{.Offset = 2,
		 .Length = 38} }
};

static struct NandEccLayout NandOob_128 = {
	.EccBytes = 48,
	.EccPos = {
		   80, 81, 82, 83, 84, 85, 86, 87,
		   88, 89, 90, 91, 92, 93, 94, 95,
		   96, 97, 98, 99, 100, 101, 102, 103,
		   104, 105, 106, 107, 108, 109, 110, 111,
		   112, 113, 114, 115, 116, 117, 118, 119,
		   120, 121, 122, 123, 124, 125, 126, 127},
	.OobFree = {
		{.Offset = 2,
		 .Length = 78} }
};


/*
 * Function:      MxIdMatch
 * Arguments:	  Mxic:      pointer to an mxchip structure of SPI NAND device.
 * Return Value:  MXST_SUCCESS
 *                MXST_ID_NOT_MATCH
 * Description:   This function is to check if the ID value is matched to the ID in flash list.
 *                If they are matched, flash information will be assigned to Mxic structure.
 */
int MxIdMatch(MxChip *Mxic)
{
	MxFlashInfo *FlashInfo;
	int n;
	int Status;
	u8 Id[SPI_NAND_FLASH_MAX_ID_LEN];

	Status = Mx_RDID(Mxic, Id);
	if (Status == MXST_FAILURE)
		return Status;

	for (n = 0; n < ARRAY_SIZE(SpiFlashParamsTable) ; n++) {
		FlashInfo = &SpiFlashParamsTable[n];
		if (!memcmp(Id, FlashInfo->Id, SPI_NAND_FLASH_MAX_ID_LEN)) {
			Mxic->Id = FlashInfo->Id;
			Mxic->ChipSpclFlag = FlashInfo->SpclFlag;
			Mxic->ChipSz = FlashInfo->ChipSz;
			Mxic->BlockSz = FlashInfo->BlockSz;
			Mxic->CmdList = FlashInfo->CmdList;
			Mxic->PageSize = NAND_PAGE_SIZE_2048;
			Mxic->OobSize = NAND_OOB_SIZE_64;
			Mxic->page_shift = ffs(Mxic->PageSize) - 1;
			Mxic->Buffers = malloc(sizeof(*Mxic->Buffers));

			return MXST_SUCCESS;
		}
	}

	return MXST_ID_NOT_MATCH;
}

/*
 * Function:      MxSoftwareInit
 * Arguments:	  Mxic,          pointer to an mxchip structure of nor flash device.
 *                EffectiveAddr, base address.
 * Return Value:  MXST_SUCCESS.
 *                MXST_DEVICE_IS_STARTED.
 * Description:   This function is for initializing the variables.
 */
int MxSoftwareInit(MxChip *Mxic, spi_flash_instance_t * controller)
{
 	static	MxSpi Spi  ;
 	Mxic->Priv = &Spi;

 	/*
 	 * If the device is busy, disallow the initialize and return a status
 	 * indicating it is already started. This allows the user to stop the
 	 * device and re-initialize, but prevents a user from inadvertently
 	 * initializing. This assumes the busy flag is cleared at startup.
 	 */
 	if (Spi.IsBusy == TRUE) {
 		return MXST_DEVICE_IS_STARTED;
 	}

 	/*
 	 * Set some default values.
 	 */
 	Spi.IsBusy = FALSE;
 	Spi.SendBufferPtr = NULL;
 	Spi.RecvBufferPtr = NULL;
 	Spi.RequestedBytes = 0;
 	Spi.RemainingBytes = 0;
 	Spi.FlashProtocol = PROT_1_1_1;
 	Spi.PreambleEn = 0;
 	Spi.DataPass = 0;
 	Spi.Controller = controller;

 	return MXST_SUCCESS;
}

/*
 * Function:      Mx_Nand_EccInit
 * Arguments:	  Mxic,    pointer to an mxchip structure of nor flash device.
 *                EccMode, ECC mode.
 * Return Value:  MXST_SUCCESS.
 *                MXST_FAILURE.
 * Description:   This function initializes the software variables related
 * to ECC generation, ECC checking and writing ECC Bytes in spare Bytes.
 */
int Mx_Nand_EccInit(MxChip *Mxic, u8 EccMode)
{
	u8 i;
	int Status;

	/* Set the internal oob buffer location,just after the page data */
	Mxic->OobBufPtr = Mxic->Buffers->DataBuf + Mxic->PageSize;

	/*
	 * If no default placement scheme is given, select an appropriate one.
	 */
	if (!Mxic->EccCtrl.Layout && (EccMode != NAND_ECC_SOFT_BCH)) {
		switch (Mxic->OobSize) {
		case 8:
			Mxic->EccCtrl.Layout = &NandOob_8;
			break;
		case 16:
			Mxic->EccCtrl.Layout = &NandOob_16;
			break;
		case 64:
			Mxic->EccCtrl.Layout = &NandOob_64;
			break;
		case 128:
			Mxic->EccCtrl.Layout = &NandOob_128;
			break;
		default:
			;
			break;
		}
	}

	if (EccMode != NAND_ECC_ONDIE)
	{
		Status = MxDisableOnDieEcc(Mxic);
		if (Status == MXST_FAILURE) {
			return MXST_FAILURE;
		}
	}
	else
	{
		Status = MxEnableOnDieEcc(Mxic);
		if (Status == MXST_FAILURE) {
			return MXST_FAILURE;
		}
	}

	switch (EccMode) {
		case NAND_ECC_HW_OOB_FIRST:
			break;
		case NAND_ECC_HW:
		    ;
			break;
		case NAND_ECC_HW_SYNDROME:
		case NAND_ECC_SOFT:
			break;
		case NAND_ECC_SOFT_BCH:
				Mxic->EccCtrl.read_page = Mx_ReadPage_SwEcc;
				Mxic->EccCtrl.write_page = Mx_WritePage_SwEcc;
				Mxic->EccCtrl.EccBytes = NAND_ECC_BYTES;
			 	Mxic->EccCtrl.EccSize  = NAND_ECC_SIZE;
			 	Mxic->EccCtrl.EccTotalBytes = (Mxic->PageSize / Mxic->EccCtrl.EccSize) * Mxic->EccCtrl.EccBytes;
			 	Mxic->EccCtrl.EccSteps = Mxic->PageSize / Mxic->EccCtrl.EccSize;

				Mxic->EccCtrl.Strength =
						Mxic->EccCtrl.EccBytes * 8 / fls(8 * Mxic->EccCtrl.EccSize);

			break;

		case NAND_ECC_NONE:
			Mxic->EccCtrl.read_page = Mx_Nand_Read_Page_Raw;
			Mxic->EccCtrl.write_page = Mx_Nand_Write_Page_Raw;
			Mxic->EccCtrl.read_page_raw = Mx_Nand_Read_Page_Raw;
			Mxic->EccCtrl.write_page_raw = Mx_Nand_Write_Page_Raw;
			Mxic->EccCtrl.read_oob = Mx_Nand_Read_Oob_Std;
			Mxic->EccCtrl.write_oob = Mx_Nand_Write_Oob_Std;
			Mxic->EccCtrl.EccBytes = 0;
			Mxic->EccCtrl.EccSize  = Mxic->PageSize;
			Mxic->EccCtrl.EccTotalBytes = 0;
			Mxic->EccCtrl.Strength = 0;
			break;
		case NAND_ECC_ONDIE:
			Mxic->EccCtrl.read_page = Mx_ReadPage_OndieEcc;
			Mxic->EccCtrl.write_page = Mx_WritePage_OndieEcc;
			Mxic->EccCtrl.read_page_raw = Mx_Nand_Read_Page_Raw;
			Mxic->EccCtrl.write_page_raw = Mx_Nand_Write_Page_Raw;
			Mxic->EccCtrl.read_oob = Mx_Nand_Read_Oob_Std;
			Mxic->EccCtrl.write_oob = Mx_Nand_Write_Oob_Std;
			Mxic->EccCtrl.EccBytes = 0;
			Mxic->EccCtrl.EccSize  = Mxic->PageSize;
			Mxic->EccCtrl.EccTotalBytes = 0;
			Mxic->EccCtrl.Strength = 0;
			break;

		default:
			break;
	}

	/*
	 * The number of Bytes available for a client to place data into
	 * the out of band area.
	 */
	Mxic->EccCtrl.Layout->OobAvail = 0;
	for (i = 0; Mxic->EccCtrl.Layout->OobFree[i].Length
			&& i < ARRAY_SIZE(Mxic->EccCtrl.Layout->OobFree); i++)
	{
		Mxic->EccCtrl.Layout->OobAvail += Mxic->EccCtrl.Layout->OobFree[i].Length;
	}
	Mxic->OobAvail = Mxic->EccCtrl.Layout->OobAvail;

 	return MXST_SUCCESS;
}

/*
 * Function:      Mx_ReadPage_SwEcc
 * Arguments:	  Mxic,        pointer to an mxchip structure of nor flash device.
 *                Addr,        device address to read
 *                Buf,         pointer to a data buffer where the read data will be stored
 *                OobRequired, if read OOB is required
 * Return Value:  MXST_SUCCESS.
 *                MXST_FAILURE.
 * Description:   This function reads a specific page from NAND device using SW ECC block.
 * It checks for the ECC errors and corrects single bit errors. The multiple bit
 * error are reported as failure.
 */
int Mx_ReadPage_SwEcc(MxChip *Mxic, u32 Addr, u8 *Buf, int OobRequired)
{
 	int Status;
 	int i, EccSize = Mxic->EccCtrl.EccSize;
 	int EccBytes = Mxic->EccCtrl.EccBytes;
 	int EccSteps = Mxic->EccCtrl.EccSteps;
 	u8 *P = Buf;
 	u8 *EccCalc = Mxic->Buffers->EccCalc;
 	u8 *EccCode = Mxic->Buffers->EccCode;
 	u32 *EccPos = Mxic->EccCtrl.Layout->EccPos;
 	u8 MaxBitflips = 0;

 	Status = Mx_Nand_Read_Page_Raw(Mxic, Addr, Buf, OOB_REQUIRED);
 	if (Status == MXST_FAILURE) {
 		return MXST_FAILURE;
 	}

  	/*
  	 * Calculate the software ECC
  	 */
 	for (i = 0; EccSteps; EccSteps--, i += EccBytes, P += EccSize)
 		nand_bch_calculate_ecc(Mxic, P, &EccCalc[i]);

  	/*
  	 * Read the stored ECC code
  	 */
 	for (i = 0; i < Mxic->EccCtrl.EccTotalBytes; i++)
 	{
 		EccCode[i] = Mxic->OobBufPtr[EccPos[i]];
 	}

 	EccSteps = Mxic->EccCtrl.EccSteps;
 	P = Buf;

  	/*
  	 * Check for ECC errors
  	 */
 	for (i = 0 ; EccSteps; EccSteps--, i += EccBytes, P += EccSize) {
 		int stat;

 		stat = nand_bch_correct_data(Mxic, P, &EccCode[i], &EccCalc[i]);
 		if (stat < 0)
 			Mxic->EccStatus.Failed++;
 		else
 		{
 			Mxic->EccStatus.Corrected += stat;
 			MaxBitflips = (MaxBitflips > stat) ? MaxBitflips : stat;
 		}
 	}

 	if(MaxBitflips)
 		;
  	return MXST_SUCCESS;
}

/*
 * Function:      Mx_ReadPage_SwEcc
 * Arguments:	  Mxic,        pointer to an mxchip structure of nor flash device.
 *                Addr,        device address to write
 *                Buf,         pointer to a data buffer where the write data is stored
 *                OobRequired, if write OOB is required
 * Return Value:  MXST_SUCCESS.
 *                MXST_FAILURE.
 *                MXST_TIMEOUT.
 * Description:    This function writes a specific page in the NAND device using HW ECC block.
 * The ECC code is written into the spare Bytes of the page.
 */
int Mx_WritePage_SwEcc(MxChip *Mxic, u32 Addr, u8 *Buf, int OobRequired)
{
  	int i, EccSize = Mxic->EccCtrl.EccSize;
  	int EccBytes = Mxic->EccCtrl.EccBytes;
  	int EccSteps = Mxic->EccCtrl.EccSteps;
  	u8 *P = Buf;
  	u8 *EccCalc = Mxic->Buffers->EccCalc;
  	u32 *EccPos = Mxic->EccCtrl.Layout->EccPos;

  	/*
  	 * Calculate the software ECC
  	 */
  	for (i = 0; EccSteps; EccSteps--, i += EccBytes, P += EccSize)
  		nand_bch_calculate_ecc(Mxic, P, &EccCalc[i]);

  	for (i = 0; i < Mxic->EccCtrl.EccTotalBytes; i++)
  	{
  		Mxic->OobBufPtr[EccPos[i]] = EccCalc[i];
  	}
  	return Mx_Nand_Write_Page_Raw(Mxic, Addr, Buf, OOB_REQUIRED);
}

/*
 * Function:      Mx_ReadPage_OndieEcc
 * Arguments:	  Mxic,        pointer to an mxchip structure of nor flash device.
 *                Addr,        device address to read
 *                Buf,         pointer to a data buffer where the read data will be stored
 *                OobRequired, if read OOB is required
 * Return Value:  MXST_SUCCESS.
 *                MXST_FAILURE.
 * Description:   This function reads a specific page from NAND device using on-die ECC block.
 * It checks for the ECC errors and corrects single bit errors. The multiple bit
 * error are reported as failure.
 */
int Mx_ReadPage_OndieEcc(MxChip *Mxic, u32 Addr, u8 *Buf, int OobRequired)
{
	int Status;
	u8 EccStatus;
	u8 MaxBitflips;

	Status = Mx_Nand_Read_Page_Raw(Mxic, Addr, Buf, OobRequired);
	if (Status == MXST_FAILURE) {
		return MXST_FAILURE;
	}
	Status = Mx_GET_FEATURE(Mxic, FEATURES_ADDR_STATUS, &EccStatus);
	if (EccStatus & SR_ECC_STATUS_ERR_NO_COR)
	{
		Mxic->EccStatus.Failed++;
	}
	else if (EccStatus & SR_ECC_STATUS_ERR_COR)
	{
		if(Mxic->CmdList & MX_ECC_STAT_READ)
		{
			Status = Mx_ECC_STAT_READ(Mxic, &MaxBitflips);
			if (Status == MXST_FAILURE) {
					return MXST_FAILURE;
			}
			Mxic->EccStatus.Corrected += MaxBitflips;
		}
		else
		{
			MaxBitflips = 1;
			Mxic->EccStatus.Corrected ++;
		}
	}

	if(MaxBitflips)
	 	;

	return MXST_SUCCESS;
}

/*
 * Function:      Mx_WritePage_OndieEcc
 * Arguments:	  Mxic,        pointer to an mxchip structure of nor flash device.
 *                Addr,        device address to write
 *                Buf,         pointer to a data buffer where the write data is stored
 *                OobRequired, if write OOB is required
 * Return Value:  MXST_SUCCESS.
 *                MXST_FAILURE.
 *                MXST_TIMEOUT.
 * Description:    This function writes a specific page in the NAND device using on-die ECC block.
 * The ECC code is written into the spare Bytes of the page.
 */
int Mx_WritePage_OndieEcc(MxChip *Mxic, u32 Addr, u8 *Buf, int OobRequired)
{
	return Mx_Nand_Write_Page_Raw(Mxic, Addr, Buf, OobRequired);
}

/*
 * Function:      Mx_Nand_Transfer_Oob
 * Arguments:	  Mxic, pointer to an mxchip structure of nor flash device.
 *                oob,  oob destination address
 *                Ops,  oob ops structure
 *                Len,  size of oob to transfer
 * Return Value:  MXST_SUCCESS.
 *                MXST_FAILURE.
 * Description:   This function transfers oob to client buffer.
 */
static void Mx_Nand_Transfer_Oob(MxChip *Mxic, u8 *Oob,
  				  struct Mtd_Oob_Ops *Ops, size_t Len)
{
  	switch (Ops->Mode) {

  	case MTD_OPS_PLACE_OOB:
  	case MTD_OPS_RAW:
  		memcpy(Oob, Mxic->OobBufPtr + Ops->OobOffs, Len);
  		return;

  	default: {
  		struct NandOobFree *Free = Mxic->EccCtrl.Layout->OobFree;
  		uint32_t Boffs = 0, Roffs = Ops->OobOffs;
  		size_t Bytes = 0;

  		for (; Free->Length && Len; Free++, Len -= Bytes) {
  			/* Read request not from Offset 0? */
  			if (Roffs) {
  				if (Roffs >= Free->Length) {
  					Roffs -= Free->Length;
  					continue;
  				}
  				Boffs = Free->Offset + Roffs;
  				Bytes = min(Len, (Free->Length - Roffs));
  				Roffs = 0;
  			} else {
  				Bytes = min(Len, Free->Length);
  				Boffs = Free->Offset;
  			}
  			memcpy(Oob, Mxic->OobBufPtr + Boffs, Bytes);
  			Oob += Bytes;
  		}
  		return;
  	}
  }
}

/*
 * Function:      Mx_Nand_Fill_Oob
 * Arguments:	  Mxic, pointer to an mxchip structure of nor flash device.
 *                oob,  oob destination address
 *                Len,  oob data write Length
 *                Ops,  oob ops structure
 * Return Value:  MXST_SUCCESS.
 *                MXST_FAILURE.
 * Description:   This function transfers client buffer to oob.
 */
static void Mx_Nand_Fill_Oob(MxChip *Mxic, u8 *Oob,
  				u32 Len, struct Mtd_Oob_Ops *Ops)
{
  	memset(Mxic->OobBufPtr, 0xff, Mxic->OobSize);

  	switch (Ops->Mode) {

  	case MTD_OPS_PLACE_OOB:
  	case MTD_OPS_RAW:
  		memcpy(Mxic->OobBufPtr + Ops->OobOffs, Oob, Len);
  		return;

  	default: {
  		struct NandOobFree *Free = Mxic->EccCtrl.Layout->OobFree;
  		uint32_t Boffs = 0, Woffs = Ops->OobOffs;
  		size_t Bytes = 0;

  		for (; Free->Length && Len; Free++, Len -= Bytes) {
  			/* Write request not from Offset 0? */
  			if (Woffs) {
  				if (Woffs >= Free->Length) {
  					Woffs -= Free->Length;
  					continue;
  				}
  				Boffs = Free->Offset + Woffs;
  				Bytes = min(Len, (Free->Length - Woffs));
  				Woffs = 0;
  			} else {
  				Bytes = min(Len, Free->Length);
  				Boffs = Free->Offset;
  			}
  			memcpy(Mxic->OobBufPtr + Boffs, Oob, Bytes);
  			Oob += Bytes;
  		}
  		return;
  	}
  }
}

/*
 * Function:      Mx_Nand_Do_Write_Ops
 * Arguments:	  Mxic, pointer to an mxchip structure of nor flash device.
 *                Addr, Offset to write to
 *                oob,  operations description structure
 * Return Value:  MXST_SUCCESS.
 *                MXST_FAILURE.
 *                MXST_TIMEOUT.
 * Description:   This function writes data from buffer to flash.
 */
int Mx_Nand_Do_Write_Ops(MxChip *Mxic, u32 Addr, struct Mtd_Oob_Ops *Ops)
{
	int Status;
  	int Column;
  	int WriteLen = Ops->Len;

  	int OobWriteLen = Ops->OobLen;
  	int OobMaxLen = (Ops->Mode == MTD_OPS_AUTO_OOB) ?
  			Mxic->EccCtrl.Layout->OobAvail : Mxic->OobSize;

 	u8 *Oob = Ops->OobBuf;
	u8 *Buf = Ops->DatBuf;
	int OobRequired = Oob ? 1 : 0;

	Column = Addr & (Mxic->PageSize - 1);
	Addr &= ~0xFFF;

	while (1) {
		int Bytes = Mxic->PageSize;
		u8 *Wbuf = Buf;
		int Use_BufPoi;
		int Part_PageWr = (Column || WriteLen < (Mxic->PageSize - 1));

		if (Part_PageWr)
			Use_BufPoi = 1;
		else
			Use_BufPoi = 0;

		/* Partial page write? */
		if (Use_BufPoi) {
			if (Part_PageWr)
				Bytes = min(Bytes - Column, WriteLen);
			memset(Mxic->Buffers->DataBuf, 0xff, Mxic->PageSize);
			memcpy(&Mxic->Buffers->DataBuf[Column], Buf, Bytes);
			Wbuf = Mxic->Buffers->DataBuf;
		}

		if (Oob) {
			u8 Len = min(OobWriteLen, OobMaxLen);
			Mx_Nand_Fill_Oob(Mxic, Oob, Len, Ops);
			OobWriteLen -= Len;
		} else {
			/* We still need to erase leftover OOB data */
			memset(Mxic->OobBufPtr, 0xff, Mxic->OobSize);
		}

		Status = Mxic->EccCtrl.write_page(Mxic, Addr, Wbuf, OobRequired);
		if (Status == MXST_FAILURE)
			return MXST_FAILURE;

		WriteLen -= Bytes;
		if (!WriteLen)
			break;

		Column = 0;
		Buf += Bytes;
		Addr += PAGE_OFFSET;

	}

	Ops->RetLen = Ops->Len - WriteLen;
	if (Oob)
		Ops->OobRetLen = Ops->OobLen;

	return MXST_SUCCESS;
}

/*
 * Function:      Mx_Nand_Do_Read_Ops
 * Arguments:	  Mxic,  pointer to an mxchip structure of nor flash device.
 *                Addr,  Offset to read from
 *                oob,   operations description structure
 * Return Value:  MXST_SUCCESS.
 *                MXST_FAILURE.
 * Description:   This function reads data from flash to buffer.
 */
int Mx_Nand_Do_Read_Ops(MxChip *Mxic, u32 Addr, struct Mtd_Oob_Ops *Ops)
{
	int Status;
	int RealPage,Col, Bytes, Aligned, OobRequired;
	u32 ReadLen = Ops->Len;
	u32 OobReadLen = Ops->OobLen;
	u32 MaxOobSize = Ops->Mode == MTD_OPS_AUTO_OOB ?
			Mxic->OobAvail : Mxic->OobSize;

	u8 *bufpoi, *oob, *buf;
	u8 ecc_fail = FALSE;

	RealPage = (int)(Addr >> Mxic->page_shift);

	Col = (int)(Addr & (Mxic->PageSize - 1));
	Addr &= ~0xFFF;

	buf = Ops->DatBuf;
	oob = Ops->OobBuf;
	OobRequired = oob ? 1 : 0;
	{
		u32 ecc_failures = Mxic->EccStatus.Failed;
		Bytes = min(Mxic->PageSize - Col, ReadLen);
		Aligned = (Bytes == Mxic->PageSize);
		{

			bufpoi = Aligned ? buf : Mxic->Buffers->DataBuf ;

			/*
			 * Now read the page into the buffer.  Absent an error,
			 * the read methods return max bitflips per ecc step.
			 */
			if (Ops->Mode == MTD_OPS_RAW)
			{
				Status = Mxic->EccCtrl.read_page_raw(Mxic, Addr, bufpoi, OobRequired);
				if (Status == MXST_FAILURE)
					return MXST_FAILURE;
			}
			else
			{
				Status = Mx_Nand_Read_Page_Raw(Mxic, Addr, bufpoi, 0);
				if (Status == MXST_FAILURE)
					return MXST_FAILURE;
			}

			/* Transfer not Aligned data */
			if (!Aligned) {
				memcpy(buf, Mxic->Buffers->DataBuf + Col, Bytes);
			}

			buf += Bytes;

			if (oob) {
				int toread = min(OobReadLen, MaxOobSize);

				if (toread) {
					Mx_Nand_Transfer_Oob(Mxic, oob, Ops, toread);
					OobReadLen -= toread;
				}
			}

			if(Mxic->EccStatus.Failed - ecc_failures)
				ecc_fail = TRUE;
		}

		RealPage ++;
		ReadLen -= Bytes;

		/* For subsequent reads align to page boundary */
		Col = 0;
		/* Increment page address */
		Addr += PAGE_OFFSET;

	}

	Ops->RetLen = Ops->Len - (u32) ReadLen;
	if (oob)
		Ops->OobRetLen = Ops->OobLen - OobReadLen;

	if(ecc_fail)
	{
		return -EBADMSG;
	}

	return MXST_SUCCESS;
}

/*
 * Function:      Mx_Nand_Do_Read_Oob
 * Arguments:	  Mxic,  pointer to an mxchip structure of nor flash device.
 *                Addr,  Offset to read from
 *                oob,   operations description structure
 * Return Value:  MXST_SUCCESS.
 *                MXST_FAILURE.
 * Description:   This function reads out-of-band.
 */
int Mx_Nand_Do_Read_Oob(MxChip *Mxic, u32 Addr, struct Mtd_Oob_Ops *Ops)
{
	int ReadLen = Ops->OobLen;
	int Status = 0;
	int  MaxLen = (Ops->Mode == MTD_OPS_AUTO_OOB) ?
			Mxic->EccCtrl.Layout->OobAvail : Mxic->OobSize;
	int OobOffs = Ops->OobOffs;

	if (OobOffs >= MaxLen) {
		return -EINVAL;
	}

	/*read data from chip*/
	Status = Mx_Nand_Read_Oob_Std(Mxic, Addr);
	if (Status == MXST_FAILURE)
		return MXST_FAILURE;

	Mx_Nand_Transfer_Oob(Mxic, Ops->OobBuf, Ops, ReadLen);

	Ops->OobRetLen = ReadLen;

	return Status;
}


/*
 * Function:      Mx_Nand_Do_Write_Oob
 * Arguments:	  Mxic,  pointer to an mxchip structure of nor flash device.
 *                Addr,  Offset to write to
 *                oob,   operations description structure
 * Return Value:  MXST_SUCCESS.
 *                MXST_FAILURE.
 * Description:   This function writes out-of-band.
 */
int Mx_Nand_Do_Write_Oob(MxChip *Mxic, u32 Addr, struct Mtd_Oob_Ops *Ops)
{
	int Status = 0;
	int  MaxLen = (Ops->Mode == MTD_OPS_AUTO_OOB) ?
			Mxic->EccCtrl.Layout->OobAvail : Mxic->OobSize;
	int OobOffs = Ops->OobOffs;
	int WriteLen = Ops->OobLen;

	/* Do not allow write past end of page */
	if (OobOffs + WriteLen > MaxLen) {
		return -EINVAL;
	}
	if (OobOffs > MaxLen) {
		return -EINVAL;
	}

	Mx_Nand_Fill_Oob(Mxic, Ops->OobBuf, WriteLen, Ops);

	Status = Mx_Nand_Write_Oob_Std(Mxic, Addr);
	if (Status == MXST_FAILURE)
		return MXST_FAILURE;

	Ops->OobRetLen = WriteLen;

	return Status;
}

/*
 * Function:      Mx_Nand_Write_Page_Raw
 * Arguments:	  Mxic,  pointer to an mxchip structure of nor flash device.
 *                Addr,  Offset to write to
 *                Buf,   pointer to a data buffer where the write data is stored
 *                OobRequired, if write oob is required
 * Return Value:  MXST_SUCCESS.
 *                MXST_FAILURE.
 *                MXST_TIMEOUT.
 * Description:   This function writes raw page.
 */
int Mx_Nand_Write_Page_Raw(MxChip *Mxic, u32 Addr, u8 *Buf, int OobRequired)
{
	int Status;
	u32 PageSize = (OobRequired) ? Mxic->PageSize + Mxic->OobSize : Mxic->PageSize;
	u8 WriteBuf[2112] = {0};
	if(OobRequired)
	{
		memcpy(WriteBuf, Buf, Mxic->PageSize);
		memcpy(WriteBuf + Mxic->PageSize, Mxic->OobBufPtr, Mxic->OobSize);
	}
	Status = Mxic->AppGrp._WriteToCache(Mxic, Addr & ~0xFFF, PageSize, OobRequired ? WriteBuf : Buf, 0);
	if (Status == MXST_FAILURE)
		return Status;

	return Mx_PROGRAM_EXEC(Mxic, Addr);
}

/*
 * Function:      Mx_Nand_Write_Sub_Page_Raw
 * Arguments:	  Mxic,  pointer to an mxchip structure of nor flash device.
 *                Addr,  Offset to write to
 *                Buf,   pointer to a data buffer where the write data is stored
 *                OobRequired, if write oob is required
 * Return Value:  MXST_SUCCESS.
 *                MXST_FAILURE.
 *                MXST_TIMEOUT.
 * Description:   This function writes raw page.
 */
int Mx_Nand_Write_Sub_Page_Raw(MxChip *Mxic, u32 Addr, u32 ByteCount, u8 *Buf)
{
	int Status;
	Status = Mxic->AppGrp._WriteToCache(Mxic, Addr, ByteCount, Buf, 0);
	if (Status == MXST_FAILURE)
		return Status;

	return Mx_PROGRAM_EXEC(Mxic, Addr);
}

/*
 * Function:      Mx_Nand_Read_Page_Raw
 * Arguments:	  Mxic,  pointer to an mxchip structure of nor flash device.
 *                Addr,  Offset to read from
 *                Buf,   pointer to a data buffer where the read data will be stored
 *                OobRequired, if read oob is required
 * Return Value:  MXST_SUCCESS.
 *                MXST_FAILURE.
 * Description:   This function reads raw page.
 */
int Mx_Nand_Read_Page_Raw(MxChip *Mxic, u32 Addr, u8 *Buf, int OobRequired)
{
	int Status;
	u32 PageSize = (OobRequired) ? Mxic->PageSize + Mxic->OobSize : Mxic->PageSize;

	Status = Mx_READ(Mxic, Addr );
	if (Status == MXST_FAILURE)
		return Status;
	Status = Mxic->AppGrp._ReadFromCache(Mxic, Addr & ~0xFFF, PageSize, Buf, 0);
	if (Status == MXST_FAILURE)
		return Status;
	if(OobRequired)
	{
		memcpy(Mxic->OobBufPtr, Buf + Mxic->PageSize,  Mxic->OobSize);
	}
	 return Status;
}

/*
 * Function:      Mx_Nand_Read_Page_Raw
 * Arguments:	  Mxic,  pointer to an mxchip structure of nor flash device.
 *                Addr,  Offset to read from
 *                Buf,   pointer to a data buffer where the read data will be stored
 *                OobRequired, if read oob is required
 * Return Value:  MXST_SUCCESS.
 *                MXST_FAILURE.
 * Description:   This function reads raw page.
 */
int Mx_Nand_Read_Sub_Page_Raw(MxChip *Mxic, u32 Addr, u32 ByteCount, u8 *Buf)
{
	int Status;

	Status = Mx_READ(Mxic, Addr);
	if (Status == MXST_FAILURE)
		return Status;
	Status = Mxic->AppGrp._ReadFromCache(Mxic, Addr, ByteCount, Buf, 0);
	if (Status == MXST_FAILURE)
		return Status;
	 return Status;
}

/*
 * Function:      Mx_Nand_Write_Oob_Std
 * Arguments:	  Mxic,  pointer to an mxchip structure of nor flash device.
 *                Addr,  Offset to write to
 * Return Value:  MXST_SUCCESS.
 *                MXST_FAILURE.
 *                MXST_TIMEOUT.
 * Description:   This function reads raw page.
 */
int Mx_Nand_Write_Oob_Std(MxChip *Mxic, u32 Addr)
{
	int Status;

	Status = Mxic->AppGrp._WriteToCache(Mxic, (Addr & ~0xFFF) + Mxic->PageSize, Mxic->OobSize, Mxic->OobBufPtr, 0);
	if (Status == MXST_FAILURE)
		return Status;

	return Mx_PROGRAM_EXEC(Mxic, Addr);
}

/*
 * Function:      Mx_Nand_Read_Oob_Std
 * Arguments:	  Mxic,  pointer to an mxchip structure of nor flash device.
 *                Addr,  Offset to read from
 * Return Value:  MXST_SUCCESS.
 *                MXST_FAILURE.
 * Description:   This function reads raw page.
 */
int Mx_Nand_Read_Oob_Std(MxChip *Mxic, u32 Addr)
{
	int Status;

	Status = Mx_READ(Mxic, Addr);
	if (Status == MXST_FAILURE)
		return Status;
	return Mxic->AppGrp._ReadFromCache(Mxic, (Addr & ~0xFFF) + Mxic->PageSize, Mxic->OobSize, Mxic->OobBufPtr, 0);
}

/*
 * Function:      Mx_Nand_Block_Bad
 * Arguments:	  Mxic,  pointer to an mxchip structure of nor flash device.
 *                Addr,  block address to check
 * Return Value:  MXST_SUCCESS.
 *                MXST_FAILURE.
 * Description:   This function checks if block at Offset is bad.
 */
int Mx_Nand_Block_Bad(MxChip *Mxic, u32 Addr)
{
 	int Status;
  	struct Mtd_Oob_Ops Ops = {0};
  	u32 BlockAddr;
  	u8 Bad0 = 0;
  	u8 Bad1 = 0;

  	BlockAddr = Addr;
  	Ops.Mode = MTD_OPS_PLACE_OOB;
  	Ops.OobLen = 1;
  	Ops.OobBuf = &Bad0;

  	Status = Mx_Nand_Do_Read_Oob(Mxic, BlockAddr, &Ops);
 	if (Status == MXST_FAILURE) {
 		return MXST_FAILURE;
 	}

 	Ops.Mode = MTD_OPS_PLACE_OOB;
	Ops.OobLen = 1;
	Ops.OobBuf = &Bad1;

	Status = Mx_Nand_Do_Read_Oob(Mxic, BlockAddr + PAGE_OFFSET, &Ops);
	if (Status == MXST_FAILURE) {
		return MXST_FAILURE;
	}

  	if (Bad0 != 0xFF || Bad1 != 0xFF)
  		Status =  MXST_BLOCK_IS_BAD;
  	else
  		Status =  MXST_BLOCK_ISNOT_BAD;

  	return Status;
}

/*
 * Function:      Mx_Nand_Block_Mark_Bad
 * Arguments:	  Mxic,  pointer to an mxchip structure of SPI NAND device.
 *                Addr,  32 bit flash memory address of marking area
 * Return Value:  MXST_SUCCESS
 *                MXST_FAILURE
 *                MXST_TIMEOUT.
 * Description:   This function is for marking the bad block.
 */
int Mx_Nand_Block_Mark_Bad(MxChip *Mxic, u32 Addr)
{
 	int Status;
   	struct Mtd_Oob_Ops Ops = {0};
   	u32 BlockAddr;
   	u8 Buf0 = 0;
   	u8 Buf1 = 0;

 	/*erase bad block before mark bad block*/
 	Status = Mx_BE(Mxic, Addr);
 	if (Status == MXST_FAILURE) {
 		return MXST_FAILURE;
 	}

 	BlockAddr = Addr;
 	Ops.Mode = MTD_OPS_PLACE_OOB;
 	Ops.OobLen = 1;
 	Ops.OobBuf = &Buf0;

 	Status = Mx_Nand_Do_Write_Oob(Mxic, BlockAddr, &Ops);
 	if (Status == MXST_FAILURE) {
		return MXST_FAILURE;
	}

   	Ops.OobLen = 1;
	Ops.OobBuf = &Buf1;

	Status = Mx_Nand_Do_Write_Oob(Mxic, (BlockAddr + PAGE_OFFSET), &Ops);
	if (Status == MXST_FAILURE) {
		return MXST_FAILURE;
	}

	Mxic->EccStatus.BadBlocks++;

   	return Status;
}
