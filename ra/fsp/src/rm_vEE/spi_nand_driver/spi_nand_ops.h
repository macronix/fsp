/**
  ******************************************************************************
  * @file    spi_nand_ops.h
  * @author  MX Application Team
  * @date    10-July-2018
  * @brief	 Header for spi_nand_ops.c module.
  ******************************************************************************
  */

#ifndef SPI_NAND_OPS_H_
#define SPI_NAND_OPS_H_

#include "../spi_nand_driver/spi_nand_cmd.h"

enum READ_MODE{
	 READ_1IO        = 0x00,
	 READ_2IO 	     = 0x01,
	 READ_4IO		 = 0x02,
};

enum WRITE_MODE{
	WRITE_1IO        = 0x00,
	WRITE_1IO_RAND   = 0x01,
	WRITE_4IO		 = 0x02,
	WRITE_4IO_RAND   = 0x03,
};


enum FlashLockStatus {
	FLASH_IS_UNLOCKED          = 14,
	FLASH_IS_LOCKED            = 15,
	FLASH_IS_PARTIALLY_LOCKED  = 16,
	FLASH_IS_LOCKED_TIGHT      = 17,
};
struct Mtd_Oob_Ops {
	u8      Mode;
	u32		Len;
	u32		RetLen;
	u32		OobLen;
	u32		OobRetLen;
	u32		OobOffs;
	u8		*DatBuf;
	u8		*OobBuf;
};

/*oob operation mode*/
enum {
	MTD_OPS_PLACE_OOB = 0,
	MTD_OPS_AUTO_OOB = 1,
	MTD_OPS_RAW = 2,
};

#define	OOB_REQUIRED		1
#define	OOB_NOT_REQUIRED	0

#define min(a,b) (((a)<(b))?(a):(b))
#define max(a,b) (((a)>(b))?(a):(b))
/*
 * here is used to check boundary of 64K/4K protection area
 */
#define MX_64KB					(1 << 16)
#define MX_4KB					(1 << 12)
#define MX_WP64K_FIRST			MX_64KB
#define MX_WP64K_LAST(mtd_sz)	((mtd_sz) - MX_64KB)

int MxSoftwareInit(MxChip *Mxic, spi_flash_instance_t * controller);
int MxChipReset(MxChip *Mxic);

int MxLock(MxChip *Mxic, u32 Addr, u64 Len);
int MxUnlock(MxChip *Mxic, u32 Addr, u64 Len);
int MxIsLocked(MxChip *Mxic, u32 Addr, u64 Len);

int MxIdMatch(MxChip *Mxic);

int Mx_Nand_EccInit(MxChip *Mxic, u8 EccMode);
int Mx_ReadPage_SwEcc(MxChip *Mxic,u32 Addr, u8 *Buf, int OobRequired);
int Mx_WritePage_SwEcc(MxChip *Mxic,u32 Addr, u8 *Buf, int OobRequired);
int Mx_ReadPage_OndieEcc(MxChip *Mxic, u32 Addr, u8 *Buf, int OobRequired);
int Mx_WritePage_OndieEcc(MxChip *Mxic, u32 Addr, u8 *Buf, int OobRequired);

int Mx_Nand_Do_Write_Ops(MxChip *Mxic, u32 Addr, struct Mtd_Oob_Ops *Ops);
int Mx_Nand_Do_Read_Ops(MxChip *Mxic, u32 Addr, struct Mtd_Oob_Ops *Ops);

int Mx_Nand_Write_Page_Raw(MxChip *Mxic, u32 Addr, u8 *Buf, int OobRequired);
int Mx_Nand_Write_Sub_Page_Raw(MxChip *Mxic, u32 Addr, u32 ByteCount, u8 *Buf);
int Mx_Nand_Read_Page_Raw(MxChip *Mxic, u32 Addr, u8 *Buf, int OobRequired);
int Mx_Nand_Read_Sub_Page_Raw(MxChip *Mxic, u32 Addr, u32 ByteCount, u8 *Buf);

int Mx_Nand_Read_Oob_Std(MxChip *Mxic, u32 Addr);
int Mx_Nand_Write_Oob_Std(MxChip *Mxic, u32 Addr);
int Mx_Nand_Do_Read_Oob(MxChip *Mxic, u32 Addr, struct Mtd_Oob_Ops *Ops);
int Mx_Nand_Do_Write_Oob(MxChip *Mxic, u32 Addr, struct Mtd_Oob_Ops *Ops);

int Mx_Nand_Block_Bad(MxChip *Mxic, u32 Addr);
int Mx_Nand_Block_Mark_Bad(MxChip *Mxic, u32 Addr);
#endif /* NOR_OPS_H_ */
