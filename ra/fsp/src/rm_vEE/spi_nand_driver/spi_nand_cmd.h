/**
  ******************************************************************************
  * @file    spi_nand_cmd.h
  * @author  MX Application Team
  * @date    10-July-2018
  * @brief   Header for spi_nand_cmd.c module.
  ******************************************************************************
  */

#ifndef SPI_NAND_CMD_H_
#define SPI_NAND_CMD_H_

#include "../spi_nand_driver/spi.h"

#define MX31LF1GE4BC_FLASH_SIZE                   0x4000000 /* 512 MBits => 64 MBytes */
#define MX31LF1GE4BC_BLOCK_SIZE                   2048*64   /* 64 PAGE */
#define MX31LF1GE4BC_PAGE_SIZE                    2048     /* 262144 pages of 256 bytes */
#define MX31LF1GE4BC_SUB_PAGE_SIZE                512     /* 262144 pages of 256 bytes */
#define MX31LF1GE4BC_CHUNK_SIZE                   0x10      /* fred: 16 bytes */

#define MX31LF1GE4BC_BLOCK_OFFSET				  0x40000
#define MX31LF1GE4BC_PAGE_OFFSET				  0x1000
#define MX31LF1GE4BC_SUB_PAGE_OFFSET			  0x200

/* get/set features address definition */
#define  FEATURES_ADDR_CONFIG       	0x10
#define  FEATURES_ADDR_BLOCK_PROTECTION	0xA0
#define  FEATURES_ADDR_SECURE_OTP		0xB0
#define  FEATURES_ADDR_STATUS			0xC0

/* status register definition */
#define  SR_OIP            	  0x01
#define  SR_WEL			   	  0x02
#define  SR_ERASE_FAIL     	  0x04
#define  SR_PROGRAM_FAIL   	  0x08
#define  SR_ECC_STATUS_MASK   0x30
#define  SR_ECC_STATUS_NO_ERR   	0x00
#define  SR_ECC_STATUS_ERR_COR   	0x10
#define  SR_ECC_STATUS_ERR_NO_COR   0x20

/* secure OTP register definition */
#define  SECURE_OTP_QE               0x01
#define  SECURE_OTP_ECC_EN		     0x10
#define  SECURE_OTP_SECURE_OTP_EN    0x40
#define  SECURE_OTP_SECURE_OTP_PROT  0x80

/* block protection register definition */
#define  BLOCK_PROT_SP          0x01
#define  BLOCK_PROT_COMPLE		0x02
#define  BLOCK_PROT_INVERT   	0x04
#define  BLOCK_PROT_BP0  		0x08
#define  BLOCK_PROT_BP1  		0x10
#define  BLOCK_PROT_BP2  		0x20
#define  BLOCK_PROT_BPRWD  		0x80
#define  BLOCK_PROT_BP_MASK  	0x38
#define  BLOCK_PROT_BP_OFFSET  	3
#define  BLOCK_PROT_COMPLE_OFFSET  	1

#define  BLOCK_PROT_SP_ENABLE	1
#define  BLOCK_PROT_SP_DISABLE	0
#define  LOCK_TIGHT_ENABLE		1
#define  LOCK_TIGHT_DISABLE		0

#define  WRAP_BIT_0  		0x00
#define  WRAP_BIT_1  		0x01
#define  WRAP_BIT_2  		0x02
#define  WRAP_BIT_3  		0x03

#define PERFORMANCE_TEST_SZ 2048
#define TEST_SZ 2112

#define PAGE_OFFSET  0x1000
#define BLOCK_OFFSET 0x40000
#define PAGE_MASK    0x00040FFF  //bit 16 is plane select bit

#define SPI_NAND_FLASH_MAX_ID_LEN 2
#define PLANE_SELECT_BIT_MASK 0x40000
#define PLANE0	0
#define PLANE1	1
#define NAND_FLASH_TIMEOUT_VALUE   1000000    /* 1S */

#define NAND_MAX_BLOCKS		32768	/**< Max number of Blocks */
#define NAND_MAX_PAGE_SIZE	16384	/**< Max page size of NAND flash */
#define NAND_MAX_OOB_SIZE	512	    /**< Max OOB bytes of a NAND flash page */
#define NAND_ECC_SIZE		512	    /**< ECC block size */
#define NAND_ECC_BYTES		7	    /**< ECC bytes per ECC block */

#define NAND_PAGE_SIZE_512		512	    /**< Page size 512 */
#define NAND_PAGE_SIZE_1024		1024	/**< Page size 1024 */
#define NAND_PAGE_SIZE_2048		2048	/**< Page size 2048 */
#define NAND_PAGE_SIZE_4096		4096	/**< Page size 4096 */
#define NAND_PAGE_SIZE_8192		8192	/**< Page size 8192 */

#define NAND_OOB_SIZE_8		    8	/**< OOB bytes size 8 */
#define NAND_OOB_SIZE_16		16	/**< OOB bytes size 16 */
#define NAND_OOB_SIZE_32		32	/**< OOB bytes size 32 */
#define NAND_OOB_SIZE_64		64	/**< OOB bytes size 64 */
#define NAND_OOB_SIZE_128		128	/**< OOB bytes size 128 */
#define NAND_OOB_SIZE_256		256	/**< OOB bytes size 256 */

struct _MxChip;

typedef struct {
	int (*_HardwareInit)(struct _MxChip *, u32 EffectiveAddr);
	int (*_WriteToCache)(struct _MxChip *, u32 Addr, u32 Cnt, u8 *Buf, u8 WrapBit);
	int (*_ReadFromCache)(struct _MxChip *, u32 Addr, u32 Cnt, u8 *Buf, u8 WrapBit);

} AppGrp;


/*
 * This enum contains ECC Mode
 */
typedef enum {
	NAND_ECC_NONE,
	NAND_ECC_SOFT,
	NAND_ECC_HW,
	NAND_ECC_HW_SYNDROME,
	NAND_ECC_HW_OOB_FIRST,
	NAND_ECC_SOFT_BCH,
	NAND_ECC_ONDIE	/**< On-Die ECC */
} Nand_EccMode;

struct NandEccCtrl {
	Nand_EccMode Mode;
	u32 EccSteps;		/**< Number of ECC steps for the flash page */
	u32 EccSize;		/**< ECC size */
	u32 EccBytes;	    /**< Number of ECC bytes for a block */
	u32 EccTotalBytes;	/**< Total number of ECC bytes for Page */
	u32 Strength;
	struct NandEccLayout	*Layout;
	void *Priv;

	int (*read_page)(struct _MxChip *, u32 Addr, u8 *Buf, int Oob_Required);
	int (*write_page)(struct _MxChip *, u32 Addr, u8 *Buf, int Oob_Required);
	int (*read_page_raw)(struct _MxChip *, u32 Addr, u8 *Buf, int Oob_Required);
	int (*write_page_raw)(struct _MxChip *, u32 Addr, u8 *Buf, int Oob_Required);
	int (*read_oob)(struct _MxChip *, u32 Addr);
	int (*write_oob)(struct _MxChip *, u32 Addr);
};

/**
 * struct nand_buffers - buffer structure for read/write
 * @ecccalc:	buffer for calculated ECC
 * @ecccode:	buffer for ECC read from flash
 * @databuf:	buffer for data - dynamically sized
 *
 * Do not change the order of buffers. databuf and oobrbuf must be in
 * consecutive order.
 */
struct NandBuffers {
	u8	EccCalc[NAND_MAX_OOB_SIZE];	/**< Buffer for calculated ECC */
	u8	EccCode[NAND_MAX_OOB_SIZE]; /**< Buffer for stored ECC */
	u8  DataBuf[530 ];
};

struct NandOobFree {
	u32 Offset;
	u32 Length;
};

#define MTD_MAX_OOBFREE_ENTRIES	8

/*
 * ECC layout control structure. Exported to user space for
 * diagnosis and to allow creation of raw images
 */
struct NandEccLayout {
	u32 EccBytes;
	u32 EccPos[128];
	u32 OobAvail;
	struct NandOobFree OobFree[MTD_MAX_OOBFREE_ENTRIES];
};

struct NandEccStatus {
	u32 Corrected;
	u32 Failed;
	u32 BadBlocks;
	u32 BbtBlocks;
};

typedef struct _MxChip{
	void *Priv;
	AppGrp AppGrp;
	char *Name;
	u8 *Id;
	u32 ChipSz;
	u32 BlockSz;
	u32 ChipSpclFlag;
	u32 CmdList;
	u32 CurFreq;
	u32 PageSize;       			/**< Bytes per page */
	u16 OobSize;   					/**< Size of OOB area in bytes */
	u32 OobAvail;
	u8 *OobBufPtr;					/**< Pointer to store OOB buffer */
	u8  page_shift;
	struct NandEccCtrl EccCtrl;		/**< ECC configuration parameters */
	struct NandBuffers *Buffers;
	struct NandEccStatus EccStatus;
	u8	EccCalc1[NAND_MAX_OOB_SIZE];	/**< Buffer for calculated ECC */

} MxChip;

enum MX_NAND_CMDS {
	MX_READ_ID        				  = 0x00000001,
	MX_SET_FEATURE                    = 0x00000002, /* include GET_FEATURE */
	MX_ECC_STAT_READ   				  = 0x00000004,
	MX_PAGE_READ       				  = 0x00000008,
	MX_READ_CACHE		      		  = 0x00000010,
	MX_READ_CACHE2       			  = 0x00000020,
	MX_READ_CACHE4	  	  		  	  = 0x00000040,
	MX_READ_CACHE_SEQUENTIAL   	      = 0x00000080,
	MX_READ_CACHE_END				  = 0x00000100,
	MX_WREN							  = 0x00000200, /* include WRDI */
	MX_PROGRAM_EXEC 				  = 0x00000400,
	MX_PP_LOAD			      		  = 0x00000800,
	MX_PP_RAND_LOAD		      		  = 0x00001000,
	MX_4PP_RAND_LOAD		      	  = 0x00002000,
	MX_4PP_LOAD		      			  = 0x00004000,
	MX_BE			      			  = 0x00008000,
	MX_RESET		      			  = 0x00010000,
};

#define MX_CMD_RDID					 0x9F		/* Read Identification */
#define MX_CMD_READ					 0x13
#define MX_CMD_READ_CACHE			 0x0B
#define MX_CMD_READ_CACHE2			 0x3B
#define MX_CMD_READ_CACHE4			 0x6B
#define MX_CMD_READ_CACHE_SEQUENTIAL 0x31
#define MX_CMD_READ_CACHE_END		 0x3F
#define MX_CMD_RD_ECC_STATUS		 0x7C

#define MX_CMD_GET_FEATURE			 0x0F
#define MX_CMD_SET_FEATURE			 0x1F

#define MX_CMD_WREN                  0x06
#define MX_CMD_WRDI                  0x04
#define MX_CMD_PP_LOAD				 0x02
#define MX_CMD_PP_RAND_LOAD			 0x84
#define MX_CMD_4PP_LOAD				 0x32
#define MX_CMD_4PP_RAND_LOAD	     0x34
#define MX_CMD_PROGRAM_EXEC			 0x10

#define MX_CMD_BE					 0xD8
#define MX_CMD_RESET				 0xFF
#define MX_CMD_ECC_STAT_READ		 0x7C
#define MX_CMD_ECC_WARNING   		 0xA9

int Mx_RDID(MxChip *Mxic, u8 *Buf);
int Mx_READ(MxChip *Mxic, u32 Addr);

int MxWaitForFlashReady(MxChip *Mxic,u32 ExpectTime);
int Mx_GET_FEATURE(MxChip *Mxic, u32 Addr, u8 *Buf);
int Mx_SET_FEATURE(MxChip *Mxic, u32 Addr, u8 *Buf);
int Mx_READ_CACHE(MxChip *Mxic, u32 Addr, u32 ByteCount, u8 *Buf, u8 WrapBit);
int Mx_READ_CACHE2(MxChip *Mxic, u32 Addr, u32 ByteCount, u8 *Buf, u8 WrapBit);
int Mx_READ_CACHE4(MxChip *Mxic, u32 Addr, u32 ByteCount, u8 *Buf, u8 WrapBit);
int Mx_READ_CACHE_SEQUENTIAL(MxChip *Mxic);
int Mx_READ_CACHE_END(MxChip *Mxic);

int Mx_WREN(MxChip *Mxic);
int Mx_WRDI(MxChip *Mxic);
int Mx_PP_LOAD(MxChip *Mxic, u32 Addr, u32 ByteCount, u8 *Buf, u8 WrapBit);
int Mx_PP_RAND_LOAD(MxChip *Mxic, u32 Addr, u32 ByteCount, u8 *Buf, u8 WrapBit);
int Mx_4PP_LOAD(MxChip *Mxic, u32 Addr, u32 ByteCount, u8 *Buf, u8 WrapBit);
int Mx_4PP_RAND_LOAD(MxChip *Mxic, u32 Addr, u32 ByteCount, u8 *Buf, u8 WrapBit);
int Mx_PROGRAM_EXEC(MxChip *Mxic, u32 Addr);
int Mx_BE(MxChip *Mxic, u32 Addr);
int Mx_RESET(MxChip *Mxic);

int MxIsOnDieEccEnable(MxChip *Mxic);
int MxEnableOnDieEcc(MxChip *Mxic);
int MxDisableOnDieEcc(MxChip *Mxic);
int Mx_ECC_STAT_READ(MxChip *Mxic, u8 *Buf);

#endif /* NOR_CMD_H_ */
