/*
 * Macronix Read-While-Write EEPROM Emulator.
 *
 * Copyright (c) 2018, Macronix International Co, Ltd.
 * This code is confidential, please DO NOT redistribute or publish it.
 *
 * Attention: This module is still in development, and WITHOUT ANY WARRANTY.
 * Please only use it as a reference.
 */

#ifndef RM_VEE_H_
#define RM_VEE_H_

#include <rm_vee_api.h>
#include "rm_vee_cfg.h"

#ifdef SPINAND
#include "../../src/rm_vEE/spi_nand_driver/spi_nand_app.h"

#ifndef QSPI_CONTROL
#error "SPINand should use QSPI controller!"
#endif

#endif

void assert_failed(uint8_t* file, uint32_t line);
fsp_err_t get_flash_status(mxic_vee_ctrl_t * const p_api_ctrl);

/* Use FreeRTOS */
typedef TimeOut_t osTimeOut;
#define osTaskDelay(xTicksToDelay) vTaskDelay(xTicksToDelay)
#define osTaskSetTimeOutState(pxTimeOut) vTaskSetTimeOutState(pxTimeOut)
#define osTaskCheckForTimeOut(pxTimeOut, pxTicksToWait) xTaskCheckForTimeOut(pxTimeOut, pxTicksToWait)

/* Flash specifications */
#ifdef SPINAND
#define MX_FLASH_BLOCK_OFFSET           MX31LF1GE4BC_BLOCK_OFFSET
#define MX_FLASH_PAGE_OFFSET            MX31LF1GE4BC_PAGE_OFFSET
#define MX_FLASH_SUB_PAGE_OFFSET        MX31LF1GE4BC_SUB_PAGE_OFFSET
#endif

#ifdef MX_FLASH_SUPPORT_RWW
/* Typical PP/SE/BE busy ticks */
#define MX_FLASH_PAGE_WRITE_TICKS       (1 / portTICK_PERIOD_MS)    /* include SW delay */
#define MX_FLASH_SECTOR_ERASE_TICKS     (24 / portTICK_PERIOD_MS)
#define MX_FLASH_BLOCK_ERASE_TICKS      (220 / portTICK_PERIOD_MS)
#endif

/* EEPROM physical layout */
#ifdef SPINAND
#define MX_EEPROM_SECTOR_SIZE           MX_FLASH_BLOCK_SIZE
#define MX_EEPROM_SECTOR_OFFSET         MX_FLASH_BLOCK_OFFSET

#define MX_EEPROM_CLUSTER_SIZE          (MX_EEPROM_SECTOR_SIZE * MX_EEPROM_SECTORS_PER_CLUSTER)
#define MX_EEPROM_CLUSTER_OFFSET        (MX_EEPROM_SECTOR_OFFSET * MX_EEPROM_SECTORS_PER_CLUSTER)
#define MX_EEPROM_BANK_SIZE             (MX_EEPROM_CLUSTER_SIZE * MX_EEPROM_CLUSTERS_PER_BANK)
#else
#define MX_EEPROM_CLUSTER_SIZE          (MX_FLASH_SECTOR_SIZE * MX_EEPROM_SECTORS_PER_CLUSTER)
#define MX_EEPROM_BANK_SIZE             (MX_EEPROM_CLUSTER_SIZE * MX_EEPROM_CLUSTERS_PER_BANK)
#endif

#if defined(MX_FLASH_BANKS) && (MX_EEPROM_BANKS > MX_FLASH_BANKS)
#error "too many banks!"
#endif

#if (MX_EEPROM_SECTORS_PER_CLUSTER > 256)
#error "too many sectors per cluster!"
#endif

/* EEPROM parameters */
#ifdef SPINAND
#define MX_EEPROM_HEADER_SIZE           (4)
#define MX_EEPROM_PAGE_SIZE             (MX_EEPROM_ENTRY_SIZE - MX_EEPROM_HEADER_SIZE)
#define MX_EEPROM_ENTRIES_PER_SECTOR    ((MX_EEPROM_SECTOR_SIZE / MX_EEPROM_ENTRY_SIZE) - 6) //2 is for system entry, 4 is for write gc markers
#define MX_EEPROM_SYSTEM_ENTRY_OFFSET   (0x800)
#define MX_EEPROM_SYSTEM_ENTRY_SIZE     (16)
#define MX_EEPROM_DATA_SECTORS          (MX_EEPROM_SECTORS_PER_CLUSTER)
#define MX_EEPROM_ENTRIES_PER_CLUSTER   (MX_EEPROM_ENTRIES_PER_SECTOR * MX_EEPROM_DATA_SECTORS)
#define MX_EEPROM_LPAS_PER_SECTOR       (8)
#define MX_EEPROM_LPAS_PER_CLUSTER      (MX_EEPROM_DATA_SECTORS - MX_EEPROM_FREE_SECTORS) * MX_EEPROM_LPAS_PER_SECTOR
#define MX_EEPROM_BLOCK_SIZE            (MX_EEPROM_PAGE_SIZE * MX_EEPROM_LPAS_PER_CLUSTER)
#define MX_EEPROM_BLOCKS                (MX_EEPROM_CLUSTERS_PER_BANK)
#define MX_EEPROM_SIZE                  (MX_EEPROM_BLOCK_SIZE * MX_EEPROM_BLOCKS)
#define MX_EEPROMS                      (MX_EEPROM_BANKS)
#define MX_EEPROM_TOTAL_SIZE            (MX_EEPROM_SIZE * MX_EEPROMS)
#else
#define MX_EEPROM_HEADER_SIZE           (4)
#define MX_EEPROM_PAGE_SIZE             (MX_EEPROM_ENTRY_SIZE - MX_EEPROM_HEADER_SIZE)
#define MX_EEPROM_ENTRIES_PER_SECTOR    (MX_FLASH_SECTOR_SIZE / MX_EEPROM_ENTRY_SIZE)
#define MX_EEPROM_SYSTEM_SECTOR         (MX_EEPROM_SECTORS_PER_CLUSTER - 1)
#define MX_EEPROM_SYSTEM_SECTOR_OFFSET  (MX_EEPROM_SYSTEM_SECTOR * MX_FLASH_SECTOR_SIZE)
#define MX_EEPROM_SYSTEM_ENTRY_SIZE     (16)
#define MX_EEPROM_SYSTEM_ENTRIES        (MX_FLASH_SECTOR_SIZE / MX_EEPROM_SYSTEM_ENTRY_SIZE)
#define MX_EEPROM_DATA_SECTORS          (MX_EEPROM_SYSTEM_SECTOR)
#define MX_EEPROM_FREE_SECTORS          (1)
#define MX_EEPROM_ENTRIES_PER_CLUSTER   (MX_EEPROM_ENTRIES_PER_SECTOR * MX_EEPROM_DATA_SECTORS)
#define MX_EEPROM_LPAS_PER_CLUSTER      (MX_EEPROM_DATA_SECTORS - MX_EEPROM_FREE_SECTORS)
#define MX_EEPROM_BLOCK_SIZE            (MX_EEPROM_PAGE_SIZE * MX_EEPROM_LPAS_PER_CLUSTER)
#define MX_EEPROM_BLOCKS                (MX_EEPROM_CLUSTERS_PER_BANK)
#define MX_EEPROM_SIZE                  (MX_EEPROM_BLOCK_SIZE * MX_EEPROM_BLOCKS)
#define MX_EEPROMS                      (MX_EEPROM_BANKS)
#define MX_EEPROM_TOTAL_SIZE            (MX_EEPROM_SIZE * MX_EEPROMS)
#endif

#if (MX_EEPROM_ENTRY_SIZE < MX_FLASH_CHUNK_SIZE)
#error "too small data entry size!"
#endif

#if (MX_EEPROM_ENTRIES_PER_SECTOR > 256)
#error "too many entries per sector!"
#endif

#if (MX_EEPROM_SYSTEM_ENTRY_SIZE < MX_FLASH_CHUNK_SIZE)
#error "too small system entry size!"
#endif


/* Wear leveling interval */
#define MX_EEPROM_WL_INTERVAL           10000

/* Background thread */
#ifdef MX_EEPROM_BACKGROUND_THREAD
#define MX_EEPROM_BG_THREAD_PRIORITY    1                           /* Background thread priority */
#define MX_EEPROM_BG_THREAD_STACK_SIZE  256                         /* Background thread stack size */
#define MX_EEPROM_BG_THREAD_DELAY       10000                       /* Background thread delay (ms) */
#define MX_EEPROM_BG_THREAD_TIMEOUT     10                          /* Background thread timeout (ms) */
#endif

#if !defined(MX_FLASH_SUPPORT_RWW) && !defined(MX_OSPI_FLASH_NORMAL)
#ifdef MX_EEPROM_ECC_CHECK
#error "ECC Check only for OctaFlash"
#endif
#endif

#ifdef MX_EEPROM_PC_PROTECTION

#ifndef MX_EEPROM_CRC_HW
#error "CRC Not Define"
#endif

#ifndef SPINAND
#if !defined(MX_FLASH_SUPPORT_RWW) && !defined(MX_EEPROM_ECC_CHECK)
#ifdef MX_OSPI_FLASH_NORMAL
#error "ECC Not Define"
#endif
#endif
#else
#if defined(MX_FLASH_SUPPORT_RWW) || defined(MX_EEPROM_ECC_CHECK)
#error "SPI_Nand Not support RWW and ECC"
#endif
#endif

#ifndef MX_EEPROM_READ_ROLLBACK
#error "ROLLBACK Not Define"
#endif

#endif

#if defined(MX_FLASH_SUPPORT_RWW) && defined(MX_EEPROM_ECC_CHECK)
#error "cannot read ECC status in RWW mode!"
#endif

#ifdef MX_EEPROM_CRC_HW
#define CRC16_POLY                      (0x1021)
#define CRC16_DATA_LENGTH               (MX_EEPROM_PAGE_SIZE / 4)
#endif

#define MX_EEPROM_WRITE_RETRIES         2    /* number of write retries */

#define MX_EEPROM_READ_RETRIES          2    /* number of read retries */

#if defined(MX_EEPROM_READ_ROLLBACK) && (MX_EEPROM_READ_RETRIES == 0)
#error "please set the number of read retries!"
#endif


#if defined(MX_GENERIC_RWW) && !defined(MX_FLASH_SUPPORT_RWW)
#error "please enable RWW feature!"
#endif

/* RWWEE ID: "MX" */
#define MFTL_ID            0x4D58

/* Maximum unsigned value */
#define DATA_NONE8         0xff
#define DATA_NONE16        0xffff
#define DATA_NONE32        0xffffffffUL

#ifdef SPINAND
#define NOT_EXIT           0xfe

/* System Entry Addr*/
#define SYS_ENTRY_ADDR_E_E      0                                      /* OPS_ERASE_END */
#define SYS_ENTRY_ADDR_G_B_D    0 + MX_EEPROM_SYSTEM_ENTRY_SIZE        /* OPS_GC_CP_BEGIN + source sector addr (write in Gc destination sector)*/
#define SYS_ENTRY_ADDR_G_B_S    63 * MX_FLASH_PAGE_OFFSET + 2048       /* OPS_GC_CP_BEGIN + des sector addr (write in Gc source sector)*/
#define SYS_ENTRY_ADDR_G_E      63 * MX_FLASH_PAGE_OFFSET + 2048 + MX_EEPROM_SYSTEM_ENTRY_SIZE     /* OPS_GC_CP_END (write in Gc source sector)*/
#define SYS_ENTRY_ADDR_E_B      63 * MX_FLASH_PAGE_OFFSET + 2048 + MX_EEPROM_SYSTEM_ENTRY_SIZE * 2 /* OPS_ERASE_END */
#endif


extern osMutexId UartLock;
#define pr_time(fmt, ...) ({                                \
	xSemaphoreTake(UartLock, portMAX_DELAY);                   \
    printf("[%lu] " fmt, osKernelSysTick(), ##__VA_ARGS__); \
    xSemaphoreGive(UartLock); })

#ifdef MX_DEBUG
#define mx_log(...)
#define mx_info(args...) do {pr_time(args);} while(0)
#define mx_err(args...) do {pr_time(args);} while(0)
#else
#define mx_log(...)
#define mx_info(...)
#define mx_err(...)
#endif

/* Macros */
#undef min_t
#define min_t(type, x, y) ({ \
    type __min1 = (x);       \
    type __min2 = (y);       \
    __min1 < __min2 ? __min1: __min2; })

/* RWWEE operations */
typedef enum
{
  OPS_READ        = 0x5244,
  OPS_WRITE       = 0x7772,
  OPS_ERASE_BEGIN = 0x4553,
  OPS_ERASE_END   = 0x6565,
#ifdef SPINAND
  OPS_GC_CP_BEGIN = 0xAAAA,
  OPS_GC_CP_END   = 0x5555,
#endif
  OPS_NONE        = 0x4E4E,
} rwwee_ops;

#pragma pack(1)    /* byte alignment */

/* System Entry */
struct system_entry
{
  uint16_t id;
  uint16_t ops;
  uint16_t arg;
  uint16_t cksum;
//  uint8_t pad[MX_EEPROM_SYSTEM_ENTRY_SIZE - 8];
};

/* Data entry header */
struct eeprom_header
{
  uint8_t LPA;
  uint8_t LPA_inv;
  uint16_t crc;
  uint8_t pad[MX_EEPROM_HEADER_SIZE - 4];
};

/* Data Entry */
struct eeprom_entry
{
  struct eeprom_header header;
  uint8_t data[MX_EEPROM_PAGE_SIZE];
};

#pragma pack()    /* default alignment */

/* Bank information */
struct bank_info
{
  uint32_t bank;              /* current bank */
  uint32_t bank_offset;       /* bank address */

  uint32_t block;             /* current block */
  uint32_t block_offset;      /* block address */
  struct eeprom_entry cache;  /* entry cache */
  bool cache_dirty;           /* cache status */

  /* address mapping */
  uint8_t l2ps[MX_EEPROM_LPAS_PER_CLUSTER];
  uint8_t l2pe[MX_EEPROM_LPAS_PER_CLUSTER];
  uint8_t p2l[MX_EEPROM_DATA_SECTORS];    /* TODO: bitmap */
#ifdef SPINAND
  uint8_t latest_used_entry_per_sector[MX_EEPROM_DATA_SECTORS];
  uint8_t l2ps_group[MX_EEPROM_DATA_SECTORS];
#endif

  uint32_t dirty_block;       /* obsoleted sector to be erased */
  uint32_t dirty_sector;      /* obsoleted sector to be erased */

  /* system entry address */
  uint32_t sys_entry[MX_EEPROM_BLOCKS];

  osMutexId lock;             /* bank mutex lock */

#ifdef MX_DEBUG
  /* sector erase count statistics */
  uint32_t eraseCnt[MX_EEPROM_BLOCKS][MX_EEPROM_SECTORS_PER_CLUSTER];
#endif
};

/* RWW information */
struct rww_info
{
  bool initialized;           /* EEPROM status */

#ifdef MX_FLASH_SUPPORT_RWW
  uint32_t busyBank;          /* current busy bank */
  TickType_t waitTicks;       /* ticks to wait */
  osTimeOut timeout;          /* busy start time */
  osMutexId busyLock;         /* RWW lock */
#endif

  osMutexId deviceLock;       /* device lock */;
};


/** Instance control block.  This is private to the FSP and should not be used or modified by the application. */
typedef struct st_mxic_vee_instance_ctrl
{
    uint32_t                open;
    uint32_t                init;     /* EEPROM status */
    mxic_vee_cfg_t const    * p_cfg;
    struct bank_info bi[MX_EEPROMS];  /* bank info */
    osMutexId crcLock;                /* HW CRC mutex lock */
    uint32_t rwCnt;                   /* User R/W statistics */
#ifdef MX_EEPROM_BACKGROUND_THREAD
    bool bgStart;                     /* Background thread control */
    TaskHandle_t bgThreadID;            /* Background thread ID */
#endif
#ifdef SPINAND
    MxChip Mxic;
#endif

} mxic_vee_instance_ctrl_t;

/** User configuration structure, used in open function */
typedef struct st_mxic_vee_ext_cfg
{
#ifdef MX_EEPROM_CRC_HW
    crc_instance_t const * p_crc;  ///< Pointer to a crc instance
#endif

#if defined(QSPI_CONTROL) || defined(OSPI_CONTROL)
    spi_flash_instance_t const * p_controller;
#endif

} mxic_vee_ext_cfg_t;

/* eeprom.c */
fsp_err_t RM_VEE_Read(mxic_vee_ctrl_t * const p_api_ctrl, uint32_t addr, uint32_t len, uint8_t *buf);
fsp_err_t RM_VEE_Write(mxic_vee_ctrl_t * const p_api_ctrl, uint32_t addr, uint32_t len, uint8_t *buf);
fsp_err_t RM_VEE_WriteBack(mxic_vee_ctrl_t * const p_api_ctrl);
fsp_err_t RM_VEE_SyncWrite(mxic_vee_ctrl_t * const p_api_ctrl, uint32_t addr, uint32_t len, uint8_t *buf);
fsp_err_t RM_VEE_Flush(mxic_vee_ctrl_t * const p_api_ctrl);
void RM_VEE_Background(mxic_vee_ctrl_t * const p_api_ctrl);
fsp_err_t RM_VEE_Format(mxic_vee_ctrl_t * const p_api_ctrl);
void RM_VEE_GetParam(struct eeprom_param *param);
fsp_err_t RM_VEE_Open(mxic_vee_ctrl_t * const p_api_ctrl, mxic_vee_cfg_t const * const p_cfg);
fsp_err_t RM_VEE_Init(mxic_vee_ctrl_t * const p_api_ctrl);
void RM_VEE_Close(mxic_vee_ctrl_t * const p_api_ctrl);

/* rww.c */
fsp_err_t mx_ee_rww_read(mxic_vee_ctrl_t * const p_api_ctrl, uint32_t addr, uint32_t len, void *buf);
fsp_err_t mx_ee_rww_write(mxic_vee_ctrl_t * const p_api_ctrl, uint32_t addr, uint32_t len, void *buf);
fsp_err_t mx_ee_rww_erase(mxic_vee_ctrl_t * const p_api_ctrl, uint32_t addr, uint32_t len);
fsp_err_t mx_ee_rww_init(mxic_vee_ctrl_t * const p_api_ctrl);
void mx_ee_rww_deinit(mxic_vee_ctrl_t * const p_api_ctrl);

#endif
