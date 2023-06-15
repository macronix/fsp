#ifndef RM_LITTLEFS_MXIC_OSPI_FLASH_H
#define RM_LITTLEFS_MXIC_OSPI_FLASH_H

/***********************************************************************************************************************
 * Includes
 **********************************************************************************************************************/
#include "rm_littlefs_api.h"
#include "r_ospi.h"
#include "lfs.h"

/* Common macro for FSP header files. There is also a corresponding FSP_FOOTER macro at the end of this file. */
FSP_HEADER

/*******************************************************************************************************************//**
 * @addtogroup RM_LITTLEFS_FLASH
 **********************************************************************************************************************/

/***********************************************************************************************************************
 * Macro definitions
 **********************************************************************************************************************/

/***********************************************************************************************************************
 * Typedef definitions
 **********************************************************************************************************************/

/** User configuration structure, used in open function */
typedef struct st_rm_littlefs_flash_cfg
{
    spi_flash_instance_t const * p_ospi;  ///< Pointer to a ospi instance
} rm_littlefs_flash_cfg_t;

/** Instance control block.  This is private to the FSP and should not be used or modified by the application. */
typedef struct st_rm_littlefs_flash_instance_ctrl
{
    uint32_t open;
    rm_littlefs_cfg_t const * p_cfg;
} rm_littlefs_flash_instance_ctrl_t;

/**********************************************************************************************************************
 * Exported global variables
 **********************************************************************************************************************/

/** @cond INC_HEADER_DEFS_SEC */
/** Filled in Interface API structure for this Instance. */
extern const rm_littlefs_api_t g_rm_littlefs_on_flash;

/** @endcond */

/**********************************************************************************************************************
 * Function Prototypes
 **********************************************************************************************************************/
fsp_err_t RM_LITTLEFS_FLASH_Open(rm_littlefs_ctrl_t * const p_ctrl, rm_littlefs_cfg_t const * const p_cfg);

fsp_err_t RM_LITTLEFS_FLASH_Close(rm_littlefs_ctrl_t * const p_ctrl);

int rm_littlefs_flash_read(const struct lfs_config * c, lfs_block_t block, lfs_off_t off, void * buffer,
                           lfs_size_t size);

int rm_littlefs_flash_write(const struct lfs_config * c,
                            lfs_block_t               block,
                            lfs_off_t                 off,
                            const void              * buffer,
                            lfs_size_t                size);

int rm_littlefs_flash_erase(const struct lfs_config * c, lfs_block_t block);

int rm_littlefs_flash_sync(const struct lfs_config * c);

/* Common macro for FSP header files. There is also a corresponding FSP_HEADER macro at the top of this file. */
FSP_FOOTER


#endif /* RM_LITTLEFS_FLASH_H */
