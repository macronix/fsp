#include <rm_littlefs_mxic_ospi_flash.h>
#include "rm_littlefs_mxic_ospi_flash_cfg.h"

#define RM_LITTLEFS_FLASH_MINIMUM_BLOCK_SIZE (104)
#define RM_LITTLEFS_MXIC_OSPI_FLASH_SECTOR_SIZE (4096)

/** "RLFS" in ASCII, used to determine if channel is open. */
#define RM_LITTLEFS_MXIC_OSPI_FLASH_OPEN                  (0x524C4653ULL)

/** LittleFS API mapping for LittleFS Port interface */
const rm_littlefs_api_t g_rm_littlefs_on_flash =
{
    .open       = RM_LITTLEFS_FLASH_Open,
    .close      = RM_LITTLEFS_FLASH_Close,
};

static fsp_err_t get_ospi_flash_status(void* p_ctrl)
{
    spi_flash_status_t status = {.write_in_progress = true};
    int32_t time_out          = (INT32_MAX);
    fsp_err_t err             = FSP_SUCCESS;

    do
    {
        /* Get status from OSPI flash device */
        err = R_OSPI_StatusGet((spi_flash_ctrl_t*)p_ctrl, &status);
        if (FSP_SUCCESS!= err)
        {
            return err;
        }

        /* Decrement time out to avoid infinite loop in case of consistent failure */
        --time_out;

        if ( 0 >= time_out)
        {
            return FSP_ERR_TIMEOUT;
        }

    }while (false != status.write_in_progress);

    return err;
}

/*******************************************************************************************************************//**
 * @addtogroup RM_LITTLEFS_FLASH
 * @{
 **********************************************************************************************************************/

/*******************************************************************************************************************//**
 * Opens the driver and initializes lower layer driver.
 *
 * Implements @ref rm_littlefs_api_t::open().
 *
 * @retval     FSP_SUCCESS                Success.
 * @retval     FSP_ERR_ASSERTION          An input parameter was invalid.
 * @retval     FSP_ERR_ALREADY_OPEN       Module is already open.
 * @retval     FSP_ERR_INVALID_SIZE       The provided block size is invalid.
 * @retval     FSP_ERR_INVALID_ARGUMENT   Flash BGO mode must be disabled.
 *
 * @return     See @ref RENESAS_ERROR_CODES or functions called by this function for other possible return codes. This
 *             function calls:
 *             * @ref flash_api_t::open
 **********************************************************************************************************************/
fsp_err_t RM_LITTLEFS_FLASH_Open (rm_littlefs_ctrl_t * const p_ctrl, rm_littlefs_cfg_t const * const p_cfg)
{
    rm_littlefs_flash_instance_ctrl_t * p_instance_ctrl = (rm_littlefs_flash_instance_ctrl_t *) p_ctrl;

#if RM_LITTLEFS_MXIC_OSPI_FLASH_CFG_PARAM_CHECKING_ENABLE
    FSP_ASSERT(NULL != p_ctrl);
    FSP_ASSERT(NULL != p_cfg);
    FSP_ASSERT(NULL != p_cfg->p_lfs_cfg);
    FSP_ASSERT(NULL != p_cfg->p_extend);

    rm_littlefs_flash_cfg_t const * p_extend = (rm_littlefs_flash_cfg_t *) p_cfg->p_extend;
    FSP_ASSERT(NULL != p_extend->p_ospi);
    FSP_ERROR_RETURN(RM_LITTLEFS_MXIC_OSPI_FLASH_OPEN != p_instance_ctrl->open, FSP_ERR_ALREADY_OPEN);
    FSP_ERROR_RETURN(p_cfg->p_lfs_cfg->block_size >= RM_LITTLEFS_FLASH_MINIMUM_BLOCK_SIZE, FSP_ERR_INVALID_SIZE);
    FSP_ERROR_RETURN((p_cfg->p_lfs_cfg->block_size % RM_LITTLEFS_MXIC_OSPI_FLASH_SECTOR_SIZE) == 0, FSP_ERR_INVALID_SIZE);

#else
    rm_littlefs_flash_cfg_t const * p_extend = (rm_littlefs_flash_cfg_t *) p_cfg->p_extend;
#endif

    p_instance_ctrl->p_cfg = p_cfg;

    /* This module is now open. */
    p_instance_ctrl->open = RM_LITTLEFS_MXIC_OSPI_FLASH_OPEN;

    return FSP_SUCCESS;
}

/*******************************************************************************************************************//**
 * Closes the lower level driver.
 *
 * Implements @ref rm_littlefs_api_t::close().
 *
 * @retval FSP_SUCCESS           Media device closed.
 * @retval FSP_ERR_ASSERTION     An input parameter was invalid.
 * @retval FSP_ERR_NOT_OPEN      Module not open.
 *
 * @return See @ref RENESAS_ERROR_CODES or functions called by this function for other possible return codes.
 *         This function calls:
 *             * @ref flash_api_t::close
 **********************************************************************************************************************/
fsp_err_t RM_LITTLEFS_FLASH_Close (rm_littlefs_ctrl_t * const p_ctrl)
{
    rm_littlefs_flash_instance_ctrl_t * p_instance_ctrl = (rm_littlefs_flash_instance_ctrl_t *) p_ctrl;
#if RM_LITTLEFS_MXIC_OSPI_FLASH_CFG_PARAM_CHECKING_ENABLE
    FSP_ASSERT(NULL != p_instance_ctrl);
    FSP_ERROR_RETURN(RM_LITTLEFS_MXIC_OSPI_FLASH_OPEN == p_instance_ctrl->open, FSP_ERR_NOT_OPEN);
#endif

    p_instance_ctrl->open = 0;

    return FSP_SUCCESS;
}

/*******************************************************************************************************************//**
 * @} (end addtogroup RM_LITTLEFS_FLASH)
 **********************************************************************************************************************/

/*******************************************************************************************************************//**
 * Read from the flash driver. Negative error codes are propogated to the user.
 *
 * @param[in]  c           Pointer to the LittleFS config block.
 * @param[in]  block       The block number
 * @param[in]  off         Offset in bytes
 * @param[out] buffer      The buffer to copy data into
 * @param[in]  size        The size in bytes
 *
 * @retval     LFS_ERR_OK  Read is complete.
 * @retval     LFS_ERR_IO  Lower level driver is not open.
 **********************************************************************************************************************/
int rm_littlefs_flash_read (const struct lfs_config * c,
                            lfs_block_t               block,
                            lfs_off_t                 off,
                            void                    * buffer,
                            lfs_size_t                size)
{
    rm_littlefs_flash_instance_ctrl_t * p_instance_ctrl = (rm_littlefs_flash_instance_ctrl_t *) c->context;
#if RM_LITTLEFS_MXIC_OSPI_FLASH_CFG_PARAM_CHECKING_ENABLE
    FSP_ERROR_RETURN(RM_LITTLEFS_MXIC_OSPI_FLASH_OPEN == p_instance_ctrl->open, LFS_ERR_IO);
#endif

    /* Read directly from the OSPI flash. */
    memcpy((uint8_t * )buffer, (uint8_t * )(BSP_OSPI_DEVICE_ADDRESS + (p_instance_ctrl->p_cfg->p_lfs_cfg->block_size * block) + off), size);

    return LFS_ERR_OK;
}

/*******************************************************************************************************************//**
 * Writes requested bytes to flash.
 *
 * @param[in]  c           Pointer to the LittleFS config block.
 * @param[in]  block       The block number
 * @param[in]  off         Offset in bytes
 * @param[in]  buffer      The buffer containing data to be written.
 * @param[in]  size        The size in bytes
 *
 * @retval     LFS_ERR_OK  Success.
 * @retval     LFS_ERR_IO  Lower layer is not open or failed to write the flash.
 **********************************************************************************************************************/
int rm_littlefs_flash_write (const struct lfs_config * c,
                             lfs_block_t               block,
                             lfs_off_t                 off,
                             const void              * buffer,
                             lfs_size_t                size)
{
    rm_littlefs_flash_instance_ctrl_t * p_instance_ctrl = (rm_littlefs_flash_instance_ctrl_t *) c->context;
#if RM_LITTLEFS_MXIC_OSPI_FLASH_CFG_PARAM_CHECKING_ENABLE
    FSP_ERROR_RETURN(RM_LITTLEFS_MXIC_OSPI_FLASH_OPEN == p_instance_ctrl->open, LFS_ERR_IO);
#endif

    rm_littlefs_flash_cfg_t const * p_extend = (rm_littlefs_flash_cfg_t *) p_instance_ctrl->p_cfg->p_extend;
    spi_flash_instance_t const        * p_ospi  = p_extend->p_ospi;

    /* Call the underlying driver. */
    fsp_err_t err =
            p_ospi->p_api->write(p_ospi->p_ctrl,
                              (uint32_t) buffer,
                              (BSP_OSPI_DEVICE_ADDRESS +
                               (p_instance_ctrl->p_cfg->p_lfs_cfg->block_size * block) + off),
                              size);

    /* Write failed. Return IO error. Negative error codes are propogated to the user. */
    FSP_ERROR_RETURN(FSP_SUCCESS == err, LFS_ERR_IO);
    get_ospi_flash_status(p_ospi->p_ctrl);


    return LFS_ERR_OK;
}

/*******************************************************************************************************************//**
 * Erase the logical block. The location and number of blocks to be erased will depend on block size.
 *
 * @param[in]  c           Pointer to the LittleFS config block.
 * @param[in]  block       The logical block number
 *
 * @retval     LFS_ERR_OK  Success.
 * @retval     LFS_ERR_IO  Lower layer is not open or failed to erase the flash.
 **********************************************************************************************************************/
int rm_littlefs_flash_erase (const struct lfs_config * c, lfs_block_t block)
{
    rm_littlefs_flash_instance_ctrl_t * p_instance_ctrl = (rm_littlefs_flash_instance_ctrl_t *) c->context;
#if RM_LITTLEFS_MXIC_OSPI_FLASH_CFG_PARAM_CHECKING_ENABLE
    FSP_ERROR_RETURN(RM_LITTLEFS_MXIC_OSPI_FLASH_OPEN == p_instance_ctrl->open, LFS_ERR_IO);
#endif
    rm_littlefs_flash_cfg_t const * p_extend = (rm_littlefs_flash_cfg_t *) p_instance_ctrl->p_cfg->p_extend;
    spi_flash_instance_t const        * p_ospi  = p_extend->p_ospi;

    /* Call the underlying driver. */
    fsp_err_t err =
            p_ospi->p_api->erase(p_ospi->p_ctrl,
                              (BSP_OSPI_DEVICE_ADDRESS + (p_instance_ctrl->p_cfg->p_lfs_cfg->block_size * block)),
                              p_instance_ctrl->p_cfg->p_lfs_cfg->block_size);

    /* Erase failed. Return IO error. Negative error codes are propogated to the user. */
    FSP_ERROR_RETURN(FSP_SUCCESS == err, LFS_ERR_IO);
    get_ospi_flash_status(p_ospi->p_ctrl);

    return LFS_ERR_OK;
}

/*******************************************************************************************************************//**
 * Stub function required by LittleFS. All calls immedialy write/erase the lower layer.
 * @param[in]  c           Pointer to the LittleFS config block.
 * @retval     LFS_ERR_OK  Success.
 **********************************************************************************************************************/
int rm_littlefs_flash_sync (const struct lfs_config * c)
{
    FSP_PARAMETER_NOT_USED(c);

    return LFS_ERR_OK;
}



