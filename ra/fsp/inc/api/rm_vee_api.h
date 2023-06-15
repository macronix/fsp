#ifndef RM_VEE_API_H
#define RM_VEE_API_H

/*******************************************************************************************************************//**
 * @defgroup Virtual EEPROM Interface
 * @ingroup RENESAS_INTERFACES
 * @brief Interface for Virtual EEPROM access
 *
 * @section mxic_VEE_API_SUMMARY Summary
 * The Virtual EEPROM Port configures a fail-safe key value store designed for microcontrollers on top of a lower
 * level storage device.
 *
 * Implemented by:
 * @ref mxic_VEE_FLASH
 *
 * @{
 **********************************************************************************************************************/

/***********************************************************************************************************************
 * Includes
 **********************************************************************************************************************/

/* Register definitions, common services and error codes. */
#include "bsp_api.h"
#include "stdbool.h"
#include "stdio.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"


/* Common macro for FSP header files. There is also a corresponding FSP_FOOTER macro at the end of this file. */
FSP_HEADER

typedef SemaphoreHandle_t osMutexId;


typedef struct os_mutex_def  {
  uint32_t                   dummy;    ///< dummy value.
} osMutexDef_t;


/// Access a Mutex definition.
/// \param         name          name of the mutex object.
/// \note CAN BE CHANGED: The parameter to \b osMutex shall be consistent but the
///       macro body is implementation specific in every CMSIS-RTOS.
#define osMutex(name)  \
&os_mutex_def_##name

#define assert_param(expr) \
      ((expr) ? (void)0U : assert_failed((uint8_t *)__FILE__, __LINE__))


/**********************************************************************************************************************
 * Macro definitions
 **********************************************************************************************************************/
#define mxic_VEE_API_VERSION_MAJOR    (1U)
#define mxic_VEE_API_VERSION_MINOR    (0U)

/**********************************************************************************************************************
 * Typedef definitions
 **********************************************************************************************************************/

/** User configuration structure, used in open function */
typedef struct st_mxic_vee_cfg
{
    uint32_t   mx_eeprom_sectors_per_cluster;
    uint32_t   mx_eeprom_banks;
    uint32_t   mx_eeprom_clusters_per_bank;
    uint32_t   Hash_select;
    uint32_t   mx_eeprom_free_sectors;
    uint32_t   mx_eeprom_entry_size;
    void const * p_context;                               ///< Placeholder for user data.
    void const * p_extend;                                ///< Pointer to hardware dependent configuration
} mxic_vee_cfg_t;

typedef struct st_mxic_vee_status
{
    uint32_t       last_id;             ///< Last ID written
    uint32_t       space_available;     ///< Remaining space available in the segment
    uint32_t       segment_erase_count; ///< Current segment erase count
} mxic_vee_status_t;

/** Virtual EEPROM API control block.  Allocate an instance specific control block to pass into the VEE API calls.
 * @par Implemented as
 * - @ref mxic_vee_flash_instance_ctrl_t
 */
typedef void mxic_vee_ctrl_t;

/* EEPROM parameter */
struct eeprom_param
{
  uint32_t eeprom_page_size;
  uint32_t eeprom_bank_size;
  uint32_t eeprom_banks;
  uint32_t eeprom_total_size;
  uint32_t eeprom_hash_algorithm;
};


/** Virtual EEPROM interface API. */
typedef struct st_mxic_vee_api
{
    /** Initializes the driver’s internal structures and opens the Flash driver.
     * @par Implemented as
     * - @ref mxic_VEE_FLASH_Open
     *
     * @param[in]   p_ctrl              Pointer to control block. Must be declared by user. Elements set here.
     * @param[in]   p_cfg               Pointer to configuration structure. All elements of this structure must be set by user.
     */
    fsp_err_t (* open)(mxic_vee_ctrl_t * const p_ctrl, mxic_vee_cfg_t const * const p_cfg);

    fsp_err_t (* format)(mxic_vee_ctrl_t * const p_api_ctrl);

    fsp_err_t (* init)(mxic_vee_ctrl_t * const p_ctrl);

    void (* getParam) (struct eeprom_param * param);

    fsp_err_t (* read)(mxic_vee_ctrl_t * const p_api_ctrl, uint32_t addr, uint32_t len, uint8_t *buf);

    fsp_err_t (* write)(mxic_vee_ctrl_t * const p_api_ctrl, uint32_t addr, uint32_t len, uint8_t *buf);

    fsp_err_t (* writeback)(mxic_vee_ctrl_t * const p_api_ctrl);

    fsp_err_t (* syncWrite)(mxic_vee_ctrl_t * const p_api_ctrl, uint32_t addr, uint32_t len, uint8_t *buf);

    fsp_err_t (* flush)(mxic_vee_ctrl_t * const p_api_ctrl);

    void (* background)(mxic_vee_ctrl_t * const p_api_ctrl);

    void (* close)(mxic_vee_ctrl_t * const p_api_ctrl);
} mxic_vee_api_t;



/** This structure encompasses everything that is needed to use an instance of this interface. */
typedef struct st_mxic_vee_instance
{
    mxic_vee_ctrl_t      * p_ctrl;       ///< Pointer to the control structure for this instance
    mxic_vee_cfg_t const * p_cfg;        ///< Pointer to the configuration structure for this instance
    mxic_vee_api_t const * p_api;        ///< Pointer to the API structure for this instance
} mxic_vee_instance_t;

extern const mxic_vee_api_t g_mxic_vee_on_flash;

/* Common macro for FSP header files. There is also a corresponding FSP_HEADER macro at the top of this file. */
FSP_FOOTER

/*******************************************************************************************************************//**
 * @} (end defgroup mxic_VEE_API)
 **********************************************************************************************************************/

#endif
