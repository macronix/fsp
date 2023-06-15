/**
  * Macronix Read-While-Write EEPROM Emulator.
  *
  * Copyright (c) 2021, Macronix International Co, Ltd.
  * This code is confidential, please DO NOT redistribute or publish it.
  *
  * Attention: This module is still in development, and WITHOUT ANY WARRANTY.
  * Please only use it as a reference.
  */

#include <rm_vee.h>

/* RWW structure */
static struct rww_info mx_rww =
{
  .initialized = false,
};

#ifdef MX_EEPROM_ECC_CHECK
/* OSPI Error codes */
#define OSPI_NOR_OK            ((uint8_t)0x00)
#define OSPI_NOR_ERROR         ((uint8_t)0x01)
#define OSPI_NOR_BUSY          ((uint8_t)0x02)
#define OSPI_NOR_NOT_SUPPORTED ((uint8_t)0x04)
#define OSPI_NOR_SUSPENDED     ((uint8_t)0x08)

#define OCTAL_OSPI_READ_CFG_REG_2_CMD                  0x718E
#define OCTAL_SPI_READ_CFG_REG_2_CMD                   0x71
#define MX25LM51245G_CR2_REG6_ADDR                ((uint32_t)0x00000800) /*!< CR2 register address 0x00000800 */
#define MX25LM51245G_CR2_ECCCNT_MASK              ((uint32_t)0x0000000f) /*!< ECC failure chunk counter */
#define MX25LM51245G_CR2_ECCFS_MASK               ((uint32_t)0x00000070) /*!< ECC fail status */
#define MX25LM51245G_CR2_ECCFS_ONE                ((uint32_t)0x00000010) /*!< 1 bit corrected */
#define MX25LM51245G_CR2_ECCFS_TWO                ((uint32_t)0x00000020) /*!< 2 bits deteced */
#define MX25LM51245G_CR2_ECCFS_ERR                ((uint32_t)0x00000040) /*!< Double programmed page detected */


static int BSP_OSPI_NOR_Check_ECC(ospi_instance_ctrl_t  * p_instance_ctrl, uint32_t eccStatus[2])
{

    spi_flash_protocol_t loc_spi_protocol = p_instance_ctrl->spi_protocol;
    spi_flash_direct_transfer_t ospi_read_cr2 = {0};
    if(loc_spi_protocol == SPI_FLASH_PROTOCOL_EXTENDED_SPI)
    {

         ospi_read_cr2.command        = OCTAL_SPI_READ_CFG_REG_2_CMD;
         ospi_read_cr2.address        = MX25LM51245G_CR2_REG6_ADDR;
         ospi_read_cr2.data           = 0;
         ospi_read_cr2.command_length = 1;
         ospi_read_cr2.address_length = 4;
         ospi_read_cr2.data_length    = 2;
         ospi_read_cr2.dummy_cycles   = 0;

    }
    else
    {
         ospi_read_cr2.command        = OCTAL_OSPI_READ_CFG_REG_2_CMD;
         ospi_read_cr2.address        = MX25LM51245G_CR2_REG6_ADDR;
         ospi_read_cr2.data           = 0;
         ospi_read_cr2.command_length = 2;
         ospi_read_cr2.address_length = 4;
         ospi_read_cr2.data_length    = 2;
         ospi_read_cr2.dummy_cycles   = 4;

    }
    /* direct transfer read command and read device id*/
    int err = R_OSPI_DirectTransfer(p_instance_ctrl, &ospi_read_cr2, SPI_FLASH_DIRECT_TRANSFER_DIR_READ);
    if(FSP_SUCCESS != err)
    {
        mx_log("\n R_OSPI_DirectTransfer API failed \r\n");
       return err;
    }

    eccStatus[0] = ospi_read_cr2.data & 0xff;
    eccStatus[1] = (ospi_read_cr2.data >> 8) & 0xff;

    return OSPI_NOR_OK;
}


/**
  * @brief  Check NOR flash on-die ECC status.
  * @retval Status
  */
static fsp_err_t mx_ee_check_ecc(mxic_vee_ctrl_t * const p_api_ctrl)
{
  mxic_vee_instance_ctrl_t * const p_ctrl = (mxic_vee_instance_ctrl_t *) p_api_ctrl;
  mxic_vee_ext_cfg_t * const p_ospi_extend = (mxic_vee_ext_cfg_t *)p_ctrl->p_cfg->p_extend;
  spi_flash_instance_t * const p_ospi = (spi_flash_instance_t *)p_ospi_extend->p_controller;
  uint32_t status[2] = {0xff, 0xff};
  /* Read ECC status */
  if (BSP_OSPI_NOR_Check_ECC((ospi_instance_ctrl_t *)(p_ospi->p_ctrl), status))
  {
    mx_err("mxee_ckecc: fail to read ecc status\r\n");
    mx_err("mxee_ckecc: QSPI HC state 0x%08lx, error 0x%08lx\r\n",
            status[0], status[1]);

    return FSP_ERR_ABORTED;
  }

  /* No ECC failure */
  if (status[0] == 0 || status[0] == MX25LM51245G_CR2_ECCFS_ONE)
    return FSP_SUCCESS;

  /* Detected double program */
  if (status[0] & MX25LM51245G_CR2_ECCFS_ERR)
    mx_err("mxee_ckecc: detected double program\r\n");

  /* Detected 2 bit-flips */
  if (status[0] & MX25LM51245G_CR2_ECCFS_TWO)
    mx_err("mxee_ckecc: detected 2 bit-flips\r\n");

  /* Corrected 1 bit-flip */
  if (status[0] & MX25LM51245G_CR2_ECCFS_ONE)
    mx_err("mxee_ckecc: corrected 1 bit-flip\r\n");

  mx_err("mxee_ckecc: ECC failure cnt %d\r\n",
          status[0] & MX25LM51245G_CR2_ECCCNT_MASK);

  return FSP_ERR_ABORTED;
}
#endif

/**
  * @brief  Read NOR flash.
  * @param  addr: Read start address
  * @param  len: Read length
  * @param  buf: Data buffer
  * @retval Status
  */
fsp_err_t mx_ee_rww_read(mxic_vee_ctrl_t * const p_api_ctrl, uint32_t addr, uint32_t len, void *buf)
{
  mxic_vee_instance_ctrl_t * const p_ctrl = (mxic_vee_instance_ctrl_t *) p_api_ctrl;
  int ret = 0;

#ifdef MX_FLASH_SUPPORT_RWW
  bool wait;
#endif

  mx_log("mxee_rwwrd: read  addr 0x%08lx, len %lu\r\n", addr, len);

#ifdef MX_FLASH_SUPPORT_RWW
  wait = mx_rww.waitTicks &&
        (mx_rww.busyBank == (addr & MX_FLASH_BANK_SIZE_MASK));
  if (wait)
  {
    /* Get the busy lock */
    if (xSemaphoreTake(mx_rww.busyLock, portMAX_DELAY) != pdTRUE)
         return FSP_ERR_ABORTED;

    /* Wait until busy timeout */
    if (!osTaskCheckForTimeOut(&mx_rww.timeout, &mx_rww.waitTicks))
    {
      osTaskDelay(mx_rww.waitTicks);
    }
  }
#endif

  /* Get the device lock */
  if (xSemaphoreTake(mx_rww.deviceLock, portMAX_DELAY) != pdTRUE)
      return FSP_ERR_ABORTED;
  /* Do the real read here */

#ifdef SPINAND
  ret = BSP_SPI_Read(&(p_ctrl->Mxic), addr, len, buf);
#else
#ifdef QSPI_CONTROL
  memcpy(buf, (unsigned char *)(addr + QSPI_DEVICE_START_ADDRESS), len);
#endif
#ifdef OSPI_CONTROL
   memcpy(buf, (unsigned char *)(addr + BSP_FEATURE_OSPI_DEVICE_0_START_ADDRESS), len);
#endif
#endif

#ifdef MX_EEPROM_ECC_CHECK
  /* Check ECC status after each read */
  ret |= mx_ee_check_ecc(p_ctrl);
#endif

  xSemaphoreGive(mx_rww.deviceLock);

#ifdef MX_FLASH_SUPPORT_RWW
  if (wait)
    xSemaphoreGive(mx_rww.busyLock);
#endif

  assert_param(!ret);  /* For testing */
  return (!ret ? FSP_SUCCESS : FSP_ERR_INVALID_STATE);
}

/**
  * @brief  Write NOR flash.
  * @param  addr: Write start address
  * @param  len: Write length
  * @param  buf: Data buffer
  * @retval Status
  */
fsp_err_t mx_ee_rww_write(mxic_vee_ctrl_t * const p_api_ctrl, uint32_t addr, uint32_t len, void *buf)
{
  mxic_vee_instance_ctrl_t * const p_ctrl = (mxic_vee_instance_ctrl_t *) p_api_ctrl;

#if defined(QSPI_CONTROL) || defined(OSPI_CONTROL)
  mxic_vee_ext_cfg_t const * p_extend = p_ctrl->p_cfg->p_extend;
  spi_flash_instance_t const * p_controller = p_extend->p_controller;
#endif

  int ret;

  mx_log("mxee_rwwwr: write addr 0x%08lx, len %lu\r\n", addr, len);

#ifdef MX_FLASH_SUPPORT_RWW
  /* Get the busy lock */
  if (xSemaphoreTake(mx_rww.busyLock, portMAX_DELAY) != pdTRUE)
    return FSP_ERR_ABORTED;
  /* Wait until busy timeout */
  ret = osTaskCheckForTimeOut(&mx_rww.timeout, &mx_rww.waitTicks);
  osTaskDelay(ret ? 0 : mx_rww.waitTicks);
#endif

  /* Get the device lock */
  if (xSemaphoreTake(mx_rww.deviceLock, portMAX_DELAY) != pdTRUE)
     return FSP_ERR_INVALID_STATE;
  /* Do the real write here */
#ifdef SPINAND
  ret = BSP_SPI_Write(&(p_ctrl->Mxic), addr, len, buf);
#else
#ifdef QSPI_CONTROL
  R_QSPI_Write(p_controller->p_ctrl, buf, (uint8_t *)(addr + QSPI_DEVICE_START_ADDRESS), len);
#endif
#ifdef OSPI_CONTROL
  R_OSPI_Write(p_controller->p_ctrl, buf, (uint8_t *)(addr + BSP_FEATURE_OSPI_DEVICE_0_START_ADDRESS), len);
#endif
#endif
  get_flash_status(p_ctrl);
#ifdef MX_FLASH_SUPPORT_RWW
  /* Set the busy-waiting time */
  mx_rww.busyBank = addr & MX_FLASH_BANK_SIZE_MASK;
  mx_rww.waitTicks = MX_FLASH_PAGE_WRITE_TICKS;
  osTaskSetTimeOutState(&mx_rww.timeout);
#endif

  xSemaphoreGive(mx_rww.deviceLock);

#ifdef MX_FLASH_SUPPORT_RWW
  xSemaphoreGive(mx_rww.busyLock);
#endif

  assert_param(!ret);

  return (!ret ? FSP_SUCCESS : FSP_ERR_INVALID_STATE);
}

/**
  * @brief  Wrapper function of sector/block/chip erase.
  * @param  addr: Erase start address
  * @param  len: Erase length
  * @retval Status
  */
fsp_err_t mx_erase(mxic_vee_ctrl_t * const p_api_ctrl, uint32_t addr, uint32_t len);
fsp_err_t mx_erase(mxic_vee_ctrl_t * const p_api_ctrl, uint32_t addr, uint32_t len)
{
  mxic_vee_instance_ctrl_t * const p_ctrl = (mxic_vee_instance_ctrl_t *) p_api_ctrl;

#if defined(QSPI_CONTROL) || defined(OSPI_CONTROL)
  mxic_vee_ext_cfg_t const * p_extend = p_ctrl->p_cfg->p_extend;
  spi_flash_instance_t const * p_controller = p_extend->p_controller;
#endif

  int ret = FSP_SUCCESS;

      /* Sector erase */
#ifdef SPINAND
        ret = BSP_SPI_Erase_Block(&(p_ctrl->Mxic), addr, 1);
#else
#ifdef QSPI_CONTROL
        ret = R_QSPI_Erase(p_controller->p_ctrl, (unsigned char *)(addr + QSPI_DEVICE_START_ADDRESS), len);
#endif
#ifdef OSPI_CONTROL
        ret = R_OSPI_Erase(p_controller->p_ctrl, (unsigned char *)(addr + BSP_FEATURE_OSPI_DEVICE_0_START_ADDRESS), len);
#endif
#endif
        get_flash_status(p_ctrl);
#ifdef MX_FLASH_SUPPORT_RWW
      /* Set the busy-waiting time */
      mx_rww.busyBank = addr & MX_FLASH_BANK_SIZE_MASK;
      mx_rww.waitTicks = MX_FLASH_SECTOR_ERASE_TICKS;
      osTaskSetTimeOutState(&mx_rww.timeout);
#endif

  return ret;
}

/**
  * @brief  Erase NOR flash.
  * @param  addr: Erase start address
  * @param  len: Erase length
  * @retval Status
  */
fsp_err_t mx_ee_rww_erase(mxic_vee_ctrl_t * const p_api_ctrl, uint32_t addr, uint32_t len)
{
  int ret;
  mx_log("mxee_rwwer: erase addr 0x%08lx, len %lu\r\n", addr, len);

#ifdef MX_FLASH_SUPPORT_RWW
  /* Get the busy lock */
  if (xSemaphoreTake(mx_rww.busyLock, portMAX_DELAY) != pdTRUE)
      return FSP_ERR_ABORTED;
  /* Wait until busy timeout */
  ret = osTaskCheckForTimeOut(&mx_rww.timeout, &mx_rww.waitTicks);
  osTaskDelay(ret ? 0 : mx_rww.waitTicks);
#endif

  /* Get the device lock */
  if (xSemaphoreTake(mx_rww.deviceLock, portMAX_DELAY) != pdTRUE)
    return FSP_ERR_INVALID_STATE;
  /* Do the real erase here */
  ret = mx_erase(p_api_ctrl, addr, len);

  xSemaphoreGive(mx_rww.deviceLock);

#ifdef MX_FLASH_SUPPORT_RWW
  xSemaphoreGive(mx_rww.busyLock);
#endif

  assert_param(!ret);  /*For testing */

  return (!ret ? FSP_SUCCESS : FSP_ERR_INVALID_STATE);
}

/**
  * @brief  Initialize RWW layer.
  * @retval Status
  */
fsp_err_t mx_ee_rww_init(mxic_vee_ctrl_t * const p_api_ctrl)
{
  int ret;
  if (mx_rww.initialized)
    return FSP_SUCCESS;

#ifdef MX_FLASH_SUPPORT_RWW
  /* Init the busy-waiting time */
  mx_rww.busyBank = 0xffffffff;
  mx_rww.waitTicks = 0;
  osTaskSetTimeOutState(&mx_rww.timeout);

  /* Init RWW lock */
  mx_rww.busyLock = xSemaphoreCreateMutex();
  if (!mx_rww.busyLock)
  {
    mx_err("mxee_rwwin: out of memory (busyLock)\r\n");
    ret = FSP_ERR_OUT_OF_MEMORY;
    goto err0;
  }
#endif

  /* Init device lock */
  mx_rww.deviceLock = xSemaphoreCreateMutex();
  if (!mx_rww.deviceLock)
  {
    mx_err("mxee_rwwin: out of memory (deviceLock)\r\n");
    ret = FSP_ERR_OUT_OF_MEMORY;
    goto err1;
  }

  /* Init done */
  mx_rww.initialized = true;

  return FSP_SUCCESS;

err1:
#ifdef MX_FLASH_SUPPORT_RWW
  vQueueDelete(mx_rww.busyLock);
  mx_rww.busyLock = NULL;
err0:
#endif
  return ret;
}

/**
  * @brief  Deinit RWW layer.
  */
void mx_ee_rww_deinit(mxic_vee_ctrl_t * const p_api_ctrl)
{
  if (!mx_rww.initialized)
    return;

  /* Block further user request */
  mx_rww.initialized = false;

#ifdef MX_FLASH_SUPPORT_RWW
  /* Delete RWW lock */
  xSemaphoreTake(mx_rww.busyLock, portMAX_DELAY);
  vQueueDelete(mx_rww.busyLock);
  mx_rww.busyLock = NULL;
#endif

  /* Wait for the current request to finish */
  xSemaphoreTake(mx_rww.deviceLock, portMAX_DELAY);
  get_flash_status(p_api_ctrl);

  /* Delete device lock */
  vQueueDelete(mx_rww.deviceLock);
  mx_rww.deviceLock = NULL;
}

void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

    SEGGER_RTT_printf(0,"wrong parameters value: file %s on line %lu\r\n", file, line);

  /* Infinite loop */
  while(1);
}

fsp_err_t get_flash_status(mxic_vee_ctrl_t * const p_api_ctrl)
{
    mxic_vee_instance_ctrl_t * const p_ctrl = (mxic_vee_instance_ctrl_t *) p_api_ctrl;
#if defined(QSPI_CONTROL) || defined(OSPI_CONTROL)
    mxic_vee_ext_cfg_t const * p_extend = p_ctrl->p_cfg->p_extend;
    spi_flash_instance_t const * p_controller = p_extend->p_controller;
#endif
#ifdef SPINAND
    return MxWaitForFlashReady(&(p_ctrl->Mxic), NAND_FLASH_TIMEOUT_VALUE);
#else
    (void)p_api_ctrl;
    spi_flash_status_t status = {.write_in_progress = true};
    int32_t time_out          = (INT32_MAX);
    fsp_err_t err             = FSP_SUCCESS;

    do
    {
        // Get status from QSPI flash device
#ifdef OSPI_CONTROL

        err = R_OSPI_StatusGet(p_controller->p_ctrl, &status);
#endif
#ifdef QSPI_CONTROL

        err = R_QSPI_StatusGet(p_controller->p_ctrl, &status);
#endif
        if (FSP_SUCCESS!= err)
        {
            SEGGER_RTT_printf(0, "StatusGet Failed\r\n");
            return err;
        }

        // Decrement time out to avoid infinite loop in case of consistent failure
        --time_out;

        if ( 0 >= time_out)
        {
            SEGGER_RTT_printf(0, "\r\n ** Timeout : No result from QSPI flash status register ** \r\n");
            return FSP_ERR_TIMEOUT;
        }

    }while (false != status.write_in_progress);

    return err;
#endif
}
