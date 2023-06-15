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
#include "string.h"
#include "stdlib.h"

/* "vEE" in ASCII, used to determine if channel is open. */
#define vEE_OPEN                   (0x00764545ULL)

const mxic_vee_api_t g_mxic_vee_on_flash =
{
    .open          = RM_VEE_Open,
    .format        = RM_VEE_Format,
    .init          = RM_VEE_Init,
    .getParam      = RM_VEE_GetParam,
    .read          = RM_VEE_Read,
    .write         = RM_VEE_Write,
    .writeback     = RM_VEE_WriteBack,
    .syncWrite     = RM_VEE_SyncWrite,
    .flush         = RM_VEE_Flush,
    .background    = RM_VEE_Background,
    .close         = RM_VEE_Close,
};

/* EEPROM physical address offset in each bank */
static uint32_t bank_offset[MX_EEPROMS] = MX_BANK_OFFSET;

#ifdef MX_EEPROM_CRC_HW
/* CRC handler */
crc_input_t crc_input = {.num_bytes = CRC16_DATA_LENGTH,
                         .crc_seed  = 0,};
#endif

#ifdef MX_EEPROM_PC_PROTECTION
#ifndef SPINAND
/**
  * @brief  Update system entry of current block of current bank.
  * @param  bi: Current bank handle
  * @param  entry: System entry address
  * @param  sys: System entry buffer
  * @retval Status
  */
static fsp_err_t mx_ee_read_sys(mxic_vee_ctrl_t * const p_api_ctrl, struct bank_info *bi, uint32_t entry,
                          struct system_entry *sys)
{
  mxic_vee_instance_ctrl_t * const p_ctrl = (mxic_vee_instance_ctrl_t *) p_api_ctrl;
  int ret;
  uint32_t addr;
  /* Check address validity */
  if ((bi->bank >= MX_EEPROMS) ||
      (bi->block >= MX_EEPROM_BLOCKS) ||
      (entry >= MX_EEPROM_SYSTEM_ENTRIES))
    return FSP_ERR_INVALID_ARGUMENT;

  addr = bi->block_offset + MX_EEPROM_SYSTEM_SECTOR_OFFSET +
         entry * MX_EEPROM_SYSTEM_ENTRY_SIZE;

  /* Read system entry */
  ret = mx_ee_rww_read(p_ctrl, addr, sizeof(*sys), sys);
  if (ret)
  {
    mx_err("mxee_rdsys: fail to read, bank %lu, block %lu, entry %lu\r\n",
            bi->bank, bi->block, entry);
    return ret;
  }

  /* Check system entry */
  if ((sys->id != MFTL_ID && sys->id != DATA_NONE16) ||
      (sys->cksum != (sys->id ^ sys->ops ^ sys->arg)))
  {
    mx_err("mxee_rdsys: corrupted entry, bank %lu, block %lu, entry %lu\r\n",
            bi->bank, bi->block, entry);
    return FSP_ERR_INVALID_ARGUMENT;
  }

  return FSP_SUCCESS;
}
#endif
#endif

#ifdef SPINAND
/**
  * @brief  Read system entry of current block of current bank.
  * @param  bi: Current bank handle
  * @param  sector:
  * @param  system_entry_addr:
  * @param  sys: System entry buffer
  * @retval Status
  */
static fsp_err_t mx_ee_read_sys(mxic_vee_ctrl_t * const p_api_ctrl, struct bank_info *bi, uint32_t sector, uint32_t system_entry_addr,
                          struct system_entry *sys)
{
    mxic_vee_instance_ctrl_t * const p_ctrl = (mxic_vee_instance_ctrl_t *) p_api_ctrl;
    int ret;
    uint32_t addr;

    /* Check address validity */
    if ((bi->bank >= MX_EEPROMS) ||
        (bi->block >= MX_EEPROM_BLOCKS) ||
        (sector >= MX_EEPROM_SECTORS_PER_CLUSTER))
        return FSP_ERR_INVALID_ARGUMENT;

    addr = bi->block_offset + sector * MX_EEPROM_SECTOR_OFFSET + system_entry_addr;

    /* Read system entry */
    ret = mx_ee_rww_read(p_ctrl, addr, sizeof(*sys), sys);
    if (ret) {
        mx_err("mxee_rdsys: fail to read, bank %lu, block %lu, sector %lu\r\n",
            bi->bank, bi->block, sector);
        return ret;
    }

    /* Check system entry */
    if ((sys->id != MFTL_ID && sys->id != DATA_NONE16) ||
      (sys->cksum != (sys->id ^ sys->ops ^ sys->arg))) {
        mx_err("mxee_rdsys: corrupted entry, bank %lu, block %lu, sector %lu\r\n",
            bi->bank, bi->block, sector);
        return FSP_ERR_INVALID_ARGUMENT;
    }

    return FSP_SUCCESS;
}
#endif

#ifdef MX_EEPROM_PC_PROTECTION
#ifndef SPINAND
/**
  * @brief  Update system entry of current block of current bank.
  * @param  bi: Current bank handle
  * @param  ops: Operation type
  * @param  arg: Operation argument
  * @retval Status
  */
static fsp_err_t mx_ee_update_sys(mxic_vee_ctrl_t * const p_api_ctrl, struct bank_info *bi, uint32_t block,
                            rwwee_ops ops, uint32_t arg)
{
  mxic_vee_instance_ctrl_t * const p_ctrl = (mxic_vee_instance_ctrl_t *) p_api_ctrl;
  int ret;
  uint32_t addr;
  struct system_entry sys;

  /* Check address validity */
  if ((bi->bank >= MX_EEPROMS) ||
      (block >= MX_EEPROM_BLOCKS) ||
      (bi->sys_entry[block] >= MX_EEPROM_SYSTEM_ENTRIES))
    return FSP_ERR_INVALID_ARGUMENT;

  addr = bi->bank_offset + block * MX_EEPROM_CLUSTER_SIZE +
         MX_EEPROM_SYSTEM_SECTOR_OFFSET;

  if (++bi->sys_entry[block] == MX_EEPROM_SYSTEM_ENTRIES)
  {
#ifdef MX_DEBUG
    /* Erase count statistics */
    bi->eraseCnt[block][MX_EEPROM_SYSTEM_SECTOR]++;
#endif

    /* Erase system sector */
    ret = mx_ee_rww_erase(p_ctrl, addr, MX_FLASH_SECTOR_SIZE);
    if (ret)
    {
      mx_err("mxee_wrsys: fail to erase, bank %lu, block %lu, sector %d\r\n",
              bi->bank, block, MX_EEPROM_SYSTEM_SECTOR);

      bi->sys_entry[block] = DATA_NONE32;
      return ret;
    }

    /* Round-robin method */
    bi->sys_entry[block] = 0;
  }

  /* Fill system entry */
  sys.id = MFTL_ID;
  sys.ops = ops;
  sys.arg = arg;
  sys.cksum = sys.id ^ sys.ops ^ sys.arg;

  addr += bi->sys_entry[block] * MX_EEPROM_SYSTEM_ENTRY_SIZE;

  /* Update system info */
  ret = mx_ee_rww_write(p_ctrl, addr, sizeof(sys), &sys);
  if (ret)
  {
    mx_err("mxee_wrsys: fail to update, bank %lu, block %lu, entry %lu\r\n",
            bi->bank, block, bi->sys_entry[block]);
    return ret;
  }

  return FSP_SUCCESS;
}
#endif
#endif

#ifdef SPINAND
/**
  * @brief  Update system entry of current block of current bank.
  * @param  bi: Current bank handle
  * @param  sector:
  * @param  system_entry_addr:
  * @param  ops: Operation type
  * @param  arg: Operation argument
  * @retval Status
  */
static fsp_err_t mx_ee_update_sys(mxic_vee_ctrl_t * const p_api_ctrl, struct bank_info *bi, uint32_t sector, uint32_t system_entry_addr,
        rwwee_ops ops, uint32_t arg)
{
    mxic_vee_instance_ctrl_t * const p_ctrl = (mxic_vee_instance_ctrl_t *) p_api_ctrl;
    int ret;
    uint32_t addr;
    struct system_entry sys;

    /* Check address validity */
    if ((bi->bank >= MX_EEPROMS) ||
        (bi->block >= MX_EEPROM_BLOCKS) ||
        (sector >= MX_EEPROM_SECTORS_PER_CLUSTER))
        return FSP_ERR_INVALID_ARGUMENT;

    addr = bi->block_offset + sector * MX_EEPROM_SECTOR_OFFSET + system_entry_addr;

    /* Fill system entry */
    sys.id = MFTL_ID;
    sys.ops = ops;
    sys.arg = arg;
    sys.cksum = sys.id ^ sys.ops ^ sys.arg;

    /* Update system info */
    ret = mx_ee_rww_write(p_ctrl, addr, sizeof(sys), &sys);
    if (ret) {
        mx_err("mxee_wrsys: fail to update, bank %lu, sector %lu, entry %lu\r\n",
            bi->bank, sector, bi->sys_entry[sector]);
        return ret;
    }

    return FSP_SUCCESS;
}
#endif

/**
  * @brief  Read the specified entry of current block of current bank.
  * @param  bi: Current bank handle
  * @param  entry: Local entry address
  * @param  buf: Data buffer
  * @param  header: Read entry header only (true) or the whole entry (false)
  * @retval Status
  */
static fsp_err_t mx_ee_read(mxic_vee_ctrl_t * const p_api_ctrl, struct bank_info *bi, uint32_t entry, void *buf,
                      bool header)
{
  mxic_vee_instance_ctrl_t * const p_ctrl = (mxic_vee_instance_ctrl_t *) p_api_ctrl;
  int ret;
  uint32_t addr, len, cksum;
  struct eeprom_entry *cache = buf;

  /* Check address validity */
  if ((bi->bank >= MX_EEPROMS) ||
      (bi->block >= MX_EEPROM_BLOCKS) ||
      (entry >= MX_EEPROM_ENTRIES_PER_CLUSTER))
    return FSP_ERR_INVALID_ARGUMENT;



#ifdef SPINAND
  addr = bi->block_offset + MX_EEPROM_SECTOR_OFFSET * (entry / MX_EEPROM_ENTRIES_PER_SECTOR)
             + ((entry % MX_EEPROM_ENTRIES_PER_SECTOR + 2) % 4) * MX_EEPROM_ENTRY_SIZE   /* +2 : the first and second entry is for system entry*/
             + ((entry % MX_EEPROM_ENTRIES_PER_SECTOR + 2) / 4) * MX_FLASH_PAGE_OFFSET;
#else
  addr = entry * MX_EEPROM_ENTRY_SIZE + bi->block_offset;
#endif

  len = header ? MX_EEPROM_HEADER_SIZE : MX_EEPROM_ENTRY_SIZE;

  /* Do the real read */
  ret = mx_ee_rww_read(p_ctrl, addr, len, buf);
  if (ret)
  {
    mx_err("mxee_rddat: fail to read %s, bank %lu, block %lu, entry %lu\r\n",
            header ? "header" : "entry", bi->bank, bi->block, entry);
    return ret;
  }

  /* Check entry address */
  cksum = cache->header.LPA + cache->header.LPA_inv;
  if ((cksum != DATA_NONE8) && (cksum != DATA_NONE8 + DATA_NONE8))
  {
    mx_err("mxee_rddat: corrupted entry LPA 0x%02x, inv 0x%02x, "
           "bank %lu, block %lu, entry %lu\r\n",
            cache->header.LPA, cache->header.LPA_inv,
            bi->bank, bi->block, entry);
    return FSP_ERR_INVALID_ARGUMENT;
  }

#ifdef MX_EEPROM_CRC_HW
  /* Check entry data */
  if (!header)
  {
    if (xSemaphoreTake(p_ctrl->crcLock, portMAX_DELAY) != pdTRUE)
         return FSP_ERR_ABORTED;
    mxic_vee_ext_cfg_t const * p_crc_extend = p_ctrl->p_cfg->p_extend;
    crc_instance_t const * p_crc = p_crc_extend->p_crc;
    /* Calculate data CRC */
    crc_input.p_input_buffer = (uint32_t *)cache->data;
    R_CRC_Calculate(p_crc->p_ctrl, &crc_input, &cksum);

    /* Add rwCnt inside the mutex lock */
    p_ctrl->rwCnt++;
    xSemaphoreGive(p_ctrl->crcLock);

    /* Check data CRC */
    if (cache->header.crc != (cksum & DATA_NONE16))
    {
#ifdef SPINAND
      mx_err("mxee_rddat: corrupted entry data, crc 0x%04x -> 0x%04x, "
                   "bank %lu, block %lu, entry %lu, addr 0x%02x\r\n",
                    cache->header.crc, (uint16_t)cksum,
                    bi->bank, bi->block, entry, addr);
      return FSP_ERR_INVALID_ARGUMENT;
#else
      mx_err("mxee_rddat: corrupted entry data, crc 0x%04x -> 0x%04x, "
             "bank %lu, block %lu, entry %lu\r\n",
              cache->header.crc, (uint16_t)cksum,
              bi->bank, bi->block, entry);
      return FSP_ERR_INVALID_ARGUMENT;
#endif
    }
  }
#endif

  return FSP_SUCCESS;
}

/**
  * @brief  Write the specified entry of current block of current bank.
  * @param  bi: Current bank handle
  * @param  entry: Local entry address
  * @param  buf: Data buffer
  * @retval Status
  */
static fsp_err_t mx_ee_write(mxic_vee_ctrl_t * const p_api_ctrl, struct bank_info *bi, uint32_t entry, void *buf)
{
  mxic_vee_instance_ctrl_t * const p_ctrl = (mxic_vee_instance_ctrl_t *) p_api_ctrl;
  int ret;
  uint32_t addr;
  struct eeprom_entry *cache = buf;

  /* Check address validity */
  if ((bi->bank >= MX_EEPROMS) ||
      (bi->block >= MX_EEPROM_BLOCKS) ||
      (entry >= MX_EEPROM_ENTRIES_PER_CLUSTER))
    return FSP_ERR_INVALID_ARGUMENT;

#ifdef SPINAND
  addr = bi->block_offset + MX_EEPROM_SECTOR_OFFSET * (entry / MX_EEPROM_ENTRIES_PER_SECTOR)
           + ((entry % MX_EEPROM_ENTRIES_PER_SECTOR + 2) % 4) * MX_EEPROM_ENTRY_SIZE
           + ((entry % MX_EEPROM_ENTRIES_PER_SECTOR + 2) / 4) * MX_FLASH_PAGE_OFFSET;
#else
  addr = entry * MX_EEPROM_ENTRY_SIZE + bi->block_offset;
#endif
  /* Calculate redundant LPA */
  cache->header.LPA_inv = ~cache->header.LPA;

#ifdef MX_EEPROM_CRC_HW
  if (xSemaphoreTake(p_ctrl->crcLock, portMAX_DELAY) != pdTRUE)
    return FSP_ERR_ABORTED;
  mxic_vee_ext_cfg_t const * p_crc_extend = p_ctrl->p_cfg->p_extend;
  crc_instance_t const * p_crc = p_crc_extend->p_crc;
  /* Calculate data CRC */
  uint32_t crc_value = 0;
  crc_input.p_input_buffer = (uint32_t *)cache->data;
  R_CRC_Calculate(p_crc->p_ctrl, &crc_input, &crc_value);
  cache->header.crc = (uint16_t)crc_value;

  /* Add rwCnt inside the mutex lock */
  p_ctrl->rwCnt++;

  xSemaphoreGive(p_ctrl->crcLock);
#endif

  /* Do the real write */
  ret = mx_ee_rww_write(p_ctrl, addr, MX_EEPROM_ENTRY_SIZE, cache);
  if (ret)
  {
    mx_err("mxee_wrdat: fail to write, bank %lu, block %lu, entry %lu\r\n",
            bi->bank, bi->block, entry);
  }

  return ret;
}

/**
  * @brief  Erase the obsoleted sector of current bank.
  * @param  bi: Current bank handle
  * @retval Status
  */
static fsp_err_t mx_ee_erase(mxic_vee_ctrl_t * const p_api_ctrl, struct bank_info *bi)
{
  mxic_vee_instance_ctrl_t * const p_ctrl = (mxic_vee_instance_ctrl_t *) p_api_ctrl;
  int ret;
  uint32_t addr;

  /* Check address validity */
  if (bi->bank >= MX_EEPROMS)
    return FSP_ERR_INVALID_ARGUMENT;
  if ((bi->dirty_block >= MX_EEPROM_BLOCKS) ||
      (bi->dirty_sector >= MX_EEPROM_DATA_SECTORS))
    return FSP_SUCCESS;
#ifdef SPINAND
  addr = bi->dirty_sector * MX_EEPROM_SECTOR_OFFSET +
         bi->dirty_block * MX_EEPROM_CLUSTER_OFFSET +
         bi->bank_offset;

#else
  addr = bi->dirty_sector * MX_FLASH_SECTOR_SIZE +
         bi->dirty_block * MX_EEPROM_CLUSTER_SIZE +
         bi->bank_offset;
#endif

#ifdef MX_DEBUG
  /* Erase count statistics */
  bi->eraseCnt[bi->dirty_block][bi->dirty_sector]++;
#endif

#ifdef MX_EEPROM_PC_PROTECTION
  /* Erase begin */
#ifdef SPINAND
  mx_ee_update_sys(p_ctrl, bi, bi->dirty_sector, SYS_ENTRY_ADDR_E_B, OPS_ERASE_BEGIN, DATA_NONE16);
#else
  mx_ee_update_sys(p_ctrl, bi, bi->dirty_block, OPS_ERASE_BEGIN, bi->dirty_sector);
#endif
#endif

  /* Erase obsoleted sector */
#ifdef SPINAND
  ret = mx_ee_rww_erase(p_ctrl, addr, 1);
#else
  ret = mx_ee_rww_erase(p_ctrl, addr, MX_FLASH_SECTOR_SIZE);
#endif
  if (ret)
  {
    mx_err("mxee_erase: fail to erase, bank %lu, block %lu, sector %lu\r\n",
            bi->bank, bi->dirty_block, bi->dirty_sector);
  }

#ifdef MX_EEPROM_PC_PROTECTION
  /* Erase end, XXX: will block RWE */
#ifdef SPINAND
  mx_ee_update_sys(p_ctrl, bi, bi->dirty_sector, SYS_ENTRY_ADDR_E_E, OPS_ERASE_END, DATA_NONE16);
#else
  mx_ee_update_sys(p_ctrl, bi, bi->dirty_block, OPS_ERASE_END, bi->dirty_sector);
#endif
#endif

  if (bi->dirty_block == bi->block)
  {
    /* Mark as free or bad sector */
    if (!ret)
#ifdef SPINAND
    {
      bi->p2l[bi->dirty_sector] = DATA_NONE8;
      bi->latest_used_entry_per_sector[bi->dirty_sector] = DATA_NONE8;
    }
#else
      bi->p2l[bi->dirty_sector] = DATA_NONE8;
#endif
    else
      bi->p2l[bi->dirty_sector] = MX_EEPROM_LPAS_PER_CLUSTER;
  }

  bi->dirty_block = DATA_NONE32;
  bi->dirty_sector = DATA_NONE32;

  return ret;
}

#ifdef SPINAND
static uint32_t mx_ee_build_l2p_in_a_sector(mxic_vee_ctrl_t * const p_api_ctrl, struct bank_info *bi, uint32_t sector)
{
    mxic_vee_instance_ctrl_t * const p_ctrl = (mxic_vee_instance_ctrl_t *) p_api_ctrl;
    uint32_t entry;
    uint8_t find_la_number = 0;
    uint8_t LPA_start = DATA_NONE8;
    struct eeprom_header header;

    entry = sector * MX_EEPROM_ENTRIES_PER_SECTOR + MX_EEPROM_ENTRIES_PER_SECTOR - 1;/* Read from the last entry header of each sector */
    for (uint8_t cnt = 0; cnt < MX_EEPROM_ENTRIES_PER_SECTOR; cnt++, entry--) {
        if (mx_ee_read(p_ctrl, bi, entry, &header, true)) {
            mx_err("mxee_build: fail to read entry %lu\r\n", entry);
        }
        if (find_la_number >= MX_EEPROM_LPAS_PER_SECTOR)
            break;

        if (header.LPA == DATA_NONE8) { /* Free entry */
            continue;
        }

        if (header.LPA < MX_EEPROM_LPAS_PER_CLUSTER) {

            if (LPA_start == DATA_NONE8)
                LPA_start = header.LPA - (header.LPA % MX_EEPROM_LPAS_PER_SECTOR);

            if( bi->latest_used_entry_per_sector[sector] == DATA_NONE8)
                bi->latest_used_entry_per_sector[sector] = entry % MX_EEPROM_ENTRIES_PER_SECTOR;

            /* Update L2P mapping */
            if (bi->l2ps[header.LPA] == DATA_NONE8) {
                bi->l2ps[header.LPA] = sector;
                bi->l2pe[header.LPA] = entry % MX_EEPROM_ENTRIES_PER_SECTOR;
                find_la_number++;
                continue;
            } else if (bi->l2ps[header.LPA] == sector) {
                if (bi->l2pe[header.LPA] == DATA_NONE8) {
                    bi->l2pe[header.LPA] = entry % MX_EEPROM_ENTRIES_PER_SECTOR;
                    find_la_number++;
                }
                continue;
            }
        } else {
            /* Erase obsoleted sector */
            bi->dirty_block = bi->block;
            bi->dirty_sector = sector;
            if (mx_ee_erase(p_ctrl, bi))
              mx_err("mxee_build: fail to erase sector %lu\r\n", sector);
        }
    }

    if (bi->l2ps_group[LPA_start / MX_EEPROM_LPAS_PER_SECTOR] !=  DATA_NONE8) {
        for (uint8_t cnt = 0; cnt < MX_EEPROM_LPAS_PER_SECTOR; cnt++) {
            if (bi->l2pe[LPA_start + cnt] == DATA_NONE8) {
                bi->l2ps[LPA_start + cnt] = NOT_EXIT;
                bi->l2pe[LPA_start + cnt] = NOT_EXIT;
            }
        }
    }

    return FSP_SUCCESS;
}

#endif


/**
  * @brief  Locate the latest version of specified logical page.
  * @param  bi: Current bank handle
  * @param  LPA: Local logical page address
  * @param  free: Find the latest version (false) or next free slot (true)
  * @retval Local entry address
  */
#ifdef SPINAND
static uint32_t mx_ee_find_latest(mxic_vee_ctrl_t * const p_api_ctrl, struct bank_info *bi, uint32_t LPA,
                                  bool free)
{
    mxic_vee_instance_ctrl_t * const p_ctrl = (mxic_vee_instance_ctrl_t *) p_api_ctrl;
    uint32_t entry;
    uint8_t LPA_start;

    /* Check address validity */
    if ((bi->bank >= MX_EEPROMS) ||
        (bi->block >= MX_EEPROM_BLOCKS) ||
        (LPA >= MX_EEPROM_LPAS_PER_CLUSTER))
        return DATA_NONE32;

    if (bi->l2ps[LPA] == NOT_EXIT) {    /* LPA is not exit */
        return DATA_NONE32;
    } else if (bi->l2ps[LPA] == DATA_NONE8) { /* LPA is not scanned */
        if (bi->l2ps_group[LPA / MX_EEPROM_LPAS_PER_SECTOR] ==  DATA_NONE8) {
            return DATA_NONE32;              /* means LPA is not exit */
        } else {
            mx_ee_build_l2p_in_a_sector(p_ctrl, bi, bi->l2ps_group[LPA / MX_EEPROM_LPAS_PER_SECTOR]);
            if (bi->l2ps[LPA] == NOT_EXIT)
                return DATA_NONE32;
        }
    } else {
        if (bi->l2pe[LPA] == DATA_NONE8) {                   /* LPA is exit, but it is not sure if it is the latest */
            mx_ee_build_l2p_in_a_sector(p_ctrl, bi, bi->l2ps_group[LPA / MX_EEPROM_LPAS_PER_SECTOR]);
        } else if ((bi->l2pe[LPA] == 0) && (bi->l2pe[LPA + 1] == 0)) {/* two l2pe = 0 means this is power fail recovery */
            LPA_start = LPA - (LPA % MX_EEPROM_LPAS_PER_SECTOR);        /* for power fail recovery*/
            bi->l2ps_group[LPA / MX_EEPROM_LPAS_PER_SECTOR] = bi->l2ps[LPA];
            for (uint8_t cnt = 0; cnt < MX_EEPROM_LPAS_PER_SECTOR; cnt++) {
                bi->l2ps[LPA_start + cnt] = DATA_NONE8;
                bi->l2pe[LPA_start + cnt] = DATA_NONE8;
            }
            mx_ee_build_l2p_in_a_sector(p_ctrl, bi, bi->l2ps_group[LPA / MX_EEPROM_LPAS_PER_SECTOR]);
        }
    }

    entry = bi->l2ps[LPA] * MX_EEPROM_ENTRIES_PER_SECTOR + bi->l2pe[LPA];

    /* if bi->l2ps[LPA] and bi->l2pe[LPA] are not none means latest_used_entry_per_sector[LPA] is set */
    if (!free)
        return entry;
    else if (bi->latest_used_entry_per_sector[bi->l2ps[LPA]] == MX_EEPROM_ENTRIES_PER_SECTOR - 1)
        return DATA_NONE32;
    else
        return bi->l2ps[LPA] * MX_EEPROM_ENTRIES_PER_SECTOR + bi->latest_used_entry_per_sector[bi->l2ps[LPA]] + 1;

}
#else
static uint32_t mx_ee_find_latest(mxic_vee_ctrl_t * const p_api_ctrl, struct bank_info *bi, uint32_t LPA,
                                  bool free)
{
  mxic_vee_instance_ctrl_t * const p_ctrl = (mxic_vee_instance_ctrl_t *) p_api_ctrl;
  struct eeprom_header header;
  uint32_t ofs, entry, cnt = 0, latest = DATA_NONE32;

  /* Check address validity */
  if ((bi->bank >= MX_EEPROMS) ||
      (bi->block >= MX_EEPROM_BLOCKS) ||
      (LPA >= MX_EEPROM_LPAS_PER_CLUSTER))
    return DATA_NONE32;

  ofs = bi->l2ps[LPA];

  /* No mapping */
  if (ofs >= MX_EEPROM_DATA_SECTORS)
    return DATA_NONE32;

  ofs *= MX_EEPROM_ENTRIES_PER_SECTOR;
  entry = bi->l2pe[LPA];

  /* Check if entry mapping exits */
  if (entry)
  {
    assert_param(entry < MX_EEPROM_ENTRIES_PER_SECTOR);

    if (!free)
      return entry + ofs;
    else if (entry == MX_EEPROM_ENTRIES_PER_SECTOR - 1)
      return DATA_NONE32;
    else
      cnt = entry + 1;
  }

  /* Read each entry to find the latest, TODO: binary search method */
  for (entry = ofs + cnt; cnt < MX_EEPROM_ENTRIES_PER_SECTOR; cnt++, entry++)
  {
    /* Read entry header, XXX: Potential risk to check header only? */
    if (mx_ee_read(p_ctrl, bi, entry, &header, true))
    {
      mx_err("mxee_latst: fail to read entry %lu\r\n", entry);
      continue;
    }

    if (header.LPA == LPA)
    {
      /* Found newer version */
      latest = entry;
    }
    else if (header.LPA == DATA_NONE8)
    {
      /* Empty entry */
      break;
    }
    else
    {
      mx_err("mxee_latst: corrupted entry, bank %lu, block %lu, entry %lu\r\n",
              bi->bank, bi->block, entry);
    }
  }

  /* Update L2PE mapping */
  if (latest != DATA_NONE32)
    bi->l2pe[LPA] = latest - ofs;

  /* Return latest entry */
  if (!free)
  {
    assert_param(latest != DATA_NONE32);
    return latest;
  }

  /* For testing */
  if (bi->l2pe[LPA] > 0 && bi->l2pe[LPA] < MX_EEPROM_ENTRIES_PER_SECTOR - 1 && bi->l2pe[LPA] + 1 != cnt)
  {
    mx_err("mxee_latst: alert!!! bank %lu, block %lu, LPA %lu, L2PS %u, L2PE %u, free %lu\r\n",
            bi->bank, bi->block, LPA, bi->l2ps[LPA], bi->l2pe[LPA], cnt);
    assert_param(0);
  }

  /* Return next free entry */
  if (cnt != MX_EEPROM_ENTRIES_PER_SECTOR)
    return entry;

  return DATA_NONE32;
}
#endif

/**
  * @brief  Scan current block to build mapping table.
  * @param  bi: Current bank handle
  * @param  block: Local block address
  * @retval Status
  */
static fsp_err_t mx_ee_build_mapping(mxic_vee_ctrl_t * const p_api_ctrl, struct bank_info *bi, uint32_t block)
{
  mxic_vee_instance_ctrl_t * const p_ctrl = (mxic_vee_instance_ctrl_t *) p_api_ctrl;
  struct eeprom_header header;
  uint32_t sector, entry, victim;

  /* Check address validity */
  if ((bi->bank >= MX_EEPROMS) || (block >= MX_EEPROM_BLOCKS))
    goto err;

  /* Reset L2P and P2L mappings */
#ifdef SPINAND
  bi->block = block;
  bi->block_offset = block * MX_EEPROM_CLUSTER_OFFSET + bi->bank_offset;
  memset(bi->l2ps, DATA_NONE8, sizeof(bi->l2ps));
  memset(bi->l2pe, DATA_NONE8, sizeof(bi->l2pe));
  memset(bi->p2l, DATA_NONE8, sizeof(bi->p2l));
  memset(bi->l2ps_group, DATA_NONE8, sizeof(bi->l2ps_group));
  memset(bi->latest_used_entry_per_sector, DATA_NONE8, sizeof(bi->latest_used_entry_per_sector));
#else
  bi->block = block;
  bi->block_offset = block * MX_EEPROM_CLUSTER_SIZE + bi->bank_offset;
  memset(bi->l2ps, DATA_NONE8, sizeof(bi->l2ps));
  memset(bi->l2pe, 0, sizeof(bi->l2pe));
  memset(bi->p2l, DATA_NONE8, sizeof(bi->p2l));
#endif

  for (sector = 0; sector < MX_EEPROM_DATA_SECTORS; sector++)
  {
    entry = sector * MX_EEPROM_ENTRIES_PER_SECTOR;

    /* Read the first entry header of each sector */
    if (mx_ee_read(p_ctrl, bi, entry, &header, true))
    {
      mx_err("mxee_build: fail to read entry %lu\r\n", entry);

      /* XXX: Potential risk to erase? */
      victim = sector;
      goto erase;
    }

    /* Free sector */
    if (header.LPA == DATA_NONE8)
      continue;

    /* Update P2L mapping */
    bi->p2l[sector] = header.LPA;
#ifdef SPINAND
    bi->l2ps_group[header.LPA / MX_EEPROM_LPAS_PER_SECTOR] = sector;
#endif

    if (header.LPA < MX_EEPROM_LPAS_PER_CLUSTER)
    {
      /* Update L2P mapping */
      if (bi->l2ps[header.LPA] == DATA_NONE8)
      {
        bi->l2ps[header.LPA] = sector;
        continue;
      }

      /* Handle mapping conflict */
      victim = bi->l2ps[header.LPA];
      entry = mx_ee_find_latest(p_ctrl, bi, header.LPA, true);
      if (entry / MX_EEPROM_ENTRIES_PER_SECTOR == victim)
        victim = sector;
    }
    else
      victim = sector;

erase:
    /* Erase obsoleted sector */
    bi->dirty_block = block;
    bi->dirty_sector = victim;
    if (mx_ee_erase(p_ctrl, bi))
      mx_err("mxee_build: fail to erase sector %lu\r\n", victim);

    /* Repair L2P mapping */
    if (victim != sector)
    {
      bi->l2ps[header.LPA] = sector;
      bi->l2pe[header.LPA] = 0;
    }
  }

  return FSP_SUCCESS;
err:
  bi->block = DATA_NONE32;
  bi->block_offset = DATA_NONE32;
  return FSP_ERR_INVALID_ARGUMENT;
}

#ifdef SPINAND
static uint32_t mx_ee_gc_cp(mxic_vee_ctrl_t * const p_api_ctrl, struct bank_info *bi, uint8_t src_sector, uint8_t des_sector)
{
    mxic_vee_instance_ctrl_t * const p_ctrl = (mxic_vee_instance_ctrl_t *) p_api_ctrl;
    uint32_t LPA, LPA_start, src_entry, des_entry, ret;
    struct eeprom_header header;
    struct eeprom_entry cache_tmp;

    src_entry = src_sector * MX_EEPROM_ENTRIES_PER_SECTOR;
    des_entry = des_sector * MX_EEPROM_ENTRIES_PER_SECTOR;

    ret = mx_ee_read(p_ctrl, bi, src_entry, &header, true);
    if (ret) {
        mx_err("mx_ee_gc_cp: fail to read src_entry %lu\r\n", src_entry);
    }
    LPA = header.LPA;
    if (bi->l2ps[LPA] != src_sector) { /* for power fail recovery*/
        bi->l2ps[LPA] = src_sector;
    }

    LPA_start = LPA - (LPA % MX_EEPROM_LPAS_PER_SECTOR);
    for (int i = 0; i < MX_EEPROM_LPAS_PER_SECTOR; i++) {
        src_entry = mx_ee_find_latest(p_ctrl, bi, LPA_start + i, false);
        if (src_entry < MX_EEPROM_ENTRIES_PER_CLUSTER) {
            ret = mx_ee_read(p_ctrl, bi, src_entry, &cache_tmp, false);
            if (ret) {
                mx_err("mx_ee_gc_cp: fail to read src_entry %lu\r\n", src_entry);
                return ret;
            }
            ret = mx_ee_write(p_ctrl, bi, des_entry, &cache_tmp);
            if (ret) {
                mx_err("mx_ee_gc_cp: fail to write des_entry %lu\r\n", des_entry);
                return ret;
            }

            /* Update L2E mapping */
            bi->l2pe[LPA_start + i] = des_entry % MX_EEPROM_ENTRIES_PER_SECTOR;
            bi->l2ps[LPA_start + i] = des_sector;
            bi->l2ps_group[(LPA_start + i) / MX_EEPROM_LPAS_PER_SECTOR] = des_sector;
            bi->latest_used_entry_per_sector[des_sector] = des_entry % MX_EEPROM_ENTRIES_PER_SECTOR;
            bi->p2l[des_sector] = LPA;
            des_entry ++;
        }
    }
    return des_entry;
}

static uint32_t mx_ee_gc(mxic_vee_ctrl_t * const p_api_ctrl, struct bank_info *bi, uint8_t src_sector, uint8_t des_sector)
{
    mxic_vee_instance_ctrl_t * const p_ctrl = (mxic_vee_instance_ctrl_t *) p_api_ctrl;
    uint32_t des_entry;

    /* Obsolete sector */
    bi->dirty_block = bi->block;
    bi->dirty_sector = src_sector;

    //mark OPS_GC_CP_BEGIN and des_addr(sector number) in the oob of source sector before cp data
    mx_ee_update_sys(p_ctrl, bi, src_sector, SYS_ENTRY_ADDR_G_B_S, OPS_GC_CP_BEGIN, des_sector);

    //mark OPS_GC_CP_BEGIN and src_addr(sector number) in the oob of destination sector before cp data
    mx_ee_update_sys(p_ctrl, bi, des_sector, SYS_ENTRY_ADDR_G_B_D, OPS_GC_CP_BEGIN, src_sector);

    //copy data from source sector to destination sector
    des_entry = mx_ee_gc_cp(p_ctrl, bi, src_sector, des_sector);

    //mark OPS_GC_CP_END in the oob of source sector after cp data
    mx_ee_update_sys(p_ctrl, bi, src_sector, SYS_ENTRY_ADDR_G_E, OPS_GC_CP_END, DATA_NONE16);

    if (mx_ee_erase(p_ctrl, bi))
        mx_err("mxee_rwbuf: fail to erase\r\n");

    return des_entry;
}
#endif

/**
  * @brief  Find a free entry for given logical page.
  * @param  bi: Current bank handle
  * @param  LPA: Local logical page address
  * @retval Local free entry address
  */
#ifdef SPINAND
static uint32_t mx_ee_search_free(mxic_vee_ctrl_t * const p_api_ctrl, struct bank_info *bi, uint32_t LPA)
{
    mxic_vee_instance_ctrl_t * const p_ctrl = (mxic_vee_instance_ctrl_t *) p_api_ctrl;
    uint32_t entry, sector, cnt;

    /* Check if corresponding sector used up */
    entry = mx_ee_find_latest(p_ctrl, bi, LPA, true);
    if (entry < MX_EEPROM_ENTRIES_PER_CLUSTER)
        return entry;

    // if no entry is found means three cases:
    //  case1. this LPA don't exit, but some same group LPA is located in a sector, and this sector is not full
    //  case2. this LPA don't exit, and no same group LPA exit : need find a free sector
    //  case3. this LPA exit, but the sector this LPA located is full : need find a free sector and do gc

    /* case1 */
    if ((bi->l2ps[LPA] == NOT_EXIT) && (bi->l2ps_group[LPA / MX_EEPROM_LPAS_PER_SECTOR] != DATA_NONE8)) {
        if (bi->latest_used_entry_per_sector[bi->l2ps_group[LPA / MX_EEPROM_LPAS_PER_SECTOR]] != MX_EEPROM_ENTRIES_PER_SECTOR - 1) {
              entry = bi->l2ps_group[LPA / MX_EEPROM_LPAS_PER_SECTOR] * MX_EEPROM_ENTRIES_PER_SECTOR;
              entry += bi->latest_used_entry_per_sector[bi->l2ps_group[LPA / MX_EEPROM_LPAS_PER_SECTOR]] + 1;
              return entry;
        }
    }

    /* Pick a random sector to start the search */
    sector = rand() % MX_EEPROM_DATA_SECTORS;

    /* Search from the random sector to the end sector */
    for (cnt = sector; cnt < MX_EEPROM_DATA_SECTORS; cnt++) {
        if (bi->p2l[cnt] == DATA_NONE8) {
            entry = cnt * MX_EEPROM_ENTRIES_PER_SECTOR;
            if (bi->l2ps_group[LPA / MX_EEPROM_LPAS_PER_SECTOR] == DATA_NONE8) {
                return entry;                       /* case2 */
            } else {
                return mx_ee_gc(p_ctrl, bi, bi->l2ps_group[LPA / MX_EEPROM_LPAS_PER_SECTOR], cnt);  /* case3 */
            }
        }
    }

    /* Search from the start sector to the random sector */
    for (cnt = 0; cnt < sector; cnt++) {
        if (bi->p2l[cnt] == DATA_NONE8) {
            entry = cnt * MX_EEPROM_ENTRIES_PER_SECTOR;
            if (bi->l2ps_group[LPA / MX_EEPROM_LPAS_PER_SECTOR] == DATA_NONE8) {
                return entry;                       /* case2 */
            } else {
                return mx_ee_gc(p_ctrl, bi, bi->l2ps_group[LPA / MX_EEPROM_LPAS_PER_SECTOR], cnt);  /* case3 */
            }
        }
    }
    return DATA_NONE32;
}
#else
static uint32_t mx_ee_search_free(mxic_vee_ctrl_t * const p_api_ctrl, struct bank_info *bi, uint32_t LPA)
{
  mxic_vee_instance_ctrl_t * const p_ctrl = (mxic_vee_instance_ctrl_t *) p_api_ctrl;
  uint32_t entry, sector, cnt;

  /* Check if corresponding sector used up */
  entry = mx_ee_find_latest(p_ctrl, bi, LPA, true);
  if (entry < MX_EEPROM_ENTRIES_PER_CLUSTER)
    return entry;

  /* Pick a random sector to start the search */
  sector = rand() % MX_EEPROM_DATA_SECTORS;

  /* Search from the random sector to the end sector */
  for (cnt = sector; cnt < MX_EEPROM_DATA_SECTORS; cnt++)
  {
    if (bi->p2l[cnt] == DATA_NONE8)
    {
      entry = cnt * MX_EEPROM_ENTRIES_PER_SECTOR;
      return entry;
    }
  }

  /* Search from the start sector to the random sector */
  for (cnt = 0; cnt < sector; cnt++)
  {
    if (bi->p2l[cnt] == DATA_NONE8)
    {
      entry = cnt * MX_EEPROM_ENTRIES_PER_SECTOR;
      return entry;
    }
  }

  return DATA_NONE32;
}
#endif

/**
  * @brief  Read specified logical page of current block of current bank.
  * @param  bi: Current bank handle
  * @param  LPA: Local logical page address
  * @retval Status
  */
static fsp_err_t mx_ee_read_page(mxic_vee_ctrl_t * const p_api_ctrl, struct bank_info *bi, uint32_t LPA)
{
  mxic_vee_instance_ctrl_t * const p_ctrl = (mxic_vee_instance_ctrl_t *) p_api_ctrl;
  uint32_t entry;
  int ret, retries = 0;

  /* Check address validity */
  if ((bi->bank >= MX_EEPROMS) ||
      (bi->block >= MX_EEPROM_BLOCKS) ||
      (LPA >= MX_EEPROM_LPAS_PER_CLUSTER))
  {
    ret = FSP_ERR_INVALID_ARGUMENT;
    goto err;
  }

  /* Find the latest version */
  entry = mx_ee_find_latest(p_ctrl, bi, LPA, false);

  /* Fresh read */
  if (entry >= MX_EEPROM_ENTRIES_PER_CLUSTER)
  {
    memset(bi->cache.data, DATA_NONE8, MX_EEPROM_PAGE_SIZE);
    bi->cache.header.LPA = LPA;
    return FSP_SUCCESS;
  }

retry:
  /* Do the real read */
  if (mx_ee_read(p_ctrl, bi, entry, &bi->cache, false) || bi->cache.header.LPA != LPA)
  {
    mx_err("mxee_rpage: fail to read entry %lu\r\n", entry);

    if (retries++ < MX_EEPROM_READ_RETRIES)
    {
#ifdef MX_EEPROM_READ_ROLLBACK
      /* Roll back to the prior version */
      if (entry % MX_EEPROM_ENTRIES_PER_SECTOR)
        entry--;
#endif
      goto retry;
    }

    ret = FSP_ERR_INVALID_STATE;
    goto err;
  }

  return FSP_SUCCESS;
err:
  bi->cache_dirty = false;
  bi->cache.header.LPA = DATA_NONE8;
  return ret;
}

/**
  * @brief  Write specified logical page of current block of current bank.
  * @param  bi: Current bank handle
  * @param  LPA: Local logical page address
  * @retval Status
  */
#ifdef SPINAND
static fsp_err_t mx_ee_write_page(mxic_vee_ctrl_t * const p_api_ctrl, struct bank_info *bi, uint32_t LPA)
{
    mxic_vee_instance_ctrl_t * const p_ctrl = (mxic_vee_instance_ctrl_t *) p_api_ctrl;
    uint32_t entry, ofs, sector;
    int ret, retries = 0;

    /* Check address validity */
    if ((bi->bank >= MX_EEPROMS) ||
        (bi->block >= MX_EEPROM_BLOCKS) ||
        (LPA >= MX_EEPROM_LPAS_PER_CLUSTER))
        return FSP_ERR_INVALID_ARGUMENT;

retry:
    /* Find next free entry */
    entry = mx_ee_search_free(p_ctrl, bi, LPA);
    if (entry >= MX_EEPROM_ENTRIES_PER_CLUSTER) {
        mx_err("mxee_wpage: no free entry left, bank %lu, block %lu\r\n",
            bi->bank, bi->block);
        return FSP_ERR_OUT_OF_MEMORY;
    }

    /* Do the real write */
    ret = mx_ee_write(p_ctrl, bi, entry, &bi->cache);
    if (ret) {
        mx_err("mxee_wpage: fail to write entry %lu\r\n", entry);

        if (retries++ < MX_EEPROM_WRITE_RETRIES)
            goto retry;

        return ret;
    }

    ofs = entry % MX_EEPROM_ENTRIES_PER_SECTOR;
    sector = entry / MX_EEPROM_ENTRIES_PER_SECTOR;
    /* Update L2E mapping */
    bi->l2pe[LPA] = ofs;
    bi->l2ps[LPA] = sector;
    bi->p2l[sector] = LPA;
    if (bi->l2ps_group[LPA / MX_EEPROM_LPAS_PER_SECTOR] == DATA_NONE8)
        bi->l2ps_group[LPA / MX_EEPROM_LPAS_PER_SECTOR] = sector;
    bi->latest_used_entry_per_sector[sector] = ofs;

    /* Clean page cache */
    bi->cache_dirty = false;

    return FSP_SUCCESS;
}
#else
static fsp_err_t mx_ee_write_page(mxic_vee_ctrl_t * const p_api_ctrl, struct bank_info *bi, uint32_t LPA)
{
  mxic_vee_instance_ctrl_t * const p_ctrl = (mxic_vee_instance_ctrl_t *) p_api_ctrl;
  uint32_t entry, ofs;
  int ret, retries = 0;

  /* Check address validity */
  if ((bi->bank >= MX_EEPROMS) ||
      (bi->block >= MX_EEPROM_BLOCKS) ||
      (LPA >= MX_EEPROM_LPAS_PER_CLUSTER))
    return FSP_ERR_INVALID_ARGUMENT;

retry:
  /* Find next free entry */
  entry = mx_ee_search_free(p_ctrl, bi, LPA);
  if (entry >= MX_EEPROM_ENTRIES_PER_CLUSTER)
  {
    mx_err("mxee_wpage: no free entry left, bank %lu, block %lu\r\n",
            bi->bank, bi->block);
    return FSP_ERR_OUT_OF_MEMORY;
  }

  /* Do the real write */
  ret = mx_ee_write(p_ctrl, bi, entry, &bi->cache);
  if (ret)
  {
    mx_err("mxee_wpage: fail to write entry %lu\r\n", entry);

    if (retries++ < MX_EEPROM_WRITE_RETRIES)
      goto retry;

    return ret;
  }

  ofs = entry % MX_EEPROM_ENTRIES_PER_SECTOR;
  if (ofs)
  {
    /* Update L2E mapping */
    bi->l2pe[LPA] = ofs;
  }
  else
  {
    ofs = bi->l2ps[LPA];
    if (ofs < MX_EEPROM_DATA_SECTORS)
    {
      /* Obsolete sector */
      bi->dirty_block = bi->block;
      bi->dirty_sector = ofs;
    }

    /* Update L2P and P2L mapping */
    ofs = entry / MX_EEPROM_ENTRIES_PER_SECTOR;
    bi->l2ps[LPA] = ofs;
    bi->l2pe[LPA] = 0;
    bi->p2l[ofs] = LPA;
  }

  /* Clean page cache */
  bi->cache_dirty = false;

  return FSP_SUCCESS;
}
#endif

/**
  * @brief  Handle buffer and cache.
  * @param  bi: Current bank handle
  * @param  addr: Local logical start address
  * @param  len: Request length
  * @param  buf: Data buffer
  * @param  rw: Read (false) or write (true)
  * @retval Status
  */
static fsp_err_t mx_ee_rw_buffer(mxic_vee_ctrl_t * const p_api_ctrl, struct bank_info *bi, uint32_t addr, uint32_t len,
                           uint8_t *buf, bool rw)
{
  mxic_vee_instance_ctrl_t * const p_ctrl = (mxic_vee_instance_ctrl_t *) p_api_ctrl;
  int ret;
  uint32_t block, page, ofs;

  /* Calculate current block, page, offset */
  block = addr / MX_EEPROM_BLOCK_SIZE;
  ofs = addr % MX_EEPROM_BLOCK_SIZE;
  page = ofs / MX_EEPROM_PAGE_SIZE;
  ofs = ofs % MX_EEPROM_PAGE_SIZE;

  if ((bi->block != block) || (bi->cache.header.LPA != page))
  {
    /* Flush dirty page cache */
    if (bi->cache_dirty)
    {
      ret = mx_ee_write_page(p_ctrl, bi, bi->cache.header.LPA);
      if (ret)
      {
        mx_err("mxee_rwbuf: fail to flush page cache\r\n");
        return ret;
      }
    }

    /* Build mapping */
    if (bi->block != block)
    {
      ret = mx_ee_build_mapping(p_ctrl, bi, block);
      if (ret)
      {
        mx_err("mxee_rwbuf: fail to build mapping table, bank %lu, block %lu\r\n",
                bi->bank, bi->block);
        return ret;
      }
    }

    /* Fill page cache */
    if (!rw || len < MX_EEPROM_PAGE_SIZE)
    {
      ret = mx_ee_read_page(p_ctrl, bi, page);
      if (ret)
      {
        mx_err("mxee_rwbuf: fail to fill page cache\r\n");
        bi->cache.header.LPA = DATA_NONE8;
        return ret;
      }
    }
    else
      bi->cache.header.LPA = page;
  }

  /* Update page cache/buffer */
  if (rw)
  {
    memcpy(&bi->cache.data[ofs], buf, len);
    bi->cache_dirty = true;
  }
  else
    memcpy(buf, &bi->cache.data[ofs], len);

  /* Handle obsoleted sector */
  if (mx_ee_erase(p_ctrl, bi))
    mx_err("mxee_rwbuf: fail to erase\r\n");

  return FSP_SUCCESS;
}

/**
  * @brief  Distribute continuous logical address into different banks.
  * @param  addr: Global logical start address
  * @param  len: Request length
  * @param  buf: Data buffer
  * @param  rw: Read (false) or write (true)
  * @retval Status
  */
#ifdef SPINAND
static fsp_err_t mx_ee_rw(mxic_vee_ctrl_t * const p_api_ctrl, uint32_t addr, uint32_t len, uint8_t *buf, bool rw)
{
    mxic_vee_instance_ctrl_t * const p_ctrl = (mxic_vee_instance_ctrl_t *) p_api_ctrl;
    int ret;
    struct bank_info *bi;
    uint32_t ofs, bank, rwpos, rwlen;

    if (addr + len > MX_EEPROM_TOTAL_SIZE)
        return FSP_ERR_INVALID_ARGUMENT;

    /* Determine the rwpos and rwlen */
    bank = addr / MX_EEPROM_SIZE;
    ofs = addr % MX_EEPROM_SIZE;
    rwpos = ofs;
    ofs = ofs % MX_EEPROM_PAGE_SIZE;
    rwlen = min_t(uint32_t, MX_EEPROM_PAGE_SIZE - ofs, len);

    /* Loop to R/W each logical page */
    while (len) {
        bi = &p_ctrl->bi[bank];

        /* Only allow one request per bank per time */
        if (xSemaphoreTake(bi->lock, portMAX_DELAY) != pdTRUE)
            return FSP_ERR_ABORTED;

        do {
            ret = mx_ee_rw_buffer(p_ctrl, bi, rwpos, rwlen, buf, rw);
            if (ret) {
                mx_err("mxee_toprw: fail to %s laddr 0x%08lx, len %lu\r\n",
                rw ? "write" : "read", addr, rwlen);
                xSemaphoreGive(bi->lock);
                return ret;
            }

            /* Calculate the next rwpos and rwlen */
            buf += rwlen;
            addr += rwlen;
            rwpos += rwlen;
            len -= rwlen;
            rwlen = min_t(uint32_t, MX_EEPROM_PAGE_SIZE, len);

        } while (len && (rwpos != MX_EEPROM_SIZE));

        xSemaphoreGive(bi->lock);

        bank++;
        rwpos = 0;
    }

    return FSP_SUCCESS;
}
#else
#if (MX_EEPROM_HASH_AlGORITHM == MX_EEPROM_HASH_CROSSBANK)

static fsp_err_t mx_ee_rw(uint32_t addr, uint32_t len, uint8_t *buf, bool rw)
{
  int ret;
  struct bank_info *bi;
  uint32_t page, ofs, bank, rwpos, rwlen;

  if (addr + len > MX_EEPROM_TOTAL_SIZE)
    return FSP_ERR_INVALID_ARGUMENT;

  /* Determine the rwpos and rwlen */
  page = addr / MX_EEPROM_PAGE_SIZE;
  ofs = addr % MX_EEPROM_PAGE_SIZE;
  bank = page % MX_EEPROMS;
  page = page / MX_EEPROMS;
  rwpos = page * MX_EEPROM_PAGE_SIZE + ofs;
  rwlen = min_t(uint32_t, MX_EEPROM_PAGE_SIZE - ofs, len);

  /* Loop to R/W each logical page */
  while (len)
  {
    bi = &p_ctrl->bi[bank];

    /* Only allow one request per bank per time */
    if (xSemaphoreTake(bi->lock, portMAX_DELAY) != pdTRUE)
         return MX_EOS;


    ret = mx_ee_rw_buffer(bi, rwpos, rwlen, buf, rw);

    xSemaphoreGive(bi->lock);

    if (ret)
    {
      mx_err("mxee_toprw: fail to %s laddr 0x%08lx, len %lu\r\n",
              rw ? "write" : "read", addr, rwlen);
      return ret;
    }

    /* Calculate the next rwpos and rwlen */
    buf += rwlen;
    addr += rwlen;
    len -= rwlen;

    if (ofs)
    {
      rwpos -= ofs;
      ofs = 0;
    }

    if (++bank == MX_EEPROMS)
    {
      bank = 0;
      rwpos += MX_EEPROM_PAGE_SIZE;
    }

    rwlen = min_t(uint32_t, MX_EEPROM_PAGE_SIZE, len);
  }

  return FSP_SUCCESS;
}

#elif (MX_EEPROM_HASH_AlGORITHM == MX_EEPROM_HASH_HYBRID)

#define MX_EEPROM_SUPERBLOCK_SIZE    (MX_EEPROM_BLOCK_SIZE * MX_EEPROMS)
static fsp_err_t mx_ee_rw(uint32_t addr, uint32_t len, uint8_t *buf, bool rw)
{
  int ret;
  struct bank_info *bi;
  uint32_t ofs, bank, base, size, rwpos, rwlen;

  /* Determine the rwpos and rwlen */
  ofs = addr / MX_EEPROM_BLOCK_SIZE;
  bank = ofs % MX_EEPROMS;
  base = (ofs / MX_EEPROMS) * MX_EEPROM_BLOCK_SIZE;
  size = addr % MX_EEPROM_BLOCK_SIZE;
  rwpos = base + size;
  ofs = size % MX_EEPROM_PAGE_SIZE;
  rwlen = min_t(uint32_t, MX_EEPROM_PAGE_SIZE - ofs, len);
  size = MX_EEPROM_BLOCK_SIZE - size;

  /* Loop to R/W each logical page */
  while (len)
  {
    bi = &p_ctrl->bi[bank];

    /* Only allow one request per bank per time */
    if (xSemaphoreTake(bi->lock, portMAX_DELAY) != pdTRUE)
         return MX_EOS;

    while (size && len)
    {
      ret = mx_ee_rw_buffer(bi, rwpos, rwlen, buf, rw);
      if (ret)
      {
        mx_err("mxee_toprw: fail to %s laddr 0x%08lx, len %lu\r\n",
                rw ? "write" : "read", addr, rwlen);
        xSemaphoreGive(bi->lock);
        return ret;
      }

      buf += rwlen;
      rwpos += rwlen;
      addr += rwlen;
      size -= rwlen;
      len -= rwlen;
      rwlen = min_t(uint32_t, MX_EEPROM_PAGE_SIZE, len);
    }

    xSemaphoreGive(bi->lock);

    if (++bank == MX_EEPROMS)
    {
      bank = 0;
      base += MX_EEPROM_BLOCK_SIZE;
    }

    rwpos = base;
    size = MX_EEPROM_BLOCK_SIZE;
  }

  return FSP_SUCCESS;
}

#elif (MX_EEPROM_HASH_AlGORITHM == MX_EEPROM_HASH_SEQUENTIAL)

static fsp_err_t mx_ee_rw(mxic_vee_ctrl_t * const p_api_ctrl, uint32_t addr, uint32_t len, uint8_t *buf, bool rw)
{
  mxic_vee_instance_ctrl_t * const p_ctrl = (mxic_vee_instance_ctrl_t *) p_api_ctrl;
  int ret;
  struct bank_info *bi;
  uint32_t ofs, bank, rwpos, rwlen;

  if (addr + len > MX_EEPROM_TOTAL_SIZE)
    return FSP_ERR_INVALID_ARGUMENT;

  /* Determine the rwpos and rwlen */
  bank = addr / MX_EEPROM_SIZE;
  ofs = addr % MX_EEPROM_SIZE;
  rwpos = ofs;
  ofs = ofs % MX_EEPROM_PAGE_SIZE;
  rwlen = min_t(uint32_t, MX_EEPROM_PAGE_SIZE - ofs, len);

  /* Loop to R/W each logical page */
  while (len)
  {
    bi = &p_ctrl->bi[bank];

    /* Only allow one request per bank per time */
    if (xSemaphoreTake(bi->lock, portMAX_DELAY) != pdTRUE)
         return FSP_ERR_ABORTED;
    do
    {
      ret = mx_ee_rw_buffer(p_ctrl, bi, rwpos, rwlen, buf, rw);
      if (ret)
      {
        mx_err("mxee_toprw: fail to %s laddr 0x%08lx, len %lu\r\n",
                rw ? "write" : "read", addr, rwlen);
        xSemaphoreGive(bi->lock);
        return ret;
      }

      /* Calculate the next rwpos and rwlen */
      buf += rwlen;
      addr += rwlen;
      rwpos += rwlen;
      len -= rwlen;
      rwlen = min_t(uint32_t, MX_EEPROM_PAGE_SIZE, len);

    } while (len && (rwpos != MX_EEPROM_SIZE));

    xSemaphoreGive(bi->lock);

    bank++;
    rwpos = 0;
  }

  return FSP_SUCCESS;
}
#endif
#endif

/**
  * @brief  EEPROM read API.
  * @param  addr: Start address
  * @param  len: Request length
  * @param  buf: Data buffer
  * @retval Status
  */
fsp_err_t RM_VEE_Read(mxic_vee_ctrl_t * const p_api_ctrl, uint32_t addr, uint32_t len, uint8_t *buf)
{
  mxic_vee_instance_ctrl_t * const p_ctrl = (mxic_vee_instance_ctrl_t *) p_api_ctrl;
  if (!p_ctrl->init)
    return FSP_ERR_NOT_INITIALIZED;

  return mx_ee_rw(p_api_ctrl, addr, len, buf, false);
}

/**
  * @brief  EEPROM write API.
  * @param  addr: Start address
  * @param  len: Request length
  * @param  buf: Data buffer
  * @retval Status
  */
fsp_err_t RM_VEE_Write(mxic_vee_ctrl_t * const p_api_ctrl, uint32_t addr, uint32_t len, uint8_t *buf)
{
  mxic_vee_instance_ctrl_t * const p_ctrl = (mxic_vee_instance_ctrl_t *) p_api_ctrl;
  if (!p_ctrl->init)
    return FSP_ERR_NOT_INITIALIZED;

  return mx_ee_rw(p_api_ctrl, addr, len, buf, true);
}

/**
  * @brief  Write dirty cache back (For internal use only).
  * @param  bi: Current bank handle
  * @retval Status
  */
static fsp_err_t mx_eeprom_wb(mxic_vee_ctrl_t * const p_api_ctrl, struct bank_info *bi)
{
  mxic_vee_instance_ctrl_t * const p_ctrl = (mxic_vee_instance_ctrl_t *) p_api_ctrl;
  /* Write page cache back */
  if (mx_ee_write_page(p_ctrl, bi, bi->cache.header.LPA))
  {
    mx_err("mxee_wback: fail to flush page cache\r\n");
    return FSP_ERR_WRITE_FAILED;
  }

  /* Handle obsoleted sector */
  if (mx_ee_erase(p_ctrl, bi))
  {
    mx_err("mxee_wback: fail to erase\r\n");
    return FSP_ERR_ERASE_FAILED;
  }

  return FSP_SUCCESS;
}

/**
  * @brief  EEPROM cache write back API.
  * @retval Status
  */
fsp_err_t RM_VEE_WriteBack(mxic_vee_ctrl_t * const p_api_ctrl)
{
  mxic_vee_instance_ctrl_t * const p_ctrl = (mxic_vee_instance_ctrl_t *) p_api_ctrl;
  uint32_t bank;
  int ret = FSP_SUCCESS;
  struct bank_info *bi;

  if (!p_ctrl->init)
    return FSP_ERR_NOT_INITIALIZED;

  /* Loop to check each bank */
  for (bank = 0; bank < MX_EEPROMS; bank++)
  {
    bi = &p_ctrl->bi[bank];

    /* Check dirty cache */
    if (!bi->cache_dirty)
      continue;

    if (xSemaphoreTake(bi->lock, portMAX_DELAY) != pdTRUE)
         return FSP_ERR_ABORTED;

    /* Double check */
    if (bi->cache_dirty)
    {
      /* Write back */
      if (mx_eeprom_wb(p_ctrl, bi))
      {
        mx_err("mxee_wback: fail to write back\r\n");
        ret = FSP_ERR_WRITE_FAILED;
      }
    }

    xSemaphoreGive(bi->lock);
  }

  return ret;
}

/**
  * @brief  EEPROM sync write API.
  * @param  addr: Start address
  * @param  len: Request length
  * @param  buf: Data buffer
  * @retval Status
  */
fsp_err_t RM_VEE_SyncWrite(mxic_vee_ctrl_t * const p_api_ctrl, uint32_t addr, uint32_t len, uint8_t *buf)
{
  return (RM_VEE_Write(p_api_ctrl, addr, len, buf) || RM_VEE_WriteBack(p_api_ctrl));
}

/**
  * @brief  EEPROM user cache and meta data flush API.
  *         NOTE: Call this API just before power down.
  * @retval Status
  */
fsp_err_t RM_VEE_Flush(mxic_vee_ctrl_t * const p_api_ctrl)
{
  mxic_vee_instance_ctrl_t * const p_ctrl = (mxic_vee_instance_ctrl_t *) p_api_ctrl;
  int ret = FSP_SUCCESS;
#ifdef MX_EEPROM_PC_PROTECTION
  struct bank_info *bi;
  uint32_t bank, block;
#endif

  ret = RM_VEE_WriteBack(p_api_ctrl);
  if (ret)
    return ret;

#ifdef MX_EEPROM_PC_PROTECTION
  /* Update system entry */
  for (bank = 0; bank < MX_EEPROMS; bank++)
  {
    bi = &p_ctrl->bi[bank];

    for (block = 0; block < MX_EEPROM_BLOCKS; block++)
    {
      if (xSemaphoreTake(bi->lock, portMAX_DELAY) != pdTRUE)
          return FSP_ERR_ABORTED;

#ifndef SPINAND
      if (mx_ee_update_sys(p_ctrl, bi, block, OPS_NONE, DATA_NONE32))
      {
        mx_err("mxee_flush: fail to update system entry\r\n");
        ret = FSP_ERR_ABORTED;
      }
#endif

      xSemaphoreGive(bi->lock);
    }
  }
#endif

  return ret;
}

/**
  * @brief  Handle wear leveling.
  * @retval Status
  */
static fsp_err_t mx_eeprom_wear_leveling(mxic_vee_ctrl_t * const p_api_ctrl)
{
  mxic_vee_instance_ctrl_t * const p_ctrl = (mxic_vee_instance_ctrl_t *) p_api_ctrl;
  int ret = FSP_SUCCESS;
  struct bank_info *bi;
  static uint32_t rwCnt = 0;
  uint32_t bank, page, sector;

  /* Do wear leveling at fixed frequency */
  if (p_ctrl->rwCnt - rwCnt < MX_EEPROM_WL_INTERVAL)
  {
    rwCnt = p_ctrl->rwCnt;
    return FSP_SUCCESS;
  }

  /* Choose a random logical page */
  bank = rand() % MX_EEPROMS;
  page = rand() % MX_EEPROM_LPAS_PER_CLUSTER;

  bi = &p_ctrl->bi[bank];

  /* Get current bank lock */
  if (xSemaphoreTake(bi->lock, portMAX_DELAY) != pdTRUE)
     return FSP_ERR_ABORTED;

  /* Skip unmapped page */
  sector = bi->l2ps[page];
  if (sector >= MX_EEPROM_DATA_SECTORS)
    goto out;

  if (bi->cache_dirty)
  {
    /* Skip the current dirty page */
    if (bi->cache.header.LPA == page)
      goto out;

    /* Flush dirty cache first */
    ret = mx_eeprom_wb(p_ctrl, bi);
    if (ret)
    {
      mx_err("mxee_wearl: fail to clean cache\r\n");
      goto out;
    }
  }

  /* Read the page out */
  ret = mx_ee_read_page(p_ctrl, bi, page);
  if (ret)
  {
    mx_err("mxee_wearl: fail to read page %lu\r\n", page);
    bi->cache.header.LPA = DATA_NONE8;
    goto out;
  }

  /* Set cache dirty */
  bi->cache_dirty = true;

  /* Cheat the free entry selector */
  bi->l2pe[page] = MX_EEPROM_ENTRIES_PER_SECTOR - 1;

  /* Flush the dirty cache */
  ret = mx_eeprom_wb(p_ctrl, bi);
  if (ret)
    mx_err("mxee_wearl: fail to write back\r\n");

out:
  /* Release current bank lock */
  xSemaphoreGive(bi->lock);

  return ret;
}

/**
  * @brief  EEPROM background task API.
  *         NOTE: Call this API just before MCU sleep.
  */
void RM_VEE_Background(mxic_vee_ctrl_t * const p_api_ctrl)
{
  mxic_vee_instance_ctrl_t * const p_ctrl = (mxic_vee_instance_ctrl_t *) p_api_ctrl;
#ifdef MX_EEPROM_BACKGROUND_THREAD
  uint32_t cnt, prevTime;
  bool threadMode = *(bool *)(&(p_ctrl->bgStart));

  prevTime = xTaskGetTickCount();

  for (cnt = 0; ; cnt++)
  {
    mx_info("mxee_bTask: wake-up times: %lu\r\n", cnt);
#endif
    /* Flush dirty page cache */
    if (RM_VEE_WriteBack(p_ctrl))
      mx_err("mxee_bTask: fail to flush cache\r\n");

    /* Check Wear eveling */
    if (mx_eeprom_wear_leveling(p_ctrl))
      mx_err("mxee_bTask: fail to WL\r\n");

#ifdef MX_EEPROM_BACKGROUND_THREAD
    if (!(*(bool *)(&(p_ctrl->bgStart))))
      break;

    /* Wake up periodically */
    vTaskDelayUntil(&prevTime, MX_EEPROM_BG_THREAD_DELAY/portTICK_PERIOD_MS);
  }

    mx_info("mxee_bTask: Goodbye\r\n");
    /* Terminate itself */
    vTaskDelete(NULL);


#endif
}

#ifdef MX_EEPROM_PC_PROTECTION
/**
  * @brief  Handle insufficient sector erase.
  * @param  bi: Current bank handle
  * @param  sector: Victim sector address
  * @retval Status
  */
static fsp_err_t mx_ee_check_erase(mxic_vee_ctrl_t * const p_api_ctrl, struct bank_info *bi, uint32_t sector)
{
  mxic_vee_instance_ctrl_t * const p_ctrl = (mxic_vee_instance_ctrl_t *) p_api_ctrl;
  uint32_t entry;
  struct eeprom_header header;

  /* Check address validity */
  if ((bi->bank >= MX_EEPROMS) ||
      (bi->block >= MX_EEPROM_BLOCKS) ||
      (sector >= MX_EEPROM_DATA_SECTORS))
    return FSP_ERR_INVALID_ARGUMENT;

  /* XXX: Only check the first and last entry? */

  /* Check the first entry header */
  entry = sector * MX_EEPROM_ENTRIES_PER_SECTOR;
  if (mx_ee_read(p_ctrl, bi, entry, &header, true) ||
     (header.LPA != DATA_NONE8 && header.LPA >= MX_EEPROM_LPAS_PER_CLUSTER))
  {
    mx_err("mxee_ckers: fail to read entry 0\r\n");
    goto erase;
  }

  /* Check the last entry header */
  entry += MX_EEPROM_ENTRIES_PER_SECTOR - 1;
  if (mx_ee_read(p_ctrl, bi, MX_EEPROM_ENTRIES_PER_SECTOR - 1, &header, true) ||
     (header.LPA != DATA_NONE8))
  {
    mx_err("mxee_ckers: fail to read entry %d\r\n",
            MX_EEPROM_ENTRIES_PER_SECTOR - 1);
    goto erase;
  }

  return FSP_SUCCESS;

erase:
  mx_info("mxee_ckers: detected insufficient erase, block %lu, sector %lu\r\n",
           bi->block, sector);
  bi->dirty_block = bi->block;
  bi->dirty_sector = sector;
  return mx_ee_erase(p_ctrl, bi);
}
#endif

/**
  * @brief  Check system info and handle power cycling.
  * @retval Status
  */
#ifdef SPINAND
static fsp_err_t mx_ee_check_sys(mxic_vee_ctrl_t * const p_api_ctrl)
{
    mxic_vee_instance_ctrl_t * const p_ctrl = (mxic_vee_instance_ctrl_t *) p_api_ctrl;
    uint8_t bank, block, sector;
    bool formatted = false;
    struct system_entry sys;
    uint32_t addr;

    struct bank_info *bi;
#ifdef MX_EEPROM_PC_PROTECTION
    /* Loop to check each bank */
    for (bank = 0; bank < MX_EEPROMS; bank++) {
        bi = &p_ctrl->bi[bank];
        bi->block = 0;
        bi->block_offset = bi->bank_offset;

        /* Loop to check each block */
        for (block = 0; block < MX_EEPROM_BLOCKS; block++) {
            for (sector = 0; sector < MX_EEPROM_SECTORS_PER_CLUSTER; sector++) {
                if (!mx_ee_read_sys(p_ctrl, bi, sector, SYS_ENTRY_ADDR_E_E, &sys)) {
                    if (sys.ops == OPS_ERASE_END) {
                        if (!mx_ee_read_sys(p_ctrl, bi, sector, SYS_ENTRY_ADDR_E_B, &sys)) {
                            if (sys.ops == DATA_NONE16) {
                                formatted = true;
                            } else {    /* have OPS_ERASE_BEGIN */
                                goto erase;
                            }
                        }
                    } else {
                        if (formatted != true) { //this is the first sector
                            if (!mx_ee_read_sys(p_ctrl, bi, sector + 1, SYS_ENTRY_ADDR_E_E, &sys)) {// read the next sector system entry
                                if (sys.ops == OPS_ERASE_END) { // the first sector is power off when erasing
                                    formatted = true;
                                    goto erase;
                                } else { // two sector system entry are 0xFF means this block is not formatted
                                    break;
                                }
                            }
                        }
    erase:
                        bi->dirty_block = bi->block;
                        bi->dirty_sector = sector;
                        mx_ee_erase(p_ctrl, bi);
                        break;
                    }
                }
            }
            /* Next block */
            bi->block++;
            bi->block_offset += MX_EEPROM_CLUSTER_OFFSET;
        }

        /* Clean up */
        bi->block = DATA_NONE32;
        bi->block_offset = DATA_NONE32;
        bi->dirty_block = DATA_NONE32;
        bi->dirty_sector = DATA_NONE32;
    }
#else
    /* Loop to check each bank */
    for (bank = 0; bank < MX_EEPROMS; bank++) {
        addr = bank_offset[bank] ;
        bi = &p_ctrl->bi[bank];
        bi->block = 0;
        bi->block_offset = bi->bank_offset;

        /* Loop to check each block */
        for (block = 0; block < MX_EEPROM_BLOCKS; block++) {
            for (sector = 0; sector < MX_EEPROM_SECTORS_PER_CLUSTER; sector++) {
                if (mx_ee_read_sys(p_ctrl, bi, sector, SYS_ENTRY_ADDR_E_E, &sys)) {
                    mx_err("mxee_formt: fail to read addr 0x%08lx\r\n", addr);
                    continue;
                }

                /* Check entry format */
                if ((sys.id == MFTL_ID) && (sys.cksum == (sys.id ^ sys.ops ^ sys.arg))) {
                    formatted = true;
                    break;
                }
                addr += MX_EEPROM_SECTOR_OFFSET;
              }
            addr += MX_EEPROM_CLUSTER_OFFSET;
          }

          if (formatted)
            break;
    }
    bi->block = DATA_NONE32;
#endif

    return formatted ? FSP_SUCCESS : FSP_ERR_INVALID_STATE;
}
#else
static fsp_err_t mx_ee_check_sys(mxic_vee_ctrl_t * const p_api_ctrl)
{
  mxic_vee_instance_ctrl_t * const p_ctrl = (mxic_vee_instance_ctrl_t *) p_api_ctrl;
  uint32_t bank, block;
  bool formatted = false;
  struct system_entry sys;

#ifdef MX_EEPROM_PC_PROTECTION
  struct bank_info *bi;
  uint32_t entry, lowerBound, upperBound;

  /* Loop to check each bank */
  for (bank = 0; bank < MX_EEPROMS; bank++)
  {
    bi = &p_ctrl->bi[bank];
    bi->block = 0;
    bi->block_offset = bi->bank_offset;

    /* Loop to check each block */
    for (block = 0; block < MX_EEPROM_BLOCKS; block++)
    {
      lowerBound = 0;
      upperBound = MX_EEPROM_SYSTEM_ENTRIES - 1;

      while (lowerBound < upperBound)
      {
        /* Select the middle entry */
        entry = (lowerBound + upperBound) / 2;
        if (entry == lowerBound)
          entry++;

retry:
        /* Read system entry */
        if (mx_ee_read_sys(p_ctrl, bi, entry, &sys))
        {
          mx_err("mxee_cksys: fail to read system entry %lu\r\n", entry);
          goto err;
        }

        /* Check entry format */
        if (sys.id == MFTL_ID)
        {
          /* Valid entry */
          formatted = true;
          lowerBound = entry;
          continue;
        }
        else if (sys.ops == DATA_NONE16 && sys.arg == DATA_NONE16)
        {
          /* Empty entry */
          upperBound = entry - 1;
          continue;
        }

err:
        mx_err("mxee_cksys: corrupted system entry %lu\r\n", entry);

        /* XXX: Potential risk? Try sequential search? */
        if (entry < upperBound)
        {
          entry++;
          goto retry;
        }

        lowerBound = upperBound;
      }

      /* Read the latest entry */
      bi->sys_entry[block] = lowerBound;
      if (!mx_ee_read_sys(p_ctrl, bi, lowerBound, &sys))
      {
        if (sys.id == MFTL_ID)
          formatted = true;

        /* Check insufficient erase */
        if (sys.ops == OPS_ERASE_BEGIN)
          mx_ee_check_erase(p_ctrl, bi, sys.arg);
      }

      /* Next block */
      bi->block++;
      bi->block_offset += MX_EEPROM_CLUSTER_SIZE;
    }

    /* Clean up */
    bi->block = DATA_NONE32;
    bi->block_offset = DATA_NONE32;
    bi->dirty_block = DATA_NONE32;
    bi->dirty_sector = DATA_NONE32;
  }
#else
  uint32_t addr;

  /* Loop to check each bank */
  for (bank = 0; bank < MX_EEPROMS; bank++)
  {
    addr = bank_offset[bank] + MX_EEPROM_SYSTEM_SECTOR_OFFSET;

    /* Loop to check each block */
    for (block = 0; block < MX_EEPROM_BLOCKS; block++)
    {
#ifdef OSPI_CONTROL
       if(!(memcpy((uint8_t *)&sys, (unsigned char *)(addr + BSP_FEATURE_OSPI_DEVICE_0_START_ADDRESS), sizeof(sys))))
#endif
#ifdef QSPI_CONTROL
       if(!(memcpy((uint8_t *)&sys, (unsigned char *)(addr + QSPI_DEVICE_START_ADDRESS), sizeof(sys))))
#endif
        {
        mx_err("mxee_formt: fail to read addr 0x%08lx\r\n", addr);
        continue;
      }

      /* Check entry format */
      if ((sys.id == MFTL_ID) && (sys.cksum == (sys.id ^ sys.ops ^ sys.arg)))
      {
        formatted = true;
        break;
      }

      addr += MX_EEPROM_CLUSTER_SIZE;
    }

    if (formatted)
      break;
  }
#endif

  return formatted ? FSP_SUCCESS : FSP_ERR_INVALID_STATE;
}
#endif

#ifdef SPINAND
/**
  * @brief  Check system info and handle power cycling.
  * @retval Status
  */
static fsp_err_t mx_ee_check_gc_power_fail(mxic_vee_ctrl_t * const p_api_ctrl)
{
    mxic_vee_instance_ctrl_t * const p_ctrl = (mxic_vee_instance_ctrl_t *) p_api_ctrl;
    uint8_t bank, block, sector, des_sector;
    struct system_entry sys;

#ifdef MX_EEPROM_PC_PROTECTION
    struct bank_info *bi;

    /* Loop to check each bank */
    for (bank = 0; bank < MX_EEPROMS; bank++) {
        bi = &p_ctrl->bi[bank];
        bi->block = 0;
        bi->block_offset = bi->bank_offset;

        /* Loop to check each block */
        for (block = 0; block < MX_EEPROM_BLOCKS; block++) {
            for (sector = 0; sector < MX_EEPROM_SECTORS_PER_CLUSTER; sector++) {
                if (!mx_ee_read_sys(p_ctrl, bi, sector, SYS_ENTRY_ADDR_G_B_S, &sys)) {
                    if (sys.ops == OPS_GC_CP_BEGIN) {
                        des_sector = sys.arg;
                        if (!mx_ee_read_sys(p_ctrl, bi, sector, SYS_ENTRY_ADDR_G_E, &sys)) {
                            if (sys.ops == OPS_GC_CP_END) { /* power fail before marking OPS_ERASE_BEGIN */
                                bi->dirty_block = bi->block;
                                bi->dirty_sector = sector;
                                mx_ee_erase(p_ctrl, bi);
                            } else {    /* power fail before marking OPS_ERASE_BEGIN */
                                bi->dirty_block = bi->block;
                                bi->dirty_sector = des_sector;
                                mx_ee_erase(p_ctrl, bi); /* erase destination sector */
                                mx_ee_gc_cp(p_ctrl, bi, sector, des_sector);               /* copy data */
                                bi->dirty_block = bi->block;
                                bi->dirty_sector = sector;
                                mx_ee_erase(p_ctrl, bi);    /* erase source sector */
                            }
                        }
                        break;
                    }
                }
            }

            /* Next block */
            bi->block++;
            bi->block_offset += MX_EEPROM_CLUSTER_OFFSET;
        }

        /* Clean up */
        bi->block = DATA_NONE32;
        bi->block_offset = DATA_NONE32;
        bi->dirty_block = DATA_NONE32;
        bi->dirty_sector = DATA_NONE32;
    }
#endif
    return FSP_SUCCESS;
}
#endif

/**
  * @brief  Open virtual EEPROM Module.
  * @retval Status
  */
fsp_err_t RM_VEE_Open(mxic_vee_ctrl_t * const p_api_ctrl, mxic_vee_cfg_t const * const p_cfg)
{
    mxic_vee_instance_ctrl_t * const p_ctrl = (mxic_vee_instance_ctrl_t *) p_api_ctrl;

    p_ctrl->p_cfg = p_cfg;

    p_ctrl->open  = vEE_OPEN;
    return FSP_SUCCESS;
}

/**
  * @brief  Format EEPROM Emulator.
  * @retval Status
  */
#ifdef SPINAND
fsp_err_t RM_VEE_Format(mxic_vee_ctrl_t * const p_api_ctrl)
{
  mxic_vee_instance_ctrl_t * const p_ctrl = (mxic_vee_instance_ctrl_t *) p_api_ctrl;
  mxic_vee_ext_cfg_t const * p_extend = p_ctrl->p_cfg->p_extend;
  spi_flash_instance_t const * p_controller = p_extend->p_controller;

  struct system_entry sys;
  uint32_t i, j, k, sector, addr;

  if(!(p_ctrl->open))
    return FSP_ERR_NOT_OPEN;

  /* Should not format EEPROM after init */
  if (p_ctrl->init)
    return FSP_ERR_INVALID_CALL;

  if(p_ctrl->Mxic.Id == 0)
      BSP_SPI_Nand_Init(&(p_ctrl->Mxic), p_controller);

  /* Loop to erase each sector */
  for (i = 0; i < MX_EEPROMS; i++)
  {
    addr = bank_offset[i];

    for (j = 0; j < MX_EEPROM_BLOCKS; j++)
    {
      sector = addr;

      for (k = 0; k < MX_EEPROM_SECTORS_PER_CLUSTER; k++, sector += MX_EEPROM_SECTOR_OFFSET)
      {
        if (BSP_SPI_Erase_Block(&(p_ctrl->Mxic), sector, 1))
        {
          mx_err("mxee_formt: fail to erase sector %lu\r\n", sector);
          return FSP_ERR_ERASE_FAILED;
        }
      }

      addr += MX_EEPROM_CLUSTER_OFFSET;
    }
  }

  /* Fill system entry */
  sys.id = MFTL_ID;
  sys.ops = OPS_ERASE_END;
  sys.arg = DATA_NONE16;
  sys.cksum = sys.id ^ sys.ops ^ sys.arg;

  /* Write EE ID (Actually no need to write every block) */
  for (i = 0; i < MX_EEPROMS; i++)
  {
    addr = bank_offset[i];

    for (j = 0; j < MX_EEPROM_BLOCKS; j++)
    {
        for (sector = 0; sector < MX_EEPROM_SECTORS_PER_CLUSTER; sector++)
        {
          if (BSP_SPI_Write(&(p_ctrl->Mxic), addr, sizeof(sys), (uint8_t *)&sys))
          {
            mx_err("mxee_formt: fail to write addr 0x%08lx\r\n", addr);
            return FSP_ERR_WRITE_FAILED;
          }

          addr += MX_EEPROM_SECTOR_OFFSET;
        }

      addr += MX_EEPROM_CLUSTER_OFFSET;
    }
  }

  return FSP_SUCCESS;
}
#else
fsp_err_t RM_VEE_Format(mxic_vee_ctrl_t * const p_api_ctrl)
{
  mxic_vee_instance_ctrl_t * const p_ctrl = (mxic_vee_instance_ctrl_t *) p_api_ctrl;

#if defined(QSPI_CONTROL) || defined(OSPI_CONTROL)
  mxic_vee_ext_cfg_t const * p_extend = p_ctrl->p_cfg->p_extend;
  spi_flash_instance_t const * p_controller = p_extend->p_controller;
#endif

  struct system_entry sys;
  uint32_t i, j, k, sector, addr;

  if(!(p_ctrl->open))
    return FSP_ERR_NOT_OPEN;

  /* Should not format EEPROM after init */
  if (p_ctrl->init)
    return FSP_ERR_INVALID_CALL;

  /* Loop to erase each sector */
  for (i = 0; i < MX_EEPROMS; i++)
  {
    addr = bank_offset[i];

    for (j = 0; j < MX_EEPROM_BLOCKS; j++)
    {
      sector = addr / MX_FLASH_SECTOR_SIZE;

      for (k = 0; k < MX_EEPROM_SECTORS_PER_CLUSTER; k++, sector++)
      {
#ifdef OSPI_CONTROL
        if(R_OSPI_Erase(p_controller->p_ctrl, (unsigned char *)(BSP_FEATURE_OSPI_DEVICE_0_START_ADDRESS + sector*MX_FLASH_SECTOR_SIZE), 4096))
#endif
#ifdef QSPI_CONTROL
        if(R_QSPI_Erase(p_controller->p_ctrl, (unsigned char *)(QSPI_DEVICE_START_ADDRESS + sector*MX_FLASH_SECTOR_SIZE), 4096))
#endif
          {
          mx_err("mxee_formt: fail to erase sector %lu\r\n", sector);
          return FSP_ERR_ERASE_FAILED;
          }
        get_flash_status(p_ctrl);
      }

      addr += MX_EEPROM_CLUSTER_SIZE;
    }
  }

  /* Fill system entry */
  sys.id = MFTL_ID;
  sys.ops = OPS_NONE;
  sys.arg = DATA_NONE16;
  sys.cksum = sys.id ^ sys.ops ^ sys.arg;

  /* Write RWWEE ID (Actually no need to write every block) */
  for (i = 0; i < MX_EEPROMS; i++)
  {
    addr = bank_offset[i] + MX_EEPROM_SYSTEM_SECTOR_OFFSET;

    for (j = 0; j < MX_EEPROM_BLOCKS; j++)
    {
#ifdef OSPI_CONTROL
      if(R_OSPI_Write(p_controller->p_ctrl, (uint8_t *)&sys, (uint8_t *)(addr + BSP_FEATURE_OSPI_DEVICE_0_START_ADDRESS), sizeof(sys)))
#endif
#ifdef QSPI_CONTROL
      if(R_QSPI_Write(p_controller->p_ctrl, (uint8_t *)&sys, (uint8_t *)(addr + QSPI_DEVICE_START_ADDRESS), sizeof(sys)))
#endif
        {
        mx_err("mxee_formt: fail to write addr 0x%08lx\r\n", addr);
        return FSP_ERR_WRITE_FAILED;
      }
      get_flash_status(p_ctrl);
      addr += MX_EEPROM_CLUSTER_SIZE;
    }
  }

  return FSP_SUCCESS;
}
#endif

/**
  * @brief  Handle insufficient sector erase.
  * @param  param: eeprom_param structure pointer
  */
void RM_VEE_GetParam(struct eeprom_param *param)
{
  param->eeprom_page_size = MX_EEPROM_PAGE_SIZE;
  param->eeprom_bank_size = MX_EEPROM_BANK_SIZE;
  param->eeprom_banks = MX_EEPROMS;
  param->eeprom_total_size = MX_EEPROM_TOTAL_SIZE ;
  param->eeprom_hash_algorithm = MX_EEPROM_HASH_AlGORITHM;
}

/**
  * @brief  Initialize EEPROM Emulator.
  * @retval Status
  */
fsp_err_t RM_VEE_Init(mxic_vee_ctrl_t * const p_api_ctrl)
{

  mxic_vee_instance_ctrl_t * const p_ctrl = (mxic_vee_instance_ctrl_t *) p_api_ctrl;
  int ret;
  uint32_t bank, ofs;

  if(!(p_ctrl->open))
    return FSP_ERR_NOT_OPEN;

  if(p_ctrl->init)
    return FSP_SUCCESS;

#ifdef SPINAND
  mxic_vee_ext_cfg_t const * p_extend = p_ctrl->p_cfg->p_extend;
  spi_flash_instance_t const * p_controller = p_extend->p_controller;
  if(p_ctrl->Mxic.Id == 0)
      BSP_SPI_Nand_Init(&(p_ctrl->Mxic), p_controller);
#endif

  for (bank = 0; bank < MX_EEPROMS; bank++)
  {
  /* Check address validity */
#ifdef MX_FLASH_SUPPORT_RWW
    ofs = bank_offset[bank];
    if ((ofs / MX_FLASH_BANK_SIZE != bank) ||
       ((ofs + MX_EEPROM_BANK_SIZE - 1) / MX_FLASH_BANK_SIZE != bank))
    {
      mx_err("mxee_init : invalid bank%lu offset\r\n", bank);
      ret = FSP_ERR_INVALID_ARGUMENT;
      goto err0;
    }
#else
    for (ofs = bank + 1; ofs < MX_EEPROMS; ofs++)
    {
      if (abs(bank_offset[ofs] - bank_offset[bank]) < MX_EEPROM_BANK_SIZE)
      {
        mx_err("mxee_init : offset conflict: bank%lu, bank %lu\r\n", bank, ofs);
        ret = FSP_ERR_INVALID_ARGUMENT;
        goto err0;
      }
    }
#endif

    /* Init bank info */
    p_ctrl->bi[bank].bank = bank;
    p_ctrl->bi[bank].bank_offset = bank_offset[bank];

    /* Empty current block info */
    p_ctrl->bi[bank].block = DATA_NONE32;
    p_ctrl->bi[bank].block_offset = DATA_NONE32;

    /* Empty current page cache */
    p_ctrl->bi[bank].cache_dirty = false;
    memset(&p_ctrl->bi[bank].cache, DATA_NONE8, MX_EEPROM_ENTRY_SIZE);

    /* No obsoleted sector */
    p_ctrl->bi[bank].dirty_block = DATA_NONE32;
    p_ctrl->bi[bank].dirty_sector = DATA_NONE32;

    /* Init bank mutex lock */
    p_ctrl->bi[bank].lock = xSemaphoreCreateMutex();
    if (!p_ctrl->bi[bank].lock)
    {
      mx_err("mxee_init : out of memory (bankLock)\r\n");
      ret = FSP_ERR_OUT_OF_MEMORY;
      goto err1;
    }

#ifdef MX_DEBUG
    /* Reset erase count statistics */
    memset(p_ctrl->bi[bank].eraseCnt, 0, sizeof(p_ctrl->bi[bank].eraseCnt));
#endif
  }

  /* Reset R/W counter */
  p_ctrl->rwCnt = 0;

#ifdef MX_EEPROM_CRC_HW
  /* Init HW CRC */
  mxic_vee_ext_cfg_t const * p_crc_extend = p_ctrl->p_cfg->p_extend;
  crc_instance_t const * p_crc = p_crc_extend->p_crc;

  if ( R_CRC_Open(p_crc->p_ctrl, p_crc->p_cfg))
  {
    mx_err("mxee_init : fail to init HW CRC\r\n");
    ret = FSP_ERR_NOT_FOUND;
    goto err2;
  }

  /* Init CRC mutex lock */
  p_ctrl->crcLock = xSemaphoreCreateMutex();
  if (!p_ctrl->crcLock)
  {
    mx_err("mxee_init : out of memory (crcLock)\r\n");
    ret = FSP_ERR_OUT_OF_MEMORY;
    goto err2;
  }
#endif

  /* Init RWW layer */
  ret = mx_ee_rww_init(p_ctrl);
  if (ret)
  {
    mx_err("mxee_init : fail to init RWW layer\r\n");
    goto err3;
  }

  /* Check RWWEE format */
  ret = mx_ee_check_sys(p_ctrl);
  if (ret)
  {
    mx_err("mxee_init : not found valid RWWEE format\r\n");
    goto err4;
  }

#ifdef SPINAND
  /* Check if power fail during gc */
  ret = mx_ee_check_gc_power_fail(p_ctrl);
#endif

#ifdef MX_EEPROM_BACKGROUND_THREAD
  /* Start background thread */
  mx_info("mxee_init : starting background thread\r\n");
  p_ctrl->bgStart = true;

  xTaskCreate((TaskFunction_t)RM_VEE_Background, (const portCHAR *)"eeback", (uint32_t)MX_EEPROM_BG_THREAD_STACK_SIZE, p_ctrl, (portBASE_TYPE)MX_EEPROM_BG_THREAD_PRIORITY, &p_ctrl->bgThreadID);

  if (!p_ctrl->bgThreadID)
  {
    mx_err("mxee_init : fail to start background thread\r\n");
    p_ctrl->bgStart = false;
  }
#endif

  /* Init done */
  p_ctrl->init = true;

  return FSP_SUCCESS;

err4:
  mx_ee_rww_deinit(p_ctrl);
err3:
#ifdef MX_EEPROM_CRC_HW
  vQueueDelete(p_ctrl->crcLock);
  p_ctrl->crcLock = NULL;
err2:
R_CRC_Close(p_crc->p_ctrl);
#endif
err1:
  for (bank-- ; bank < MX_EEPROMS; bank--)
  {
    vQueueDelete(p_ctrl->bi[bank].lock);
    p_ctrl->bi[bank].lock = NULL;
  }
err0:
  return ret;
}

/**
  * @brief  Deinit EEPROM Emulator.
  */
void RM_VEE_Close(mxic_vee_ctrl_t * const p_api_ctrl)
{
  mxic_vee_instance_ctrl_t * const p_ctrl = (mxic_vee_instance_ctrl_t *) p_api_ctrl;
  uint32_t cnt;

  if (!p_ctrl->init)
    return;

  /* Block further user request */
  p_ctrl->init = false;

#ifdef MX_EEPROM_BACKGROUND_THREAD
  /* Stop background thread */
  if (p_ctrl->bgStart)
  {
    mx_info("mxee_deini: stopping the background thread\r\n");

    /* Terminate the background thread */
    p_ctrl->bgStart = false;
    cnt = xTaskGetTickCount();
    while (eTaskGetState(p_ctrl->bgThreadID) != eDeleted)
    {
      if (xTaskGetTickCount() - cnt > MX_EEPROM_BG_THREAD_TIMEOUT)
      {
        mx_err("mxee_deini: killing the background thread\r\n");
        vTaskDelete(p_ctrl->bgThreadID);
        break;
      }

      vTaskDelay((1 / portTICK_PERIOD_MS));
    }

    p_ctrl->bgThreadID = NULL;
  }
#endif

  for (cnt = 0; cnt < MX_EEPROMS; cnt++)
  {
    /* Wait for the current request to finish */
    xSemaphoreTake(p_ctrl->bi[cnt].lock, portMAX_DELAY);
    /* Delete bank mutex lock */
    vQueueDelete(p_ctrl->bi[cnt].lock);
    p_ctrl->bi[cnt].lock = NULL;
  }

#ifdef MX_EEPROM_CRC_HW
  /* Deinit HW CRC */
  mxic_vee_ext_cfg_t const * p_crc_extend = p_ctrl->p_cfg->p_extend;
  crc_instance_t const * p_crc = p_crc_extend->p_crc;
  R_CRC_Close(p_crc->p_ctrl);
  /* Delete CRC mutex lock */
  vQueueDelete(p_ctrl->crcLock);
  p_ctrl->crcLock = NULL;
#endif

  /* Cleanup RWW layer */
  mx_ee_rww_deinit(p_ctrl);
}
