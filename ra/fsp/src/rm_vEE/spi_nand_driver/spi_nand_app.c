
#include "../spi_nand_driver/spi_nand_app.h"

/*
 * Function:      BSP_SPI_Read
 * Arguments:     Mxic:      pointer to an mxchip structure of SPI NAND device.
 *                Addr:      device address to read
 *                ByteCount: number of bytes to read
 *                Buf:       pointer to a data buffer where the read data will be stored
 * Return Value:  MXST_SUCCESS
 *                MXST_FAILURE
 * Description:   This function issues the Read commands to SPI Flash and reads data from the array.
 *                Data size is specified by ByteCount.
 */
int BSP_SPI_Read(MxChip *Mxic, u32 Addr, u32 ByteCount, u8 *Buf)
{
    return Mx_Nand_Read_Sub_Page_Raw(Mxic, Addr, ByteCount, Buf);
}

int BSP_SPI_Read_Conti(MxChip *Mxic, u32 Addr, u32 ByteCount, u8 *Buf)
{
    int Status;

    u8 StatusReg1;
    StatusReg1 = 0x04;
    Status = Mx_SET_FEATURE(Mxic, FEATURES_ADDR_SECURE_OTP, &StatusReg1 );
    if (Status != MXST_SUCCESS)
          return Status;

    Status = Mx_READ(Mxic, Addr );
    if (Status == MXST_FAILURE)
        return Status;
    Status = Mxic->AppGrp._ReadFromCache(Mxic, 0, ByteCount, Buf, 0);
    if (Status == MXST_FAILURE)
        return Status;

    Mx_RESET(Mxic);

    StatusReg1 = 0x00;
    Status = Mx_SET_FEATURE(Mxic, FEATURES_ADDR_SECURE_OTP, &StatusReg1 );
    if (Status != MXST_SUCCESS)
          return Status;

     return Status;
}

/*
 * Function:      BSP_SPI_Write
 * Arguments:     Mxic:      pointer to an mxchip structure of SPI NAND device.
 *                Addr:      device address to program
 *                ByteCount: number of bytes to program
 *                Buf:       pointer to a data buffer where the program data will be stored
 * Return Value:  MXST_SUCCESS
 *                MXST_FAILURE
 *                MXST_TIMEOUT
 * Description:   This function programs location to the specified data.
 */
int BSP_SPI_Write(MxChip *Mxic, u32 Addr, u32 ByteCount, u8 *Buf)
{
    return Mx_Nand_Write_Sub_Page_Raw(Mxic, Addr, ByteCount, Buf);
}

/*
 * Function:      BSP_SPI_ReadOob
 * Arguments:     Mxic:      pointer to an mxchip structure of SPI NAND device.
 *                Addr:      device address to read
 *                ops:       oob operation description structure
 * Return Value:  MXST_SUCCESS
 *                MXST_FAILURE
 * Description:   This function readS data and/or out-of-band.
 */
int BSP_SPI_ReadOob(MxChip *Mxic, u32 Addr, u32 ByteCount, u8 *Buf)
{
    int Status;
    struct Mtd_Oob_Ops Ops;
    memset(&Ops, 0, sizeof(Ops));
    Ops.OobLen = ByteCount;
    Ops.OobBuf = Buf;
    Ops.Mode = MTD_OPS_PLACE_OOB;
    Status = Mx_Nand_Do_Read_Oob(Mxic, Addr, &Ops);
    if (Status != MXST_SUCCESS)
        return MXST_FAILURE;

    return MXST_SUCCESS;
}

/*
 * Function:      BSP_SPI_WriteOob
 * Arguments:     Mxic:      pointer to an mxchip structure of SPI NAND device.
 *                Addr:      device address to program
 *                ops:       oob operation description structure
 * Return Value:  MXST_SUCCESS
 *                MXST_FAILURE
 *                MXST_TIMEOUT
 * Description:   This function writes data and/or out-of-band.
 */
int BSP_SPI_WriteOob(MxChip *Mxic, u32 Addr, u32 ByteCount, u8 *Buf)
{
    int Status;
    struct Mtd_Oob_Ops Ops;

    memset(&Ops, 0, sizeof(Ops));
    Ops.OobLen = ByteCount;
    Ops.OobBuf = Buf;
    Ops.Mode = MTD_OPS_PLACE_OOB;
    Status =  Mx_Nand_Do_Write_Oob(Mxic, Addr, &Ops);
    if (Status != MXST_SUCCESS)
        return Status;

    return MXST_SUCCESS;
}

/*
 * Function:      BSP_SPI_Erase_Block
 * Arguments:     Mxic:           pointer to an mxchip structure of SPI NAND device.
 *                Addr:           device address to erase
 *                EraseSizeCount: number of block or sector to erase
 * Return Value:  MXST_SUCCESS
 *                MXST_FAILURE
 *                MXST_TIMEOUT
 * Description:   This function erases the data in the specified Block .
 *                Function issues all required commands and polls for completion.
 */
int BSP_SPI_Erase_Block(MxChip *Mxic, u32 Addr, u32 EraseSizeCount)
{
    int Status;
    u32 n;
    for (n = 0; n < EraseSizeCount; n++)
    {
        Status = Mx_BE(Mxic, Addr + n * BLOCK_OFFSET);
        if (Status != MXST_SUCCESS)
            return Status;
    }
    return MXST_SUCCESS;
}

/*
 * Function:      BSP_SPI_Lock_Flash
 * Arguments:     Mxic,  pointer to an mxchip structure of SPI NAND device.
 *                Tight, choose tight lock enable or disable,
 *                       LOCK_TIGHT_ENABLE:BPRWD bit is 1;LOCK_TIGHT_DISABLE:BPRWD bit is 0
 * Return Value:  MXST_SUCCESS
 *                MXST_FAILURE
 * Description:   This function is for writing protection all blocks of flash memory.
 */
int BSP_SPI_Lock_Flash(MxChip *Mxic, u8 Tight)
{
    int Status;
    u8 FeatureBpOld, FeatureBpNew;

    /* check the Lock Tight Status */
    Status = Mx_GET_FEATURE(Mxic, FEATURES_ADDR_BLOCK_PROTECTION, &FeatureBpOld);
    if (Status != MXST_SUCCESS)
        return Status;

    if(FeatureBpOld & BLOCK_PROT_BPRWD)
     {
        Mx_printf("BSP_SPI_Lock_Flash: spi_nand_lock: Device is locked tight(BPRWD bit is 1)! \r\n");
        return MXST_FAILURE;
    }
    FeatureBpNew = (Tight ? BLOCK_PROT_BPRWD | BLOCK_PROT_BP_MASK: BLOCK_PROT_BP_MASK);
    Status = Mx_SET_FEATURE(Mxic, FEATURES_ADDR_BLOCK_PROTECTION, &FeatureBpNew );
    if (Status != MXST_SUCCESS)
        return Status;

    return MXST_SUCCESS;
}

/*
 * Function:      BSP_SPI_Unlock_Flash
 * Arguments:     Mxic,  pointer to an mxchip structure of SPI NAND device.
 *                Addr,  32 bit flash memory address of unlocked area( address must be beginning of nand block)
 *                Len,   number of bytes to unlock (must be a multiple of block size)
 *                Sp,   choose solid protection or not,
 *                      BLOCK_PROT_SP_ENABLE: solid protection enable;BLOCK_PROT_SP_DISABLE:solid protection disable
 * Return Value:  MXST_SUCCESS
 *                MXST_FAILURE
 * Description:   This function will cancel the block write protection state.
 */
int BSP_SPI_Unlock_Flash(MxChip *Mxic, u32 Addr, u64 Len, u8 Sp)
{
    int Status;
    u8 FeatureBpOld, FeatureBpNew;
    u8 Mask = BLOCK_PROT_BP2 | BLOCK_PROT_BP1 | BLOCK_PROT_BP0;
    u8 Shift = ffs(Mask) - 1, Pow, Val;
    u64 Len_lock;

    if ((Addr & (Mxic->BlockSz - 1)) != 0) {
        Mx_printf("BSP_SPI_Unlock_Flash: Address must be beginning of nand block!\r\n");
        return MXST_FAILURE;
    }

    if (Len == 0 || (Len % (Mxic->BlockSz)) != 0) {
        Mx_printf("BSP_SPI_Unlock_Flash: Length must be a multiple of nand block "
                "size %08lx!\r\n", Mxic->BlockSz);
        return MXST_FAILURE;
    }

    /* chip size is more than 1G，like MX35LF2GE4AB，MX35LF4GE4AB */
    if(strstr(Mxic->Name,"ab") && (Mxic->ChipSz > 2112 * 64 * 1024))
    {
        if (Addr != 0){
            Mx_printf("BSP_SPI_Unlock_Flash: Un-lock start address is not 0...\r\n");
                return MXST_FAILURE;
            }
    }

    if ((Addr == 0) && (Len == Mxic->ChipSz))
    {
        /*
         *  BP2 | BP1 | BP0 | Invert | Complementary | Protection Area
         * ------------------------------------------------------------
         *   0  |  0  |  0  |   0    |      0        | None-all unlocked
         */
        FeatureBpNew = 0; /* fully unlocked */
    }
    else if ((Addr == Mxic->BlockSz) &&
            (Len == Mxic->ChipSz - Mxic->BlockSz))
    {
        /*
         *  BP2 | BP1 | BP0 | Invert | Complementary | Protection Area
         * ------------------------------------------------------------
         *   1  |  1  |  0  |   0    |      1        | block0 locked
         */
        FeatureBpNew = 0x32; /* fully unlocked expect block-0 */
    }
    else
    {
        /*
         * Need largest pow such that:
         *
         *   1 / (2^pow) >= (len_lock / size)
         *
         * so (assuming power-of-2 size) we do:
         *
         *   pow = floor(log2(size / len_lock)) = log2(size) - ceil(log2(len_lock))
         */
        Len_lock = Mxic->ChipSz - Len;

        if (Addr <= (Mxic->ChipSz >> 1))
        {
            /*
             *           ________
             *  Addr:FF |        | upper(top)
             *          |        |
             *          |        |
             *          |        |
             *          |        |
             *  Addr:0  |________| low(bottom)
             *
             *for 2G/4G Flash(MX35LF2GE4AB/MX35LF4GE4AB), no Invert and Complementary bit
             *  BP2 | BP1 | BP0 | Protection Area
             * ------------------------------------------------------------
             *   0  |  0  |  1  | upper 1/64 locked
             *   0  |  1  |  0  | upper 1/32 locked
             *   0  |  1  |  1  | upper 1/16 locked
             *   1  |  0  |  0  | upper 1/8 locked
             *   1  |  0  |  1  | upper 1/4 locked
             *   1  |  1  |  0  | upper 1/2 locked
             *
             *
             *for 1G Flash, only handle with Invert = 1(lower)
             *  BP2 | BP1 | BP0 | Invert | Complementary | Protection Area
             * ------------------------------------------------------------
             *   0  |  0  |  1  |   1    |      0        | lower 1/64 locked
             *   0  |  1  |  0  |   1    |      0        | lower 1/32 locked
             *   0  |  1  |  1  |   1    |      0        | lower 1/16 locked
             *   1  |  0  |  0  |   1    |      0        | lower 1/8 locked
             *   1  |  0  |  1  |   1    |      0        | lower 1/4 locked
             *   1  |  1  |  0  |   1    |      0        | lower 1/2 locked
             */

            if (!is_power_of_2(Mxic->ChipSz / Len_lock))
            {
                Mx_printf("BSP_SPI_Unlock_Flash: un-lock area didn't match upper 3/4, 7/8 and so on ...\r\n");
                return MXST_FAILURE;
            }
            Pow = log2(Mxic->ChipSz / Len_lock);
        }
        else
        {
            /*
             *  BP2 | BP1 | BP0 | Invert | Complementary | Protection Area
             * ------------------------------------------------------------
             *   0  |  0  |  1  |   0    |      1        | lower 63/64 locked / upper 1/64 unlocked
             *   0  |  1  |  0  |   0    |      1        | lower 31/32 locked / upper 1/32 unlocked
             *   0  |  1  |  1  |   0    |      1        | lower 15/16 locked / upper 1/16 unlocked
             *   1  |  0  |  0  |   0    |      1        | lower 7/8 locked   / upper 1/8 unlocked
             *   1  |  0  |  1  |   0    |      1        | lower 3/4 locked   / upper 1/4 unlocked
             */

            if (!is_power_of_2(Mxic->ChipSz / Len)) {
                Mx_printf("un-lock area didn't match upper 1/4, 1/8 and so on ...\r\n");
                return MXST_FAILURE;
            }
            Pow = log2(Mxic->ChipSz / Len);
        }

        Val = Mask - (Pow << Shift);

        /* Bottom lock mode */
        if (Addr <= (Mxic->ChipSz >> 1))
            FeatureBpNew = Val | BLOCK_PROT_INVERT;
        else
            FeatureBpNew = Val | BLOCK_PROT_COMPLE;
    }

    if (Sp)
    FeatureBpNew |= BLOCK_PROT_SP;

    Status = Mx_GET_FEATURE(Mxic, FEATURES_ADDR_BLOCK_PROTECTION, &FeatureBpOld);
    if (Status != MXST_SUCCESS)
        return Status;
    if(FeatureBpOld & BLOCK_PROT_BPRWD)
        FeatureBpNew |= BLOCK_PROT_BPRWD;

    Status = Mx_SET_FEATURE(Mxic, FEATURES_ADDR_BLOCK_PROTECTION, &FeatureBpNew );
    if (Status != MXST_SUCCESS)
        return Status;
    return MXST_SUCCESS;
}

/*
 * Function:      BSP_SPI_Is_Flash_Locked
 * Arguments:     Mxic,  pointer to an mxchip structure of SPI NAND device.
 *                Addr,  32 bit flash memory address of checking area
 * Return Value:  FLASH_IS_UNLOCKED
 *                FLASH_IS_LOCKED
 * Description:   This function is for checking if the block is locked.
 */
int BSP_SPI_Is_Flash_Locked(MxChip *Mxic, u32 Addr)
{
    int Status ;
    u8 FeatureBp;
    u32 Pow;

    if ((Addr & (Mxic->BlockSz - 1)) != 0) {
        Mx_printf("BSP_SPI_Is_Flash_Locked: Start address must be beginning of nand page!\r\n");
        return MXST_FAILURE;
    }

    Status = Mx_GET_FEATURE(Mxic, FEATURES_ADDR_BLOCK_PROTECTION, &FeatureBp);
    if (Status != MXST_SUCCESS)
        return Status;
    /* BPRWD = 1 */
    if(FeatureBp & BLOCK_PROT_BPRWD)
        return FLASH_IS_LOCKED_TIGHT;
    /* BP2 = 1,BP1 = 1,BP0 = 1 */
    else if(FeatureBp & BLOCK_PROT_BP_MASK)
        return FLASH_IS_LOCKED;
    /* BP2 = 0,BP1 = 0,BP0 = 0 */
    else if((FeatureBp & BLOCK_PROT_BP_MASK) == 0)
        return FLASH_IS_UNLOCKED;

    /*Get Locked start address */
    Pow = pow(2,((BLOCK_PROT_BP_MASK >> BLOCK_PROT_BP_OFFSET) - (FeatureBp & BLOCK_PROT_BP_MASK >> BLOCK_PROT_BP_OFFSET)));

    if(strstr(Mxic->Name,"ab"))
    {
        if(Addr >= Mxic->ChipSz - Mxic->ChipSz / Pow)
            return FLASH_IS_LOCKED;
        else
            return FLASH_IS_UNLOCKED;
    }
    else
    {
        switch(FeatureBp & (BLOCK_PROT_INVERT | BLOCK_PROT_COMPLE ))
        {
            case 0x00:
                if(Addr >= Mxic->ChipSz - Mxic->ChipSz / Pow)
                    return FLASH_IS_LOCKED;
                else
                    return FLASH_IS_UNLOCKED;

            case BLOCK_PROT_COMPLE:
                if(Addr < Mxic->ChipSz - Mxic->ChipSz / Pow)
                    return FLASH_IS_LOCKED;
                else
                    return FLASH_IS_UNLOCKED;

            case BLOCK_PROT_INVERT:
                if(Addr < Mxic->ChipSz / Pow)
                    return FLASH_IS_LOCKED;
                else
                    return FLASH_IS_UNLOCKED;

            case BLOCK_PROT_INVERT | BLOCK_PROT_COMPLE:
                if(Addr >= Mxic->ChipSz / Pow)
                    return FLASH_IS_LOCKED;
                else
                    return FLASH_IS_UNLOCKED;
            default :   return FLASH_IS_LOCKED;
        }
    }
}

/*
 * Function:      BSP_SPI_Block_Is_Bad
 * Arguments:     Mxic,  pointer to an mxchip structure of SPI NAND device.
 *                Addr,  32 bit flash memory address of checking area
 * Return Value:  MXST_BLOCK_IS_BAD
 *                MXST_BLOCK_ISNOT_BAD
 * Description:   This function is for checking if the block is bad block.
 */
int BSP_SPI_Block_Is_Bad(MxChip *Mxic, u32 Addr)
{
    return Mx_Nand_Block_Bad(Mxic, Addr);
}

/*
 * Function:      BSP_SPI_Block_Mark_Bad
 * Arguments:     Mxic,  pointer to an mxchip structure of SPI NAND device.
 *                Addr,  32 bit flash memory address of marking area
 * Return Value:  MXST_SUCCESS
 *                MXST_FAILURE
 * Description:   This function is for marking the bad block.
 */
int BSP_SPI_Block_Mark_Bad(MxChip *Mxic, u32 Addr)
{
    int Status;

    Status = BSP_SPI_Block_Is_Bad(Mxic, Addr);
    if (Status == MXST_FAILURE) {
        return MXST_FAILURE;
    }
    else if (Status == MXST_BLOCK_IS_BAD) {
        /* If it was bad already, return success and do nothing */
        return MXST_SUCCESS;
    }

    return Mx_Nand_Block_Mark_Bad(Mxic, Addr);
}

int BSP_SPI_Nand_Init(MxChip *Mxic, const spi_flash_instance_t * controller)
{
    int Status;
    memset(Mxic, 0, sizeof(MxChip));
    Status = MxSoftwareInit(Mxic, controller);
    if (Status != MXST_SUCCESS)
        return Status;

    Status = MxIdMatch(Mxic);
    if (Status != MXST_SUCCESS)
        return Status;

    Status = Mx_Nand_EccInit(Mxic, NAND_ECC_ONDIE);
    if (Status != MXST_SUCCESS)
        return Status;

    u8 st_reg1;
    Mxic->AppGrp._ReadFromCache = Mx_READ_CACHE;
    Mxic->AppGrp._WriteToCache = Mx_PP_LOAD;
    Status = Mx_GET_FEATURE(Mxic, FEATURES_ADDR_SECURE_OTP, &st_reg1);
    if (Status != MXST_SUCCESS)
        return Status;
    st_reg1 &= ~SECURE_OTP_QE;
    Status = Mx_SET_FEATURE(Mxic, FEATURES_ADDR_SECURE_OTP, &st_reg1);
    if (Status != MXST_SUCCESS)
        return Status;

    return MXST_SUCCESS;
}


