
#ifndef MX31XX_SPI_NAND_APP_H_
#define MX31XX_SPI_NAND_APP_H_
#include "../spi_nand_driver/spi_nand_ops.h"

int BSP_SPI_Read(MxChip *Mxic, u32 Addr, u32 ByteCount, u8 *Buf);
int BSP_SPI_Read_Conti(MxChip *Mxic, u32 Addr, u32 ByteCount, u8 *Buf);
int BSP_SPI_ReadOob(MxChip *Mxic, u32 Addr, u32 ByteCount, u8 *Buf);
int BSP_SPI_Write(MxChip *Mxic, u32 Addr, u32 ByteCount, u8 *Buf);
int BSP_SPI_WriteOob(MxChip *Mxic, u32 Addr, u32 ByteCount, u8 *Buf);
int BSP_SPI_Erase_Block(MxChip *Mxic, u32 Addr, u32 EraseSizeCount);
int BSP_SPI_Lock_Flash(MxChip *Mxic, u8 Tight);
int BSP_SPI_Unlock_Flash(MxChip *Mxic, u32 Addr, u64 Len, u8 Sp);
int BSP_SPI_Is_Flash_Locked(MxChip *Mxic, u32 Addr);
int BSP_SPI_Block_Is_Bad(MxChip *Mxic, u32 Addr);
int BSP_SPI_Block_Mark_Bad(MxChip *Mxic, u32 Addr);
int BSP_SPI_Nand_Init(MxChip *Mxic, const spi_flash_instance_t * controller);

#endif /* MX31XX_SPI_NAND_APP_H_ */
