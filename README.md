## Overview

Macronix has updated the FSP module packs including Power Management, VEE, MX25R Extend and LittleFS driver to v5.0.0. User can get the source code from here and Macronix official website. 

**Power Manage in FSP**
**Hardware and Software Requirement**

1. Renesas FSP V5.0.0 or later
2. Renesas RA Series Development Board 
3. Nor Flash

**Introduction**

Usually, Nor Flash has more than one power mode, such as Normal mode and DeepPowerDown, but in FSP, it is not very easy to change Flash power mode, In this application, we introduce a middleware to manage Flash Power mode, and take MX25R as an example for interpretation. The Power Manage Middleware just offers some interfaces and the Flash vender should realize the interfaces according to related rules.

power_init () 

setNormalMode () 

setHighPerformanceMode () 

setLowPowerMode () 

setSuperLowPowerMode ()

 setDeepPowerDownMode ()

Each vender should realize the interfaces above. power_init() is used to do some initialization work, and then vender should select some power mode according to actual Flash. Finally, the other power should return NOT_SUPPORT.

**MX25R Extend in FSP
Hardware and Software Requirement**

1. Renesas FSP V5.0.0 or later
2. Renesas RA Series Development Board 
3. MX25R Series Flash

**Introduction**

Macronix MX25R series nor Flash has 3 power modes: HighPerformance Mode, LowPower Mode and DeepPowerDown Mode. However, if you want to control MX25R flash in Renesas FSP platform, you can not easily change the power mode.

RDID()
RDSR()
RDSCUR()
RDCR()

MX25R_HighPerformanceMode()

MX25R_LowPowerMode()

MX25R_Enter_DeepPower()

MX25R_Exit_DeepPower()

MX25R_Device_Reset()

**MXIC vEE Middleware**

**Hardware and Software Requirement**

1. Renesas FSP v5.0.0 or later
2. Renesas EK-RA6M3/EK-RA6M4 Board 
3. MXIC Nor Flash and SPI Nand Flash

**Introduction**

1. MXIC vEE(virtual EEPROM Emulation) propose an algorithm and system architecture for implementing the EEPROM emulation on a Read-While-Write flash memory. It support to run read and write/erase operations simultaneously in multi-threaded and real-time environment. It gains about 20% increase on overall read/write performance.
2. MXIC vEE can be built upon both QSPI and OSPI driver. And if you choose QSPI, you can use SPI Nand Flash.

**LittleFS MXIC Middleware**

**Hardware and Software Requirement:**

1. Renesas FSP V5.0.0 or later
2. Renesas RA Series Board
3. MXIC Nor and SPI Nand Flash

**Middleware Introduction**

1. In Renesas FSP, Users can add LittleFS to their project, and the filesystem memory is internal flash. The middleware can connect LittleFS with external MXIC nor flash.
2. This middleware can be built upon both QSPI and OSPI driver. And if you choose QSPI, you can use SPI Nand Flash as the filesystem memory.
3. When user select SPI Nand Flash in LittleFS, user can also enable NFTL, For more information about NFTL, please refer NFTL_Introduction.pdf.
4. When add NFTL to the project, user should set as large a heap and stack as possible. For example, here we set Stack = 0x8000, Heap = 0x25000. (The Stack and Heap size also depends on your application, especially Heap.)

### Related Links

Macronix FSP module packs: https://www.mxic.com.tw/en-us/support/design-support/Pages/software-support.aspx
