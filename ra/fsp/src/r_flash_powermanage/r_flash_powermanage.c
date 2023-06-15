/*
 * powermanage.c
 */

#include <r_flash_powermanage.h>

void Flash_Init(powermanage_t *p)
{
    p->power_init();
    p->currentmode = NormalMode;

}

void SetFlashPowerMode(powermanage_t *p ,powermode_t mode)
{
    if(mode == NormalMode)
    {
        p->setNormalMode();
        p->currentmode = NormalMode;
    }
    if(mode == HighPerformanceMode)
    {
        p->setHighPerformanceMode();
        p->currentmode = HighPerformanceMode;
    }
    if(mode == LowPowerMode)
    {
        p->setLowPowerMode();
        p->currentmode = LowPowerMode;
    }
    if(mode == SuperLowPowerMode)
    {
        p->setSuperLowPowerMode();
        p->currentmode = SuperLowPowerMode;
    }
    if(mode == DeepPowerDown)
    {
        p->setDeepPowerDownMode();
        p->currentmode = DeepPowerDown;
    }

}









