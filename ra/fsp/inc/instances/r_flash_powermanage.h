/*
 * powermange.h
 */

#ifndef FSP_INC_API_POWERMANGE_H_
#define FSP_INC_API_POWERMANGE_H_
#include "bsp_api.h"



typedef enum powermode {
    NormalMode          = 0,
    HighPerformanceMode = 1,
    LowPowerMode        = 2,
    SuperLowPowerMode   = 3,
    DeepPowerDown       = 4,
}powermode_t;

typedef struct powermanage
{
    fsp_err_t (*power_init)();

    fsp_err_t (*setNormalMode)();

    fsp_err_t (*setHighPerformanceMode)();

    fsp_err_t (*setLowPowerMode)();

    fsp_err_t (*setSuperLowPowerMode)();

    fsp_err_t (*setDeepPowerDownMode)();

    powermode_t currentmode;
}powermanage_t;

void Flash_Init(powermanage_t *p);
void SetFlashPowerMode(powermanage_t *p ,powermode_t mode);



#endif /* FSP_INC_API_POWERMANGE_H_ */
