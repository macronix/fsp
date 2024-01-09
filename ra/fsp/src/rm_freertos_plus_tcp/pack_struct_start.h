#include "cmsis_compiler.h"

#if defined(__GNUC__)
 #include \
    "../../../../ra/aws/FreeRTOS/FreeRTOS-Plus/Source/FreeRTOS-Plus-TCP/source/portable/Compiler/GCC/pack_struct_start.h"
#elif defined(__IAR_SYSTEMS_ICC__)

/* Ignore Pe1644 error  (definition at end of file not followed by a semicolon or a declarator). */
 #pragma diag_suppress=Pe1644
 #include \
    "../../../../ra/aws/FreeRTOS/FreeRTOS-Plus/Source/FreeRTOS-Plus-TCP/source/portable/Compiler/IAR/pack_struct_start.h"
#elif defined(__CC_ARM)
 #include \
    "../../../../ra/aws/FreeRTOS/FreeRTOS-Plus/Source/FreeRTOS-Plus-TCP/source/portable/Compiler/Keil/pack_struct_start.h"
#else
 #error Unknown compiler.
#endif
