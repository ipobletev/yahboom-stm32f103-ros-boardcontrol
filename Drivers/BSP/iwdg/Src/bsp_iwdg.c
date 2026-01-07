#include "bsp_iwdg.h"
#include "iwdg.h"

void bsp_iwdg_init(void) {
    // MX_IWDG_Init is already called in main.c, but we can call it here if needed
    // or just assume it was initialized. For better encapsulation, we could
    // move initialization logic here, but keeping it simple for now.
    MX_IWDG_Init();
}

void bsp_iwdg_refresh(void) {
    HAL_IWDG_Refresh(&hiwdg);
}
