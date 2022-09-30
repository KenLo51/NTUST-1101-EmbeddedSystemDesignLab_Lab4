#ifndef DMA2D_DRAW_H
#define DMA2D_DRAW_H

#include "stm32f7xx_hal.h"
#include "stm32746g_discovery_lcd.h"
#include "stm32746g_discovery_sdram.h"


void DMA2D_FillRect(uint32_t color, uint32_t x, uint32_t y, uint32_t width, uint32_t height);
void DMA2D_DrawImage(uint32_t data, uint32_t x, uint32_t y, uint32_t width, uint32_t height);

#endif
