#include "stm32f4xx.h"
#include "st7789.h"

int main(void) {
    ST7789_GPIO_Init();
    ST7789_Init();

    for (;;) {
        ST7789_Fill(0xFF);
    }
    return 0;
}
