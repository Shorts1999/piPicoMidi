/**
 * @file WS2812B.cpp
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 10-03-2023
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "WS2812B.hpp"

WS2812B::WS2812B(uint pin, PIO pio, bool rgbw) {
    // todo get free sm
    PIO pio = pio0;
    int sm = 0;
    uint offset = pio_add_program(pio, &ws2812_program);

    ws2812_program_init(pio, sm, offset, cRGBLedPin, 800000, false);
}
