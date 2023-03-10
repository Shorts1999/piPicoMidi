#ifndef __WS2812B_HPP
#define __WS2812B_HPP

#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/pio_instructions.h"

#include "ws2812.pio.h"

class WS2812B{
    public:
        WS2812B(uint pin, PIO pio=pio0, bool rgbw=false);
};


#endif //#ifndef __WS2812B_HPP