#ifndef __USBMIDI_HPP
#define __USBMIDI_HPP

#include <stdio.h>
#include "pico/stdlib.h"

#include "tusb.h"

#define NOTE_ON (uint8_t)0x90
#define NOTE_OFF (uint8_t)0x80
#define CONTROL_CHANGE (uint8_t)0xB0
#define PITCHBEND (uint8_t)0xE0

class USBMIDI {
    public:
        USBMIDI(int8_t channel = 0);
        void begin();
        void noteOn(uint8_t note, uint8_t value, int8_t channel = -1);
        void noteOff(uint8_t note, uint8_t value, int8_t channel = -1);
        void controlChange(uint8_t control, uint8_t value, int8_t channel = -1);
        int8_t setChannel(int8_t channel);
    private:
        int8_t mChannel;
};


#endif //USBMIDI_HPP