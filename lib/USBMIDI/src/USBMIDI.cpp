#include "USBMIDI.hpp"

#define MAX_CHANNEL 15

USBMIDI::USBMIDI(int8_t channel){
    //set default channel to 0 if out of range
    if(channel<0 || channel>MAX_CHANNEL) {this->mChannel=0;}
    this->mChannel = channel;
}

void USBMIDI::begin(){
    tusb_init();
}

void USBMIDI::noteOn(uint8_t note, uint8_t value, int8_t channel){
    //set channel to default if channel is out of range (or not specified)
    if(channel<0 || channel>MAX_CHANNEL){channel = this->mChannel;}

    uint8_t data[3] = {NOTE_ON | (uint8_t)channel, note, value};
    tud_midi_stream_write(0, data, sizeof(data)/sizeof(data[0]));

    return;
}

void USBMIDI::noteOff(uint8_t note, uint8_t value, int8_t channel){
    //set channel to default if channel is out of range (or not specified)
    if(channel<0 || channel>MAX_CHANNEL){channel = this->mChannel;}

    uint8_t data[3] = {NOTE_OFF | (uint8_t)channel, note, value};
    tud_midi_stream_write(0, data, sizeof(data)/sizeof(data[0]));

    return;
}

void USBMIDI::controlChange(uint8_t note, uint8_t value, int8_t channel){
    //set channel to default if channel is out of range (or not specified)
    if(channel<0 || channel>MAX_CHANNEL){channel = this->mChannel;}

    uint8_t data[3] = {CONTROL_CHANGE | (uint8_t)channel, note, value};
    tud_midi_stream_write(0, data, sizeof(data)/sizeof(data[0]));

    return;
}

int8_t USBMIDI::setChannel(int8_t channel){
    //return -1 if channel is out of range
    if(channel<0 || channel>MAX_CHANNEL){return -1;}

    this->mChannel = channel;
    //return the new channel on success
    return this->mChannel;
}
