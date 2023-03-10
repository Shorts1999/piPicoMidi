#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/malloc.h"
#include "pico/mem_ops.h"
#include <stdlib.h>
#include "pico.h"

#include <FreeRTOS.h>
#include <FreeRTOSConfig.h>
#include <task.h>

#include "pico/multicore.h"
#include "hardware/pio_instructions.h"
#include "hardware/adc.h"

#include "ws2812.pio.h"

#include "tusb.h"

#include "USBMIDI.h"


const uint16_t BLINK_DELAY = 500;
const uint16_t HELLO_DELAY = 2000;
#define USB_STACK_SIZE      2*configMINIMAL_STACK_SIZE

/**
 * @brief FreeRTOS delay, converts ms to ticks
 *
 * @param ms delay time in milliseconds
 */
void inline delay_ms(uint32_t ms) {
    vTaskDelay(ms / portTICK_PERIOD_MS);
}

void initQueue(void);
void midi_task(void *params);
void blink(void *params);
void usb_task(void *params);
void btnISR(uint pin, uint32_t event);
void potTask(void *params);
void ledTask(void *params);

typedef struct MidiData {
    uint8_t cmd;
    uint8_t index;  //Note or control value
    uint8_t value;  //Intensity
} MidiData;

//IO defines
const uint8_t cButtonPin = 16;
const uint8_t cPinAdc0 = 26;
const uint8_t cRGBLedPin = 17;

#define ANALOG_DELAY_TIME 100
#define MIDI_ADC_RESOLUTION 7   //MIDI data is range 0-127 (7 bits)
#define ADC_RESOLUTION 12       //Pi Pico ADC is 12-bit

static inline void put_pixel(uint32_t pixel_grb) {
    pio_sm_put_blocking(pio0, 0, pixel_grb << 8u);
}

//Queue
QueueHandle_t midiQueue = NULL;
QueueHandle_t rgbQueue = NULL;

int main(void) {
    stdio_init_all();
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);


    initQueue();

    gpio_init(cButtonPin);
    gpio_set_dir(cButtonPin, GPIO_IN);
    gpio_set_pulls(cButtonPin, true, false); // Set button pin as pull-up
    gpio_set_irq_enabled_with_callback(cButtonPin, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true, btnISR);

    adc_init();
    adc_gpio_init(cPinAdc0);

    // xTaskCreate(blink, "blink", configMINIMAL_STACK_SIZE, NULL, 0, NULL);
    xTaskCreate(usb_task, "usb", USB_STACK_SIZE, NULL, configMAX_PRIORITIES - 1, NULL);
    xTaskCreate(midi_task, "midi", configMINIMAL_STACK_SIZE, NULL, configMAX_PRIORITIES, NULL);
    xTaskCreate(potTask, "potentiometer", configMINIMAL_STACK_SIZE, NULL, configMAX_PRIORITIES - 1, NULL);
    xTaskCreate(ledTask, "led", configMINIMAL_STACK_SIZE, NULL, configMAX_PRIORITIES - 1, NULL);
    vTaskStartScheduler();
}


void initQueue(void) {
    midiQueue = xQueueCreate(10, sizeof(MidiData));
    rgbQueue = xQueueCreate(10, sizeof(uint32_t));

    if (midiQueue == NULL || rgbQueue == NULL) {
        //ABORT :: ALL HELL BREAKS LOOSE
        // abort();
        //Okay so don't actually abort but maybe turn on the LED to show an error?
        gpio_put(PICO_DEFAULT_LED_PIN, 1);
        while (1);
    }
}

void midi_task(void *params) {
    MidiData midiData;
    while (1) {
        if (xQueueReceive(midiQueue, &midiData, 10) == pdTRUE) {

            uint8_t note_data[3] = { midiData.cmd, midiData.index, midiData.value };
            tud_midi_stream_write(0, note_data, sizeof(note_data) / sizeof(note_data[0]));
            //printf("CMD: %i, Index: %i, Value: %i", midiData.cmd, midiData.index, midiData.value);
            // delay_ms(10);
        }
    }
}

void blink(void *params) {
    while (true) {
        gpio_put(PICO_DEFAULT_LED_PIN, 1);
        delay_ms(BLINK_DELAY);
        gpio_put(PICO_DEFAULT_LED_PIN, 0);
        delay_ms(BLINK_DELAY);
    }
}

void usb_task(void *params) {
    tusb_init();


    while (1) {
        tud_task();
        //Task needs to run frequently. Very small delay to give room for other tasks
        delay_ms(10);
    }
}

void potTask(void *params) {
    uint16_t potMemory = 0;

    while (1) {
        adc_select_input(0);
        uint16_t adcValue = adc_read();
        uint8_t midiAdcValue = (adcValue >> (ADC_RESOLUTION - MIDI_ADC_RESOLUTION)) & 0x7f;
        if (midiAdcValue != potMemory) {
            MidiData data = {
                .cmd = CONTROL_CHANGE,
                .index = 2,
                .value = midiAdcValue
            };

            xQueueSendToBack(midiQueue, &data, 0);
            potMemory = midiAdcValue;
        }
        delay_ms(ANALOG_DELAY_TIME);
    }
}

void btnISR(uint gpio, uint32_t event) {
    MidiData data;
    if (event & GPIO_IRQ_EDGE_FALL) {
        data.cmd = NOTE_ON;
        data.index = 60;
        data.value = 127;
        //Send state of button to MIDI task queue
        xQueueSendToBackFromISR(midiQueue, &data, NULL);

        //Update LED:
        uint32_t colour = 0x00ff00;
        xQueueSendToBackFromISR(rgbQueue, &colour, NULL);
        return;
    }
    if (event & GPIO_IRQ_EDGE_RISE) {
        data.cmd = NOTE_OFF;
        data.index = 60;
        data.value = 0;
        //Send state of button to MIDI task queue
        xQueueSendToBackFromISR(midiQueue, &data, NULL);

        uint32_t colour = 0x000000;
        //Update LED:
        xQueueSendToBackFromISR(rgbQueue, &colour, NULL);
        return;
    }
}

void ledTask(void *params) {
    // todo get free sm
    PIO pio = pio0;
    int sm = 0;
    uint offset = pio_add_program(pio, &ws2812_program);

    ws2812_program_init(pio, sm, offset, cRGBLedPin, 800000, false);

    uint32_t colour = 0;
    while (1) {
        if (xQueueReceive(rgbQueue, &colour, 100) == pdTRUE) {
            put_pixel(colour);
        }
        // delay_ms(100); //delay occurs on empty queue already
    }
}