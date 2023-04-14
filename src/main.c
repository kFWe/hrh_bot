/**
 * Minimal blinky example to test the build toolchain.
 */

#include "pico/stdlib.h"

#define LED_PIN PICO_DEFAULT_LED_PIN

int main()
{
    static char r;
    stdio_init_all();
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    while (true)
    {
        gpio_put(LED_PIN, 1);
        sleep_ms(250);
        gpio_put(LED_PIN, 0);
        sleep_ms(250);
    }
}
