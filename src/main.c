#include "SEGGER_RTT.h"
#include "SEGGER_SYSVIEW.h"
#include "hardware/irq.h"
#include "hardware/structs/systick.h"
#include "pico/stdlib.h"

#define LED_PIN PICO_DEFAULT_LED_PIN
#define BUTTON_PIN 2

void button_gpio_irq()
{
    SEGGER_SYSVIEW_RecordEnterISR();

    gpio_set_irq_enabled(BUTTON_PIN, GPIO_IRQ_EDGE_RISE, false);
    gpio_acknowledge_irq(BUTTON_PIN, GPIO_IRQ_EDGE_RISE);
    busy_wait_us(250);
    gpio_set_irq_enabled(BUTTON_PIN, GPIO_IRQ_EDGE_RISE, true);
    SEGGER_SYSVIEW_RecordExitISR();
}

// Rewrite of weak systick IRQ in crt0.s file
extern void isr_systick()  
{
    SEGGER_SYSVIEW_RecordEnterISR();
    SEGGER_SYSVIEW_RecordExitISR();
}

void init_systick()
{
    systick_hw->csr = 0;         // Disable
    systick_hw->rvr = 124999U;  // Standard System clock (125Mhz)/ (rvr value + 1) = 1ms
    systick_hw->cvr = 0;         // clear the count to force initial reload
    systick_hw->csr = 0x7;       // Enable Systic, Enable Exceptions
}

/*********************************************************************
 *
 *       SEGGER_SYSVIEW_X_GetTimestamp()
 *
 * Function description
 *   Returns the current timestamp in ticks using the system tick
 *   count and the SysTick counter.
 *   All parameters of the SysTick have to be known and are set via
 *   configuration defines on top of the file.
 *
 * Return value
 *   The current timestamp.
 *
 * Additional information
 *   SEGGER_SYSVIEW_X_GetTimestamp is always called when interrupts are
 *   disabled. Therefore locking here is not required.
 */
uint32_t SEGGER_SYSVIEW_X_GetTimestamp(void)
{
    uint32_t timeStamp = timer_hw->timerawl;
    return timeStamp;
}

int main()
{
    static char r;
    stdio_init_all();
    init_systick();

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    gpio_init(BUTTON_PIN);
    gpio_set_dir(BUTTON_PIN, GPIO_IN);
    gpio_acknowledge_irq(BUTTON_PIN, GPIO_IRQ_EDGE_RISE);
    gpio_set_irq_enabled(BUTTON_PIN, GPIO_IRQ_EDGE_RISE, true);
    gpio_add_raw_irq_handler(BUTTON_PIN, &button_gpio_irq);
    irq_set_enabled(IO_IRQ_BANK0, true);

    SEGGER_SYSVIEW_Conf();
    SEGGER_SYSVIEW_OnIdle();

    SEGGER_RTT_ConfigUpBuffer(0, NULL, NULL, 0, SEGGER_RTT_MODE_BLOCK_IF_FIFO_FULL);

    SEGGER_RTT_WriteString(0, "SEGGER Real-Time-Terminal Sample\r\n\r\n");
    SEGGER_RTT_WriteString(0, "###### Testing SEGGER_printf() ######\r\n");

    int counter = 0;
    while (true)
    {
        gpio_put(LED_PIN, 1);
        sleep_us(250000);
        gpio_put(LED_PIN, 0);
        sleep_us(250000);

        SEGGER_RTT_printf(0, "LED Cycle: %d\n", counter++);
    }
}
