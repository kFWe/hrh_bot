#include <stdio.h>
#include "SEGGER_RTT.h"
#include "SEGGER_SYSVIEW.h"
#include "hardware/clocks.h"
#include "hardware/irq.h"
#include "hardware/pio.h"
#include "hardware/pwm.h"
#include "hardware/structs/systick.h"
#include "pico/stdlib.h"
#include "quadrature.pio.h"

#define LED_PIN PICO_DEFAULT_LED_PIN
#define POS_SPEED_BUTTON_PIN 2
#define NEG_SPEED_BUTTON_PIN 3
#define DIR_CHANGE_BUTTON_PIN 4
#define MOTOR_PWM 18
#define MOTOR_IN1 17
#define MOTOR_IN2 16
#define QUADRATURE_A_PIN 14
#define QUADRATURE_B_PIN 15
#define DEBOUNCE_TIME 200000u  // ms
typedef enum Direction
{
    LEFT,
    RIGHT
} Direction;

PIO pio_;
uint sm_ = 0u;
uint8_t currentMotorSpeed_ = 0u;  // duty cycle in pecent
Direction currentMotorDirection_ = LEFT;
void setSpeed(uint8_t dutyCycle);
void setDirection(Direction motorDirection);

void button_gpio_irq()
{
    static uint32_t posButtonPushedLastTime = 0u;
    static uint32_t negButtonPushedLastTime = 0u;
    static uint32_t dirButtonPushedLastTime = 0u;

    SEGGER_SYSVIEW_RecordEnterISR();
    if (gpio_get_irq_event_mask(POS_SPEED_BUTTON_PIN) & GPIO_IRQ_EDGE_RISE)
    {
        gpio_acknowledge_irq(POS_SPEED_BUTTON_PIN, GPIO_IRQ_EDGE_RISE);

        // Only react on the edge if the last rising edge occured further away in the past than the dounce time
        uint32_t currentTime = time_us_32();
        if (currentTime - posButtonPushedLastTime > DEBOUNCE_TIME)
        {
            currentMotorSpeed_ += 10u;
            if (currentMotorSpeed_ > 100u)
            {
                currentMotorSpeed_ = 100u;
            }
            setSpeed(currentMotorSpeed_);
            posButtonPushedLastTime = time_us_32();
            SEGGER_RTT_WriteString(0, "POS SPEED IRQ\n");
        }
    }
    if (gpio_get_irq_event_mask(NEG_SPEED_BUTTON_PIN) & GPIO_IRQ_EDGE_RISE)
    {
        gpio_acknowledge_irq(NEG_SPEED_BUTTON_PIN, GPIO_IRQ_EDGE_RISE);

        // Only react on the edge if the last rising edge occured further away in the past than the dounce time
        uint32_t currentTime = time_us_32();
        if (currentTime - negButtonPushedLastTime > DEBOUNCE_TIME)
        {
            currentMotorSpeed_ -= 10u;
            if (currentMotorSpeed_ == 0u || currentMotorSpeed_ > 100u)  // undeflow
            {
                currentMotorSpeed_ = 0u;
            }
            setSpeed(currentMotorSpeed_);
            negButtonPushedLastTime = time_us_32();
            SEGGER_RTT_WriteString(0, "NEG SPEED IRQ\n");
        }
    }
    if (gpio_get_irq_event_mask(DIR_CHANGE_BUTTON_PIN) & GPIO_IRQ_EDGE_RISE)
    {
        gpio_acknowledge_irq(DIR_CHANGE_BUTTON_PIN, GPIO_IRQ_EDGE_RISE);
        uint8_t lastState = gpio_get(DIR_CHANGE_BUTTON_PIN);

        // Only react on the edge if the last rising edge occured further away in the past than the dounce time
        uint32_t currentTime = time_us_32();
        if (currentTime - dirButtonPushedLastTime > DEBOUNCE_TIME)
        {
            if (currentMotorDirection_ == LEFT)
            {
                currentMotorDirection_ = RIGHT;
            }
            else
            {
                currentMotorDirection_ = LEFT;
            }
            setDirection(currentMotorDirection_);
            dirButtonPushedLastTime = time_us_32();
            SEGGER_RTT_WriteString(0, "DIR CHANGE SPEED IRQ\n");
        }
    }
    SEGGER_SYSVIEW_RecordExitISR();
}

// Rewrite of weak systick IRQ in crt0.s file
extern void isr_systick()
{
    SEGGER_SYSVIEW_RecordEnterISR();

    pio_sm_exec_wait_blocking(pio_, sm_, pio_encode_in(pio_x, 32));
    uint x = pio_sm_get_blocking(pio_, sm_);
    SEGGER_SYSVIEW_RecordExitISR();
}

void init_systick()
{
    systick_hw->csr = 0;        // Disable
    systick_hw->rvr = 124999U;  // Standard System clock (125Mhz)/ (rvr value + 1) = 1ms
    systick_hw->cvr = 0;        // clear the count to force initial reload
    systick_hw->csr = 0x7;      // Enable Systic, Enable Exceptions
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

void setSpeed(uint8_t dutyCycle)
{
    uint16_t slice_num = pwm_gpio_to_slice_num(MOTOR_PWM);
    uint16_t duty = dutyCycle;                       // duty cycle, in percent
    uint16_t level = (0xFFFF - 1) * duty / 100 - 1;  // calculate channel level from given duty cycle in %
    pwm_set_chan_level(slice_num, 0, level);
    pwm_set_enabled(slice_num, true);
}

void setDirection(Direction motorDirection)
{
    switch (motorDirection)
    {
        case LEFT:
        {
            gpio_put(MOTOR_IN1, true);
            gpio_put(MOTOR_IN2, false);
            break;
        }
        case RIGHT:
        {
            gpio_put(MOTOR_IN1, true);
            gpio_put(MOTOR_IN2, false);
            break;
        }
        default:
        {
            // Left
            gpio_put(MOTOR_IN1, true);
            gpio_put(MOTOR_IN2, false);
        }
    }
}

int main()
{
    static char r;
    stdio_init_all();
    init_systick();

    SEGGER_SYSVIEW_Conf();
    SEGGER_SYSVIEW_OnIdle();

    SEGGER_RTT_ConfigUpBuffer(0, NULL, NULL, 0, SEGGER_RTT_MODE_BLOCK_IF_FIFO_FULL);

    SEGGER_RTT_WriteString(0, "SEGGER Real-Time-Terminal Sample\r\n\r\n");
    SEGGER_RTT_WriteString(0, "###### Testing SEGGER_printf() ######\r\n");

    // LED
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    // Button Inputs
    gpio_init(POS_SPEED_BUTTON_PIN);
    gpio_init(NEG_SPEED_BUTTON_PIN);
    gpio_set_dir(POS_SPEED_BUTTON_PIN, GPIO_IN);
    gpio_set_dir(NEG_SPEED_BUTTON_PIN, GPIO_IN);
    gpio_acknowledge_irq(POS_SPEED_BUTTON_PIN, GPIO_IRQ_EDGE_RISE);
    gpio_set_irq_enabled(POS_SPEED_BUTTON_PIN, GPIO_IRQ_EDGE_RISE, true);
    gpio_acknowledge_irq(NEG_SPEED_BUTTON_PIN, GPIO_IRQ_EDGE_RISE);
    gpio_set_irq_enabled(NEG_SPEED_BUTTON_PIN, GPIO_IRQ_EDGE_RISE, true);
    gpio_acknowledge_irq(DIR_CHANGE_BUTTON_PIN, GPIO_IRQ_EDGE_RISE);
    gpio_set_irq_enabled(DIR_CHANGE_BUTTON_PIN, GPIO_IRQ_EDGE_RISE, true);
    gpio_add_raw_irq_handler(POS_SPEED_BUTTON_PIN, &button_gpio_irq);
    gpio_add_raw_irq_handler(NEG_SPEED_BUTTON_PIN, &button_gpio_irq);
    gpio_add_raw_irq_handler(DIR_CHANGE_BUTTON_PIN, &button_gpio_irq);
    irq_set_enabled(IO_IRQ_BANK0, true);

    // Motor Driver
    gpio_init(MOTOR_IN1);
    gpio_set_dir(MOTOR_IN1, GPIO_OUT);
    gpio_init(MOTOR_IN2);
    gpio_set_dir(MOTOR_IN2, GPIO_OUT);

    gpio_set_function(MOTOR_PWM, GPIO_FUNC_PWM);
    uint16_t slice_num = pwm_gpio_to_slice_num(MOTOR_PWM);

    // PWM Freq. 5 kHz
    int32_t f_sys = clock_get_hz(clk_sys);
    float divider = f_sys / 25000U;
    pwm_set_clkdiv(slice_num, divider);
    pwm_set_wrap(slice_num, 0xFFFF);

    // Set default start direction (left)
    setDirection(currentMotorDirection_);

    // Quadrature Encoder PIO init. Position get's read every 1ms in systick irq
    pio_ = pio0;
    uint16_t offset = pio_add_program(pio_, &quadrature_program);
    sm_ = pio_claim_unused_sm(pio_, true);
    quadrature_program_init(pio_, sm_, offset, QUADRATURE_A_PIN, QUADRATURE_B_PIN);

    while (true)
    {
        gpio_put(LED_PIN, 1);
        sleep_us(2500);
        gpio_put(LED_PIN, 0);
        sleep_us(2500);
        printf("%d\n", currentMotorSpeed_);
    }
}
