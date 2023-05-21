#include <math.h>
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
void init_systick();

// Encoder Vars
#define QUADRATURE_A_PIN 14
#define QUADRATURE_B_PIN 15
PIO pio_;
uint sm_ = 0u;

// Motor Vars
#define POS_SPEED_BUTTON_PIN 2
#define NEG_SPEED_BUTTON_PIN 3
#define MOTOR_PWM 18
#define MOTOR_IN1 17
#define MOTOR_IN2 16
#define DEBOUNCE_TIME 200000u  // ms

typedef enum Direction
{
    LEFT,
    RIGHT
} Direction;
uint32_t currentMotorSpeed_ = 0u;  // rpm
uint32_t targetMotorSpeed_ = 0u;
uint32_t currentMotorPosition_ = 0u;
uint32_t targetMotorPosition_ = 0u;
Direction currentMotorDirection_ = LEFT;
Direction targetMotorDirection_ = LEFT;

void setSpeed(uint8_t dutyCycle);
void setDirection(Direction motorDirection);

// Controller Vars
uint32_t prevT = 0u;
float ePrev = 0.0f;
float eIntegral = 0.0f;
float kP = 1.0f;
float kD = 0.0f;
float kI = 0.0f;

// Rewrite of weak systick IRQ in crt0.s file
extern void isr_systick()
{
    SEGGER_SYSVIEW_RecordEnterISR();

    pio_sm_exec_wait_blocking(pio_, sm_, pio_encode_in(pio_x, 32));
    currentMotorPosition_ = pio_sm_get_blocking(pio_, sm_);
    SEGGER_SYSVIEW_RecordExitISR();
}

// Controls the speed and direction through the buttons
void increaseSpeedHandler()
{
    SEGGER_SYSVIEW_RecordEnterISR();
    static uint32_t posButtonPushedLastTime = 0u;

    if (gpio_get_irq_event_mask(POS_SPEED_BUTTON_PIN) & GPIO_IRQ_EDGE_RISE)
    {
        gpio_acknowledge_irq(POS_SPEED_BUTTON_PIN, GPIO_IRQ_EDGE_RISE);

        // Only react on the edge if the last rising edge occured further away in the past than the dounce time
        uint32_t currentTime = time_us_32();
        if (currentTime - posButtonPushedLastTime > DEBOUNCE_TIME)
        {
            targetMotorPosition_ = 2000;

            // currentMotorSpeed_ += 10u;
            // if (currentMotorSpeed_ > 100u)
            // {
            //     currentMotorSpeed_ = 100u;
            // }
            posButtonPushedLastTime = time_us_32();
            SEGGER_RTT_WriteString(0, "POS SPEED IRQ\n");
        }
    }
    SEGGER_SYSVIEW_RecordExitISR();
}

void decreaseSpeedHandler()
{
    SEGGER_SYSVIEW_RecordEnterISR();
    static uint32_t negButtonPushedLastTime = 0u;
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
            negButtonPushedLastTime = time_us_32();
            SEGGER_RTT_WriteString(0, "NEG SPEED IRQ\n");
        }
    }

    SEGGER_SYSVIEW_RecordExitISR();
}

bool controlLoop(struct repeating_timer* t)
{
    uint32_t currentT = time_us_32();

    float dt = ((float)currentT - prevT) / 1.0e6;
    prevT = currentT;

    // error
    uint32_t e = targetMotorPosition_ - currentMotorPosition_;

    // D
    float dedt = (e - ePrev) / dt;

    // I
    eIntegral = eIntegral + e * dt;

    // Control signal
    float u = kP * e + kD * dedt + kI * eIntegral;
    targetMotorSpeed_ = fabs(u);

    if (targetMotorSpeed_ > 100u)
    {
        targetMotorSpeed_ = 100u;
    }

    targetMotorDirection_ = LEFT;
    if (u < 0)
    {
        targetMotorDirection_ = RIGHT;
    }

    setDirection(targetMotorDirection_);
    setSpeed(targetMotorSpeed_);

    ePrev = e;
    return true;
}

int main()
{
    static char r;
    stdio_init_all();

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
    gpio_add_raw_irq_handler(POS_SPEED_BUTTON_PIN, &increaseSpeedHandler);
    gpio_add_raw_irq_handler(NEG_SPEED_BUTTON_PIN, &decreaseSpeedHandler);
    irq_set_enabled(IO_IRQ_BANK0, true);

    init_systick();

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

    // Setup the control loop
    struct repeating_timer timer;
    add_repeating_timer_ms(1000, controlLoop, NULL, &timer);

    while (true)
    {
        gpio_put(LED_PIN, 1);
        sleep_us(2500);
        gpio_put(LED_PIN, 0);
        sleep_us(2500);
        printf("%d \t %d \t %d\n", currentMotorPosition_, targetMotorPosition_, targetMotorSpeed_);
    }
}

void setSpeed(uint8_t dutyCycle)
{
    currentMotorSpeed_ = dutyCycle;  // TODO: update speed from encoder read values

    uint16_t slice_num = pwm_gpio_to_slice_num(MOTOR_PWM);
    uint16_t duty = dutyCycle;                     // duty cycle, in percent
    uint16_t level = (0xFFFF - 1u) * duty / 100u;  // calculate channel level from given duty cycle in %
    pwm_set_chan_level(slice_num, 0, level);
    pwm_set_enabled(slice_num, true);
}

void setDirection(Direction motorDirection)
{
    currentMotorDirection_ = motorDirection;
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

uint32_t SEGGER_SYSVIEW_X_GetTimestamp(void)
{
    uint32_t timeStamp = timer_hw->timerawl;
    return timeStamp;
}

void init_systick()
{
    systick_hw->csr = 0;        // Disable
    systick_hw->rvr = 124999U;  // Standard System clock (125Mhz)/ (rvr value + 1) = 1ms
    systick_hw->cvr = 0;        // clear the count to force initial reload
    systick_hw->csr = 0x7;      // Enable Systic, Enable Exceptions
}