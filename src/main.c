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
#include "PwmIn.pio.h"
#include <float.h>


/***********************************
* Board Config
************************************/ 

// RPI Pico
#define QUADRATURE_A_PIN 14
#define QUADRATURE_B_PIN 15
#define RC_CH1_PIN 0
#define RC_CH2_PIN 1
#define POS_SPEED_BUTTON_PIN 2
#define NEG_SPEED_BUTTON_PIN 3
#define LEFT_MOTOR_IN1 6
#define LEFT_MOTOR_IN2 7
#define LEFT_MOTOR_PWM 8
#define RIGHT_MOTOR_IN1 9
#define RIGHT_MOTOR_IN2 10
#define RIGHT_MOTOR_PWM 11


/***********************************
* Static Vars
************************************/ 
// Motor Vars
uint8_t leftMotorTargetSpeed_ = 0u;
uint8_t rightMotorTargetSpeed_ = 0u;
uint16_t pwm_wrap_ = 12500u;
uint8_t pwm_clk_div_ = 2u;
bool mixing_ = true;

// Encoder Vars
PIO pio_;
uint sm_ = 1u;

// RC Var
PIO rcPio_;
uint rcsm_ = 0u;
float rc_period_ch1 = 0.0f; // in s
float rc_pulseWidth_ch1 = 0.0f; // in s
float rc_dutyCycle_ch1 = 0.0f;
float rc_period_ch2 = 0.0f; // in s
float rc_pulseWidth_ch2 = 0.0f; // in s
float rc_dutyCycle_ch2 = 0.0f;
static uint32_t pulsewidth[4], period[4];

// Motor Vars
typedef enum Direction
{
    FORWARD,
    BACKWARD
} Direction;
uint32_t currentMotorSpeedLeft_ = 0u;  // pwm
uint32_t currentMotorSpeedRight_ = 0u;  // pwm
Direction currentMotorDirectionLeft_ = FORWARD;
Direction targetMotorDirectionLeft_ = FORWARD;
Direction currentMotorDirectionRight_ = FORWARD;
Direction targetMotorDirectionRight_ = FORWARD;


/***********************************
* Functions
************************************/ 
void init_systick();
void setSpeedLeft(uint8_t dutyCycle);
void setDirectionLeft(Direction motorDirection);
void setSpeedRight(uint8_t dutyCycle);
void setDirectionRight(Direction motorDirection);
float rc_readCh1(void);
float rc_readCh2(void);
void initialize_RC_PIO(uint *pin_list, uint num_of_pins);

// 1ms Systick irq. Used to read rc 
extern void isr_systick()
{
    SEGGER_SYSVIEW_RecordEnterISR();
    rc_readCh1();
    rc_readCh2();
    SEGGER_SYSVIEW_RecordExitISR();
}


// set the pio rc receiver irq handler
void pio_irq_handler()
{
    int state_machine = -1;
    // check which IRQ was raised:
    for (int i = 0; i < 4; i++)
    {
        if (pio0_hw->irq & 1<<i)
        {
            // clear interrupt
            pio0_hw->irq = 1 << i;
            // read pulse width from the FIFO
            pulsewidth[i] = pio_sm_get(rcPio_, i);
            // read low period from the FIFO
            period[i] = pio_sm_get(rcPio_, i);
            // clear interrupt
            pio0_hw->irq = 1 << i;
        }
    }
}

// 1ms control loop
bool controlLoop(struct repeating_timer* t)
{
    // rc signal from 0.07 to 0.17. Middle Point at 0.12
    int input_start = 0;    // The lowest number of the range input.
    int input_end = 50;    // The largest number of the range input.
    int output_start = 0; // The lowest number of the range output.
    int output_end = 100;  // The largest number of the range output.
    double slope = 1.0 * (output_end - output_start) / (input_end - input_start);

    int8_t ch1Normed = (int8_t)(rc_pulseWidth_ch1 * 100);
    int8_t ch2Normed = (int8_t)(rc_pulseWidth_ch2 * 100);
    
    if (mixing_)
    {
        int8_t mixedLeft = (ch1Normed - ch2Normed);
        int8_t mixedRight = (ch1Normed + ch2Normed);
        leftMotorTargetSpeed_  = fabs(output_start + slope * ( mixedLeft - input_start));
        rightMotorTargetSpeed_ = fabs(output_start + slope * ( mixedRight - input_start));

        targetMotorDirectionLeft_ = FORWARD;
        targetMotorDirectionRight_ = FORWARD;
        if (mixedLeft < 0)
        {
            targetMotorDirectionLeft_ = BACKWARD;
        }
        if (mixedRight < 0)
        {
            targetMotorDirectionRight_ = BACKWARD;
        }
    }
    else 
    {
        leftMotorTargetSpeed_  = fabs(output_start + slope * (ch1Normed - input_start));
        rightMotorTargetSpeed_ = fabs(output_start + slope * (ch1Normed - input_start));

        targetMotorDirectionLeft_ = FORWARD;
        targetMotorDirectionRight_ = FORWARD;
        if (ch1Normed < 0)
        {
            targetMotorDirectionLeft_ = BACKWARD;
            targetMotorDirectionRight_ = BACKWARD;
        }
    }

    if (leftMotorTargetSpeed_ > 100u)
    {
        leftMotorTargetSpeed_ = 100u;
    }
    if (rightMotorTargetSpeed_ > 100u)
    {
        rightMotorTargetSpeed_ = 100u;
    }

   
    setDirectionLeft(targetMotorDirectionLeft_);
    setDirectionRight(targetMotorDirectionRight_);
    setSpeedLeft(leftMotorTargetSpeed_);
    setSpeedRight(rightMotorTargetSpeed_);
    
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

    // Left Motor Driver
    gpio_init(LEFT_MOTOR_IN1);
    gpio_set_dir(LEFT_MOTOR_IN1, GPIO_OUT);
    gpio_init(LEFT_MOTOR_IN2);
    gpio_set_dir(LEFT_MOTOR_IN2, GPIO_OUT);

    gpio_set_function(LEFT_MOTOR_PWM, GPIO_FUNC_PWM);
    uint16_t slice_num = pwm_gpio_to_slice_num(LEFT_MOTOR_PWM);
    pwm_set_enabled(slice_num, false);

    // PWM Freq. 5 kHz
    int32_t f_sys = clock_get_hz(clk_sys);
    pwm_set_clkdiv(slice_num, pwm_clk_div_);
    pwm_set_wrap(slice_num, pwm_wrap_);

    // Right Motor Driver
    gpio_init(RIGHT_MOTOR_IN1);
    gpio_set_dir(RIGHT_MOTOR_IN1, GPIO_OUT);
    gpio_init(RIGHT_MOTOR_IN2);
    gpio_set_dir(RIGHT_MOTOR_IN2, GPIO_OUT);

    gpio_set_function(RIGHT_MOTOR_PWM, GPIO_FUNC_PWM);
    uint16_t slice_num_right = pwm_gpio_to_slice_num(RIGHT_MOTOR_PWM);
    pwm_set_enabled(slice_num_right, false);

    // PWM Freq. 5 kHz
    pwm_set_clkdiv(slice_num_right, pwm_clk_div_);
    pwm_set_wrap(slice_num_right, pwm_wrap_);

    // Init irq which reads out the signals
    init_systick();

    // Quadrature Encoder PIO init. Position get's read every 1ms in systick irq
    pio_ = pio1;
    uint16_t offset = pio_add_program(pio_, &quadrature_program);
    sm_ = pio_claim_unused_sm(pio_, true);
    quadrature_program_init(pio_, sm_, offset, QUADRATURE_A_PIN, QUADRATURE_B_PIN);

    // Init RC input reader
    uint pin_list[2] = {RC_CH1_PIN, RC_CH2_PIN};
    initialize_RC_PIO(pin_list, 2);

    // Setup the control loop
    struct repeating_timer timer;
    add_repeating_timer_ms(1, controlLoop, NULL, &timer);

    while (true)
    {
        
        printf("%.8f \t %.8f \t %d \t %d \n", rc_pulseWidth_ch1, rc_pulseWidth_ch2, leftMotorTargetSpeed_, rightMotorTargetSpeed_);
        //printf("%d \t %d \t %d \t %d\n", currentMotorPosition_, targetMotorPosition_, currentMotorSpeedLeft_, currentMotorSpeedRight_);
    }
}

void setSpeedLeft(uint8_t dutyCycle)
{
    currentMotorSpeedLeft_ = dutyCycle;  // TODO: update speed from encoder read values

    uint16_t slice_num = pwm_gpio_to_slice_num(LEFT_MOTOR_PWM);
    uint16_t duty = dutyCycle;                     // duty cycle, in percent
    uint16_t level = (pwm_wrap_ - 1u) * duty / 100u;  // calculate channel level from given duty cycle in %
    pwm_set_chan_level(slice_num, 0, level);
    pwm_set_enabled(slice_num, true);
}

void setSpeedRight(uint8_t dutyCycle)
{
    currentMotorSpeedRight_ = dutyCycle;  // TODO: update speed from encoder read values

    uint16_t slice_num = pwm_gpio_to_slice_num(RIGHT_MOTOR_PWM);
    uint16_t duty = dutyCycle;                     // duty cycle, in percent
    uint16_t level = (pwm_wrap_ - 1u) * duty / 100u;  // calculate channel level from given duty cycle in %
    pwm_set_chan_level(slice_num, 1, level);
    pwm_set_enabled(slice_num, true);
}

void setDirectionLeft(Direction motorDirection)
{
    currentMotorDirectionLeft_ = motorDirection;
    switch (motorDirection)
    {
        case FORWARD:
        {
            gpio_put(LEFT_MOTOR_IN1, true);
            gpio_put(LEFT_MOTOR_IN2, false);
            break;
        }
        case BACKWARD:
        {
            gpio_put(LEFT_MOTOR_IN1, false);
            gpio_put(LEFT_MOTOR_IN2, true);
            break;
        }
        default:
        {
            // Left
            gpio_put(LEFT_MOTOR_IN1, true);
            gpio_put(LEFT_MOTOR_IN2, false);
        }
    }
}

void setDirectionRight(Direction motorDirection)
{
    currentMotorDirectionLeft_ = motorDirection;
    switch (motorDirection)
    {
        case FORWARD:
        {
            gpio_put(RIGHT_MOTOR_IN1, true);
            gpio_put(RIGHT_MOTOR_IN2, false);
            break;
        }
        case BACKWARD:
        {
            gpio_put(RIGHT_MOTOR_IN1, false);
            gpio_put(RIGHT_MOTOR_IN2, true);
            break;
        }
        default:
        {
            // Left
            gpio_put(RIGHT_MOTOR_IN1, true);
            gpio_put(RIGHT_MOTOR_IN2, false);
        }
    }
}


 // read the period and pulsewidth
float rc_readCh1(void)
{
    // Convert from clock cycle to ms
    rc_period_ch1 = 2 * period[0] * 0.000008f;
        
    // one clock cycle is 1/125000 ms
    rc_pulseWidth_ch1 = (floorf( (2.0f * (float)pulsewidth[0] * 0.000008f) * 100.0f) / 100.0f) - 1.5f;
    
    // read_dutycycle (between 0 and 1)
    rc_dutyCycle_ch1 = roundf(((float)pulsewidth[0] / (float)period[0]) * 100.0f) / 100.0f - 0.12f;

    return 0;
}

float rc_readCh2(void)
{
     // Convert from clock cycle to ms
    rc_period_ch2 = 2 * period[1] * 0.000008;
    
    // one clock cycle is 1/125000 ms
    rc_pulseWidth_ch2 = (floorf( (2.0f * (float)pulsewidth[1] * 0.000008f) * 100.0f) / 100.0f) - 1.5f;
    
    // read_dutycycle (between 0 and 1)
    rc_dutyCycle_ch2 = roundf(((float)pulsewidth[1] / (float)period[1])* 100.0f) / 100.0f -0.12f;

    return 0;
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


void initialize_RC_PIO(uint *pin_list, uint num_of_pins)
{
    // pio 0 is used
    rcPio_ = pio0;
    // load the pio program into the pio memory
    uint offset = pio_add_program(rcPio_, &PwmIn_program);
    // start num_of_pins state machines
    for (int i = 0; i < num_of_pins; i++)
    {
        // prepare state machine i
        pulsewidth[i] = 0;
        period[i] = 0;

        // configure the used pins (pull down, controlled by PIO)
        gpio_pull_down(pin_list[i]);
        pio_gpio_init(rcPio_, pin_list[i]);
        // make a sm config
        pio_sm_config c = PwmIn_program_get_default_config(offset);
        // set the 'jmp' pin
        sm_config_set_jmp_pin(&c, pin_list[i]);
        // set the 'wait' pin (uses 'in' pins)
        sm_config_set_in_pins(&c, pin_list[i]);
        // set shift direction
        sm_config_set_in_shift(&c, false, false, 0);
        // init the pio sm with the config
        pio_sm_init(rcPio_, i, offset, &c);
        // enable the sm
        pio_sm_set_enabled(rcPio_, i, true);
    }
    // set the IRQ handler
    irq_set_exclusive_handler(PIO0_IRQ_0, pio_irq_handler);
    // enable the IRQ
    irq_set_enabled(PIO0_IRQ_0, true);
    // allow irqs from the low 4 state machines
    pio0_hw->inte0 = PIO_IRQ0_INTE_SM0_BITS | PIO_IRQ0_INTE_SM1_BITS | PIO_IRQ0_INTE_SM2_BITS | PIO_IRQ0_INTE_SM3_BITS ;
};