#include <float.h>
#include <math.h>
#include <stdio.h>
#include "PwmIn.pio.h"
#include "hardware/clocks.h"
#include "hardware/i2c.h"
#include "hardware/irq.h"
#include "hardware/pio.h"
#include "hardware/pwm.h"
#include "hardware/structs/systick.h"
#include "pico/stdlib.h"

/***********************************
 * Board Config
 ************************************/

// RPI Pico

#define RC_CH1_PIN 0
#define RC_CH2_PIN 1
#define RIGHT_MOTOR_PWM 3
#define RIGHT_MOTOR_PHASE 4
#define LEFT_MOTOR_PWM 5
#define LEFT_MOTOR_PHASE 6
#define DRV8835_MODE 7

/***********************************
 * Static Vars
 ************************************/
// Motor Vars
uint8_t leftMotorTargetSpeed_ = 0u;
uint8_t rightMotorTargetSpeed_ = 0u;
uint8_t mixedLeft_ = 0;
uint8_t mixedRight_ = 0;
uint16_t pwm_wrap_ = 12500u;
uint8_t pwm_clk_div_ = 1u;
uint8_t pwmDeadzone_ = 7;

// RC Var
PIO rcPio_;
uint rcsm_ = 0u;
float rcPeriodCH1 = 0.0f;
float rcPulseWidthCH1 = 0.0f;
float rcPeriodCH2 = 0.0f;
float rcPulseWidthCH2 = 0.0f;
static uint32_t pulsewidth[4], period[4];

// Motor Vars
typedef enum Direction
{
    FORWARD,
    BACKWARD
} Direction;
typedef enum MixStrategy
{
    NONE,
    LINEAR,
    POS_EXPONENTIAL,
    NEG_EXPONENTIAL,
    LAST
} MixStrategy;
uint8_t currentMotorSpeedLeft_ = 0u;   // pwm
uint8_t currentMotorSpeedRight_ = 0u;  // pwm
Direction currentMotorDirectionLeft_ = FORWARD;
Direction targetMotorDirectionLeft_ = FORWARD;
Direction currentMotorDirectionRight_ = FORWARD;
Direction targetMotorDirectionRight_ = FORWARD;
MixStrategy mixStrategy_ = NEG_EXPONENTIAL;

/***********************************
 * Functions
 ************************************/
void init_systick();
void set_pwm(uint8_t dutyCycle, uint8_t pwmPin);
void set_direction_left(Direction motorDirection);
void set_direction_right(Direction motorDirection);
float rc_readCh1(void);
float rc_readCh2(void);
void initialize_rc_PIO(uint* pin_list, uint num_of_pins);
void initializeBoard();

// 1ms Systick irq. Used to update the static vars for the rc channel data
extern void isr_systick()
{
    rc_readCh1();
    rc_readCh2();
}

// 1ms control loop
bool controlLoop(struct repeating_timer* t)
{
    // rc signal used is the pulse width of the rc receiver which was normed to be -1.0 to 1.0, with 0.0 beeing the
    // middle point.
    int8_t ch1Normed = (int8_t)(rcPulseWidthCH1 * 100);
    int8_t ch2Normed = (int8_t)(rcPulseWidthCH2 * 100);

    switch (mixStrategy_)
    {
        case NONE:
        {
            leftMotorTargetSpeed_ = abs(ch2Normed);
            rightMotorTargetSpeed_ = abs(ch2Normed);
            break;
        }
        case LINEAR:
        {
            if (ch2Normed == 0)
            {
                // Turn at full speed
                leftMotorTargetSpeed_ = abs(ch1Normed);
                rightMotorTargetSpeed_ = abs(ch1Normed);
                break;
            }

            mixedLeft_ = abs(ch2Normed);
            mixedRight_ = abs(ch2Normed);
            int8_t mix = abs(ch2Normed) - abs(ch1Normed);

            // Right Turn
            if (ch1Normed < 0)
            {
                if (mix < 0)  // underflow check
                {
                    mixedLeft_ = 0;
                }
                else
                {
                    mixedLeft_ = mix;
                }
            }

            // Left Turn
            if (ch1Normed > 0)
            {
                if (mix < 0)  // underflow check
                {
                    mixedRight_ = 0;
                }
                else
                {
                    mixedRight_ = mix;
                }
            }
            leftMotorTargetSpeed_ = mixedLeft_;
            rightMotorTargetSpeed_ = mixedRight_;
            break;
        }
        case POS_EXPONENTIAL:
        {
            mixedLeft_ = abs(ch2Normed);
            mixedRight_ = abs(ch2Normed);

            if (ch2Normed == 0)
            {
                // Turn at full speed
                leftMotorTargetSpeed_ = abs(ch1Normed);
                rightMotorTargetSpeed_ = abs(ch1Normed);
                break;
            }

            // Max range for channel is from 0 to 100. We need to limit the exponential function around those to points
            // x_norm = ch1/100
            // y = x_norm^a * 100
            // Exponent 0 < a <= 1
            float a = 0.5f;
            float x_normed = (float)abs(ch1Normed) / 100.0f;
            float y = pow(x_normed, a) * 100.0f;

            int8_t mix = abs(ch2Normed) - abs((int)y);

            // Right Turn
            if (ch1Normed < 0)
            {
                if (mix < 0)  // underflow check
                {
                    mixedLeft_ = 0;
                }
                else
                {
                    mixedLeft_ = mix;
                }
            }

            // Left Turn
            if (ch1Normed > 0)
            {
                if (mix < 0)  // underflow check
                {
                    mixedRight_ = 0;
                }
                else
                {
                    mixedRight_ = mix;
                }
            }
            leftMotorTargetSpeed_ = mixedLeft_;
            rightMotorTargetSpeed_ = mixedRight_;
            break;
        }
        case NEG_EXPONENTIAL:
        {
            if (ch2Normed == 0)
            {
                // Turn at full speed
                leftMotorTargetSpeed_ = abs(ch1Normed);
                rightMotorTargetSpeed_ = abs(ch1Normed);
                break;
            }

            mixedLeft_ = abs(ch2Normed);
            mixedRight_ = abs(ch2Normed);

            // Max range for channel is 0 to 100. We need to limit the exponential function around those to points
            // x_norm = ch1/100
            // y = x_norm^a * 100
            // Exponent a > 1
            float a = 2.0f;
            float x_normed = (float)abs(ch1Normed) / 100.0f;
            float y = pow(x_normed, a) * 100.0f;

            int8_t mix = abs(ch2Normed) - abs((int)y);
            float scaleMixOutput = 0.25f;

            // Right Turn
            if (ch1Normed < 0)
            {
                if (mix < 0)  // underflow check
                {
                    mixedLeft_ = 0;
                }
                else
                {
                    mixedLeft_ = mix;
                }
            }

            // Left Turn
            if (ch1Normed > 0)
            {
                if (mix < 0)  // underflow check
                {
                    mixedRight_ = 0;
                }
                else
                {
                    mixedRight_ = mix;
                }
            }

            leftMotorTargetSpeed_ = mixedLeft_;
            rightMotorTargetSpeed_ = mixedRight_;

            break;
        }
    }

    if (ch2Normed == 0)
    {
        if (ch1Normed < 0)
        {
            targetMotorDirectionLeft_ = BACKWARD;
            targetMotorDirectionRight_ = FORWARD;
        }
        if (ch1Normed > 0)
        {
            targetMotorDirectionLeft_ = FORWARD;
            targetMotorDirectionRight_ = BACKWARD;
        }
        // keep the last direction if both channels are low
    }
    else
    {
        if (ch2Normed > 0)
        {
            targetMotorDirectionLeft_ = FORWARD;
            targetMotorDirectionRight_ = FORWARD;
        }
        else
        {
            targetMotorDirectionLeft_ = BACKWARD;
            targetMotorDirectionRight_ = BACKWARD;
        }
    }

    // Check Max & Min
    if (leftMotorTargetSpeed_ > 100u)
    {
        leftMotorTargetSpeed_ = 100u;
    }
    if (rightMotorTargetSpeed_ > 100u)
    {
        rightMotorTargetSpeed_ = 100u;
    }
    if (leftMotorTargetSpeed_ < pwmDeadzone_)
    {
        leftMotorTargetSpeed_ = 0;
    }
    if (rightMotorTargetSpeed_ < pwmDeadzone_)
    {
        rightMotorTargetSpeed_ = 0;
    }

    set_direction_left(targetMotorDirectionLeft_);
    set_direction_right(targetMotorDirectionRight_);
    set_pwm(leftMotorTargetSpeed_, LEFT_MOTOR_PWM);
    set_pwm(rightMotorTargetSpeed_, RIGHT_MOTOR_PWM);

    // sync pwms
    pwm_set_mask_enabled((1 << pwm_gpio_to_slice_num(LEFT_MOTOR_PWM)) | (1 << pwm_gpio_to_slice_num(RIGHT_MOTOR_PWM)));
    return true;
}

int main()
{
    static char r;
    stdio_init_all();

    initializeBoard();

    // Setup the control loop
    struct repeating_timer timer;
    add_repeating_timer_ms(1, controlLoop, NULL, &timer);

    while (true)
    {
        printf("%.8f \t %.8f \t %d \t %d\n", rcPulseWidthCH1, rcPulseWidthCH2, leftMotorTargetSpeed_,
               rightMotorTargetSpeed_);
    }
}

void initializeBoard()
{
    gpio_init(DRV8835_MODE);
    gpio_set_dir(DRV8835_MODE, GPIO_OUT);
    gpio_put(DRV8835_MODE, true);

    // Left Motor Driver
    gpio_init(LEFT_MOTOR_PHASE);
    gpio_set_dir(LEFT_MOTOR_PHASE, GPIO_OUT);

    gpio_set_function(LEFT_MOTOR_PWM, GPIO_FUNC_PWM);
    uint16_t slice_num = pwm_gpio_to_slice_num(LEFT_MOTOR_PWM);
    pwm_set_enabled(slice_num, false);

    // PWM Freq. 5 kHz
    int32_t f_sys = clock_get_hz(clk_sys);
    pwm_set_clkdiv(slice_num, pwm_clk_div_);
    pwm_set_wrap(slice_num, pwm_wrap_);

    // Right Motor Driver
    gpio_init(RIGHT_MOTOR_PHASE);
    gpio_set_dir(RIGHT_MOTOR_PHASE, GPIO_OUT);

    gpio_set_function(RIGHT_MOTOR_PWM, GPIO_FUNC_PWM);
    uint16_t slice_num_right = pwm_gpio_to_slice_num(RIGHT_MOTOR_PWM);
    pwm_set_enabled(slice_num_right, false);

    // PWM Freq. 5 kHz
    pwm_set_clkdiv(slice_num_right, pwm_clk_div_);
    pwm_set_wrap(slice_num_right, pwm_wrap_);

    // Init irq which reads out the signals
    init_systick();

    // Init RC input reader
    uint pinList[3] = {RC_CH1_PIN, RC_CH2_PIN};
    initialize_rc_PIO(pinList, 2);
}

/* Sets the pwm level on a specific pwm channel but does not output it, to allow for syncing of multiple channels
 */
void set_pwm(uint8_t dutyCycle, uint8_t pwmPin)
{
    currentMotorSpeedLeft_ = dutyCycle;  // TODO: update speed from encoder read values

    uint16_t sliceNumber = pwm_gpio_to_slice_num(pwmPin);
    uint16_t duty = dutyCycle;                           // duty cycle, in percent
    uint16_t pwmLevel = (pwm_wrap_ - 1u) * duty / 100u;  // calculate channel level from given duty cycle in %
    pwm_set_chan_level(sliceNumber, pwm_gpio_to_channel(pwmPin), pwmLevel);
}

void set_direction_left(Direction motorDirection)
{
    currentMotorDirectionLeft_ = motorDirection;
    switch (motorDirection)
    {
        case FORWARD:
        {
            gpio_put(LEFT_MOTOR_PHASE, true);
            break;
        }
        case BACKWARD:
        {
            gpio_put(LEFT_MOTOR_PHASE, false);
            break;
        }
        default:
        {
            gpio_put(LEFT_MOTOR_PHASE, true);
        }
    }
}

void set_direction_right(Direction motorDirection)
{
    currentMotorDirectionLeft_ = motorDirection;
    switch (motorDirection)
    {
        case FORWARD:
        {
            gpio_put(RIGHT_MOTOR_PHASE, true);
            break;
        }
        case BACKWARD:
        {
            gpio_put(RIGHT_MOTOR_PHASE, false);
            break;
        }
        default:
        {
            // Left
            gpio_put(RIGHT_MOTOR_PHASE, true);
        }
    }
}

// read the period and pulsewidth
float rc_readCh1(void)
{
    // Convert from clock cycle to ms
    rcPeriodCH1 = 2 * period[0] * 0.000008f;

    // Check if the rc sender is on
    if (pulsewidth[0] > 0)
    {
        // one clock cycle is 1/125000 ms
        rcPulseWidthCH1 = 2.0f * ((floorf((2.0f * (float)pulsewidth[0] * 0.000008f) * 100.0f) / 100.0f) - 1.5f);
    }

    return 0;
}

// read the period and pulsewidth
float rc_readCh2(void)
{
    // Convert from clock cycle to ms
    rcPeriodCH2 = 2 * period[1] * 0.000008;

    // Check if the rc sender is on
    if (pulsewidth[1] > 0)
    {
        // one clock cycle is 1/125000 ms
        rcPulseWidthCH2 = 2.0f * ((floorf((2.0f * (float)pulsewidth[1] * 0.000008f) * 100.0f) / 100.0f) - 1.5f);
    }

    return 0;
}

void init_systick()
{
    systick_hw->csr = 0;        // Disable
    systick_hw->rvr = 124999U;  // Standard System clock (125Mhz)/ (rvr value + 1) = 1ms
    systick_hw->cvr = 0;        // clear the count to force initial reload
    systick_hw->csr = 0x7;      // Enable Systic, Enable Exceptions
}

// set the pio rc receiver irq handler
void pio_irq_handler()
{
    int state_machine = -1;
    // check which IRQ was raised:
    for (int i = 0; i < 4; i++)
    {
        if (pio0_hw->irq & 1 << i)
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

void initialize_rc_PIO(uint* pin_list, uint num_of_pins)
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
    pio0_hw->inte0 = PIO_IRQ0_INTE_SM0_BITS | PIO_IRQ0_INTE_SM1_BITS | PIO_IRQ0_INTE_SM2_BITS | PIO_IRQ0_INTE_SM3_BITS;
};