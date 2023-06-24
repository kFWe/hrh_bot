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

#define LED_PIN PICO_DEFAULT_LED_PIN
void init_systick();

// Encoder Vars
#define QUADRATURE_A_PIN 14
#define QUADRATURE_B_PIN 15
PIO pio_;
uint sm_ = 1u;

// RC Var
#define RC_CH1_PIN 13
#define RC_CH2_PIN 12
PIO rcPio_;
uint rcsm_ = 0u;
float rc_period_ch1 = 0.0f; // in s
float rc_pulseWidth_ch1 = 0.0f; // in s
float rc_dutyCycle_ch1 = 0.0f;
float rc_period_ch2 = 0.0f; // in s
float rc_pulseWidth_ch2 = 0.0f; // in s
float rc_dutyCycle_ch2 = 0.0f;
float rc_readCh1(void);
float rc_readCh2(void);

// data about the PWM input measured in pio clock cycles
static uint32_t pulsewidth[4], period[4];

// Motor Vars
#define POS_SPEED_BUTTON_PIN 2
#define NEG_SPEED_BUTTON_PIN 3
#define LEFT_MOTOR_PWM 18
#define LEFT_MOTOR_IN1 17
#define LEFT_MOTOR_IN2 16
#define RIGHT_MOTOR_PWM 21
#define RIGHT_MOTOR_IN1 20
#define RIGHT_MOTOR_IN2 19
#define DEBOUNCE_TIME 200000u  // ms

typedef enum Direction
{
    FORWARD,
    BACKWARD
} Direction;
uint32_t currentMotorSpeedLeft_ = 0u;  // pwm
uint32_t currentMotorSpeedRight_ = 0u;  // pwm
uint32_t targetMotorSpeed_ = 0u;
uint32_t currentMotorPosition_ = 0u;
uint32_t targetMotorPosition_ = 0u;
Direction currentMotorDirectionLeft_ = FORWARD;
Direction targetMotorDirectionLeft_ = FORWARD;
Direction currentMotorDirectionRight_ = FORWARD;
Direction targetMotorDirectionRight_ = FORWARD;

void setSpeedLeft(uint8_t dutyCycle);
void setDirectionLeft(Direction motorDirection);
void setSpeedRight(uint8_t dutyCycle);
void setDirectionRight(Direction motorDirection);

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
    rc_readCh1();
    rc_readCh2();
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
            
                targetMotorSpeed_ = 50u;
                setDirectionLeft(FORWARD);
                setSpeedLeft(targetMotorSpeed_);
                setDirectionRight(FORWARD);
                setSpeedRight(targetMotorSpeed_);
           
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
            targetMotorSpeed_ = 50u;
                setDirectionLeft(BACKWARD);
                setSpeedLeft(targetMotorSpeed_);
                setDirectionRight(BACKWARD);
                setSpeedRight(targetMotorSpeed_);
            negButtonPushedLastTime = time_us_32();
            SEGGER_RTT_WriteString(0, "NEG SPEED IRQ\n");
        }
    }

    SEGGER_SYSVIEW_RecordExitISR();
}

bool controlLoop(struct repeating_timer* t)
{
    // rc signal from 0.07 to 0.17. Middle Point at 0.12
    int input_start = 0;    // The lowest number of the range input.
    int input_end = 50;    // The largest number of the range input.
    int output_start = 0; // The lowest number of the range output.
    int output_end = 100;  // The largest number of the range output.
    double slope = 1.0 * (output_end - output_start) / (input_end - input_start);

    int8_t channelNormed = (int8_t)(rc_dutyCycle_ch2*1000);
    int8_t u = output_start + slope * (channelNormed - input_start);

    targetMotorSpeed_ = fabs(u);

    if (targetMotorSpeed_ > 100u)
    {
        targetMotorSpeed_ = 100u;
    }

    targetMotorDirectionLeft_ = FORWARD;
    targetMotorDirectionRight_ = FORWARD;
    if (channelNormed < 0)
    {
        targetMotorDirectionLeft_ = BACKWARD;
        targetMotorDirectionRight_ = BACKWARD;
    }

    setDirectionLeft(targetMotorDirectionLeft_);
    setDirectionRight(targetMotorDirectionRight_);
    setSpeedLeft(targetMotorSpeed_);
    setSpeedRight(targetMotorSpeed_);
    
    return true;
}

// set the irq handler
    static void pio_irq_handler()
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

 // read the period and pulsewidth
float rc_readCh1(void)
{
    // Convert from clock cycle to SI

    // one clock cycle is 1/125000000 seconds
    rc_period_ch1 = period[0] * 0.000000008;
    
    // read_pulsewidth (in seconds)
    
    // one clock cycle is 1/125000000 seconds
    rc_pulseWidth_ch1 = pulsewidth[0] * 0.000000008;
    
    // read_dutycycle (between 0 and 1)
    rc_dutyCycle_ch1 = roundf(((float)pulsewidth[0] / (float)period[0]) * 100.0f) / 100.0f - 0.12f;

    return 0;
}

float rc_readCh2(void)
{
    // Convert from clock cycle to SI

    // one clock cycle is 1/125000000 seconds
    rc_period_ch2 = period[1] * 0.000000008;
    
    // read_pulsewidth (in seconds)
    
    // one clock cycle is 1/125000000 seconds
    rc_pulseWidth_ch2 = pulsewidth[1] * 0.000000008;
    
    // read_dutycycle (between 0 and 1)
    rc_dutyCycle_ch2 = roundf(((float)pulsewidth[1] / (float)period[1])* 100.0f) / 100.0f -0.12f;

    return 0;
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

    // Left Motor Driver
    gpio_init(LEFT_MOTOR_IN1);
    gpio_set_dir(LEFT_MOTOR_IN1, GPIO_OUT);
    gpio_init(LEFT_MOTOR_IN2);
    gpio_set_dir(LEFT_MOTOR_IN2, GPIO_OUT);

    gpio_set_function(LEFT_MOTOR_PWM, GPIO_FUNC_PWM);
    uint16_t slice_num = pwm_gpio_to_slice_num(LEFT_MOTOR_PWM);

    // PWM Freq. 5 kHz
    int32_t f_sys = clock_get_hz(clk_sys);
    float divider = f_sys / 25000U;
    pwm_set_clkdiv(slice_num, divider);
    pwm_set_wrap(slice_num, 0xFFFF);

    // Set default start direction (left)
    setDirectionLeft(currentMotorDirectionLeft_);

    // Right Motor Driver
    gpio_init(RIGHT_MOTOR_IN1);
    gpio_set_dir(RIGHT_MOTOR_IN1, GPIO_OUT);
    gpio_init(RIGHT_MOTOR_IN2);
    gpio_set_dir(RIGHT_MOTOR_IN2, GPIO_OUT);

    gpio_set_function(RIGHT_MOTOR_PWM, GPIO_FUNC_PWM);
    uint16_t slice_num_right = pwm_gpio_to_slice_num(RIGHT_MOTOR_PWM);

    // PWM Freq. 5 kHz
    pwm_set_clkdiv(slice_num_right, divider);
    pwm_set_wrap(slice_num_right, 0xFFFF);

    // Set default start direction (left)
    setDirectionRight(currentMotorDirectionRight_);

    // Quadrature Encoder PIO init. Position get's read every 1ms in systick irq
    pio_ = pio1;
    uint16_t offset = pio_add_program(pio_, &quadrature_program);
    sm_ = pio_claim_unused_sm(pio_, true);
    quadrature_program_init(pio_, sm_, offset, QUADRATURE_A_PIN, QUADRATURE_B_PIN);

    // Read RC signal for left channel
    uint pin_list[2] = {13, 12};
    initialize_RC_PIO(pin_list, 2);

    // Setup the control loop
    struct repeating_timer timer;
    add_repeating_timer_ms(1, controlLoop, NULL, &timer);

    while (true)
    {
        gpio_put(LED_PIN, 1);
        sleep_us(2500);
        gpio_put(LED_PIN, 0);
        sleep_us(2500);
        
        printf("%.8f \t %.8f\n", rc_dutyCycle_ch1, rc_dutyCycle_ch2);
        //printf("%d \t %d \t %d \t %d\n", currentMotorPosition_, targetMotorPosition_, currentMotorSpeedLeft_, currentMotorSpeedRight_);
    }
}

void setSpeedLeft(uint8_t dutyCycle)
{
    currentMotorSpeedLeft_ = dutyCycle;  // TODO: update speed from encoder read values

    uint16_t slice_num = pwm_gpio_to_slice_num(LEFT_MOTOR_PWM);
    uint16_t duty = dutyCycle;                     // duty cycle, in percent
    uint16_t level = (0xFFFF - 1u) * duty / 100u;  // calculate channel level from given duty cycle in %
    pwm_set_chan_level(slice_num, 0, level);
    pwm_set_enabled(slice_num, true);
}

void setSpeedRight(uint8_t dutyCycle)
{
    currentMotorSpeedRight_ = dutyCycle;  // TODO: update speed from encoder read values

    uint16_t slice_num = pwm_gpio_to_slice_num(RIGHT_MOTOR_PWM);
    uint16_t duty = dutyCycle;                     // duty cycle, in percent
    uint16_t level = (0xFFFF - 1u) * duty / 100u;  // calculate channel level from given duty cycle in %
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