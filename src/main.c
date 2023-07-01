#include <float.h>
#include <math.h>
#include <stdio.h>
#include "PwmIn.pio.h"
#include "SEGGER_RTT.h"
#include "SEGGER_SYSVIEW.h"
#include "hardware/clocks.h"
#include "hardware/i2c.h"
#include "hardware/irq.h"
#include "hardware/pio.h"
#include "hardware/pwm.h"
#include "hardware/structs/systick.h"
#include "pico/stdlib.h"
#include "quadrature.pio.h"

/***********************************
 * Board Config
 ************************************/

// RPI Pico

#define RC_CH1_PIN 0
#define RC_CH2_PIN 1
#define RC_CH3_PIN 2
#define LEFT_MOTOR_PHASE 3
#define LEFT_MOTOR_PWM 4
#define RIGHT_MOTOR_PHASE 5
#define RIGHT_MOTOR_PWM 6
#define DRV8835_MODE 7

#define MPU6050_I2C 1
#define MPU6050_SDA 10
#define MPU6050_SCL 11
#define MPU6050_INT_PIN 12

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

// MPU Vars
bool isMpuPresent_ = true;
i2c_inst_t* mpu_i2c_;
uint8_t mpu_addr_ = 0x68;  // 0x68;

// RC Var
PIO rcPio_;
uint rcsm_ = 0u;
float rcPeriodCH1 = 0.0f;      // in ms
float rcPulseWidthCH1 = 0.0f;  // in ms
float rcPeriodCH2 = 0.0f;      // in ms
float rcPulseWidthCH2 = 0.0f;  // in ms
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
bool isFlipped_ = false;
uint8_t currentMotorSpeedLeft_ = 0u;   // pwm
uint8_t currentMotorSpeedRight_ = 0u;  // pwm
Direction currentMotorDirectionLeft_ = FORWARD;
Direction targetMotorDirectionLeft_ = FORWARD;
Direction currentMotorDirectionRight_ = FORWARD;
Direction targetMotorDirectionRight_ = FORWARD;
MixStrategy mixStrategy_ = NONE;

/***********************************
 * Functions
 ************************************/
void init_systick();
void set_pwm(uint8_t dutyCycle, uint8_t pwmPin);
void set_direction_left(Direction motorDirection);
void set_direction_right(Direction motorDirection);
float rc_readCh1(void);
float rc_readCh2(void);
float rc_readCh3(void);
void initialize_rc_PIO(uint* pin_list, uint num_of_pins);
void init_mpu_i2c();
void mpu6050_reset();
void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t* temp);

// 1ms Systick irq. Used to update the static vars for the rc channel data
extern void isr_systick()
{
    SEGGER_SYSVIEW_RecordEnterISR();
    rc_readCh1();
    rc_readCh2();
    rc_readCh3();
    SEGGER_SYSVIEW_RecordExitISR();
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
            mixedLeft_ = abs(ch2Normed);
            mixedRight_ = abs(ch2Normed);
            if (ch2Normed == 0)
            {
                // Turn at full speed
                leftMotorTargetSpeed_ = abs(ch1Normed);
                rightMotorTargetSpeed_ = abs(ch1Normed);
                break;
            }

            // Max range for channel is 0 to 100. We need to limit the exponential function around those to points
            // x_norm = ch1/100
            // y = x_norm^a * 100
            // Exponent a > 1
            float a = 3.0f;
            float x_normed = (float)abs(ch1Normed) / 100.0f;
            float y = pow(x_normed, a) * 100.0f;

            int8_t mix = abs(ch2Normed) - abs((int)y);

            // Right Turn
            if (ch1Normed < 0)
            {
                // Driving right
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
                // Driving right
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

    // Max & Min
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

    SEGGER_SYSVIEW_Conf();
    SEGGER_SYSVIEW_OnIdle();

    SEGGER_RTT_ConfigUpBuffer(0, NULL, NULL, 0, SEGGER_RTT_MODE_BLOCK_IF_FIFO_FULL);

    SEGGER_RTT_WriteString(0, "SEGGER Real-Time-Terminal Sample\r\n\r\n");

    // IMU MPU6050
    init_mpu_i2c();
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
    uint pinList[3] = {RC_CH1_PIN, RC_CH2_PIN, RC_CH3_PIN};
    initialize_rc_PIO(pinList, 3);

    // Setup the control loop
    struct repeating_timer timer;
    add_repeating_timer_ms(1, controlLoop, NULL, &timer);

    int16_t acceleration[3], gyro[3], temp;
    while (true)
    {
        if (isMpuPresent_)
        {
            mpu6050_read_raw(acceleration, gyro, &temp);
        }

        printf("%.8f \t %.8f \t %d \t %d\n", rcPulseWidthCH1, rcPulseWidthCH2, leftMotorTargetSpeed_,
               rightMotorTargetSpeed_);
        // // ACC X Y Z
        // printf("%d \t%d \t%d\t", acceleration[0], acceleration[1], acceleration[2]);

        // // // GYRO X Y Z
        // printf("%d \t%d \t%d\n", gyro[0], gyro[1], gyro[2]);

        // // Temperature is simple so use the datasheet calculation to get deg C.
        // // Note this is chip temperature.

        // // Temp
        // printf("%f\n", (temp / 340.0) + 36.53);
    }
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
            gpio_put(LEFT_MOTOR_PHASE, false);
            break;
        }
        case BACKWARD:
        {
            gpio_put(LEFT_MOTOR_PHASE, true);
            break;
        }
        default:
        {
            gpio_put(LEFT_MOTOR_PHASE, false);
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
            gpio_put(RIGHT_MOTOR_PHASE, false);
            break;
        }
        case BACKWARD:
        {
            gpio_put(RIGHT_MOTOR_PHASE, true);
            break;
        }
        default:
        {
            // Left
            gpio_put(RIGHT_MOTOR_PHASE, false);
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

float rc_readCh3(void)
{
    // Convert from clock cycle to ms
    static float lastLevel = 0.0f;
    float ch3 = 2 * period[2] * 0.000008;

    // Check if the rc sender is on
    if (pulsewidth[2] > 0)
    {
        // one clock cycle is 1/125000 ms
        ch3 = (floorf((2.0f * (float)pulsewidth[2] * 0.000008f) * 100.0f) / 100.0f);
    }

    // Detect level change that signals a button press
    if (fabs(lastLevel - ch3) > FLT_EPSILON * FLT_MIN)
    {
        mixStrategy_++;
        if (mixStrategy_ >= LAST)
        {
            mixStrategy_ = NONE;
        }
    }
    lastLevel = ch3;
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

void init_mpu_i2c()
{
    mpu_i2c_ = i2c1;
    i2c_init(mpu_i2c_, 400 * 1000);
    gpio_set_function(MPU6050_SDA, GPIO_FUNC_I2C);
    gpio_set_function(MPU6050_SCL, GPIO_FUNC_I2C);

    mpu6050_reset();
}
void mpu6050_reset()
{
    // Two byte reset. First byte register, second byte data
    // There are a load more options to set up the device in different ways that could be added here
    uint8_t buf[] = {0x6B, 0x00};
    int res = i2c_write_blocking(mpu_i2c_, mpu_addr_, buf, 2, false);
    if (res == PICO_ERROR_GENERIC)
    {
        isMpuPresent_ = false;
        printf("No MPU Present!\n");
    }
}

void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t* temp)
{
    // For this particular device, we send the device the register we want to read
    // first, then subsequently read from the device. The register is auto incrementing
    // so we don't need to keep sending the register we want, just the first.

    uint8_t buffer[6];

    // Start reading acceleration registers from register 0x3B for 6 bytes
    uint8_t val = 0x3B;
    int res = i2c_write_blocking(mpu_i2c_, mpu_addr_, &val, 1, true);  // true to keep master control of bus

    res = i2c_read_blocking(mpu_i2c_, mpu_addr_, buffer, 6, false);

    for (int i = 0; i < 3; i++)
    {
        accel[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }

    // Now gyro data from reg 0x43 for 6 bytes
    // The register is auto incrementing on each read
    val = 0x43;
    res = i2c_write_blocking(mpu_i2c_, mpu_addr_, &val, 1, true);
    res = i2c_read_blocking(mpu_i2c_, mpu_addr_, buffer, 6, false);  // False - finished with bus

    for (int i = 0; i < 3; i++)
    {
        gyro[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
        ;
    }

    // Now temperature from reg 0x41 for 2 bytes
    // The register is auto incrementing on each read
    val = 0x41;
    res = i2c_write_blocking(mpu_i2c_, mpu_addr_, &val, 1, true);
    res = i2c_read_blocking(mpu_i2c_, mpu_addr_, buffer, 2, false);  // False - finished with bus

    *temp = buffer[0] << 8 | buffer[1];
}