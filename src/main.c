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
#define LEFT_MOTOR_IN1 3
#define LEFT_MOTOR_IN2 4
#define LEFT_MOTOR_PWM 5
#define RIGHT_MOTOR_IN1 6
#define RIGHT_MOTOR_IN2 7
#define RIGHT_MOTOR_PWM 8

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
uint16_t pwm_wrap_ = 12500u;
uint8_t pwm_clk_div_ = 2u;
bool mixing_ = true;

// MPU Vars
i2c_inst_t* mpu_i2c_;
uint8_t mpu_addr_ = 0x68;

// RC Var
PIO rcPio_;
uint rcsm_ = 0u;
float rc_period_ch1 = 0.0f;      // in s
float rc_pulseWidth_ch1 = 0.0f;  // in s
float rc_dutyCycle_ch1 = 0.0f;
float rc_period_ch2 = 0.0f;      // in s
float rc_pulseWidth_ch2 = 0.0f;  // in s
float rc_dutyCycle_ch2 = 0.0f;
static uint32_t pulsewidth[4], period[4];

// Motor Vars
typedef enum Direction
{
    FORWARD,
    BACKWARD
} Direction;
typedef enum MixStrategy
{
    ADD,
    LINEAR,
    EXPONENTIAL
}MixStrategy;
bool isFlipped = false;
uint8_t currentMotorSpeedLeft_ = 0u;   // pwm
uint8_t currentMotorSpeedRight_ = 0u;  // pwm
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
void initialize_RC_PIO(uint* pin_list, uint num_of_pins);
void init_mpu_i2c();
void mpu6050_reset();
void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t* temp);

// 1ms Systick irq. Used to update the static vars for the rc channel data
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
    // int input_start = 0;   // The lowest number of the range input.
    // int input_end = 50;    // The largest number of the range input.
    // int output_start = 0;  // The lowest number of the range output.
    // int output_end = 100;  // The largest number of the range output.
    // double slope = 1.0 * (output_end - output_start) / (input_end - input_start);

    // rc signal used is the pulse width of the rc receiver which was normed to be -0.5 to 0.5, with 0.0 beeing the
    // middle point.
    int8_t ch1Normed = (int8_t)(rc_pulseWidth_ch1 * 100);
    int8_t ch2Normed = (int8_t)(rc_pulseWidth_ch2 * 100);

    if (mixing_)
    {
        int8_t mixedLeft = (ch1Normed + ch2Normed);
        int8_t mixedRight = (ch1Normed - ch2Normed);

        // Calculate the duty cycle for the pwm used to control the motor and map the rc range (0 to 50 to 0 to 100)
        // leftMotorTargetSpeed_ = fabs(output_start + slope * (mixedLeft - input_start));
        // rightMotorTargetSpeed_ = fabs(output_start + slope * (mixedRight - input_start));
        leftMotorTargetSpeed_ = abs(mixedLeft);
        rightMotorTargetSpeed_ = abs(mixedRight);

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
        // leftMotorTargetSpeed_ = fabs(output_start + slope * (ch2Normed - input_start));
        // rightMotorTargetSpeed_ = fabs(output_start + slope * (ch2Normed - input_start));
        leftMotorTargetSpeed_ = ch2Normed;
        rightMotorTargetSpeed_ = ch2Normed;

        targetMotorDirectionLeft_ = FORWARD;
        targetMotorDirectionRight_ = FORWARD;
        if (ch2Normed < 0)
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

    // IMU MPU6050
    //init_mpu_i2c();

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

    // Init RC input reader
    uint pin_list[2] = {RC_CH1_PIN, RC_CH2_PIN};
    initialize_RC_PIO(pin_list, 2);

    // Setup the control loop
    struct repeating_timer timer;
    add_repeating_timer_ms(1, controlLoop, NULL, &timer);

    int16_t acceleration[3], gyro[3], temp;
    while (true)
    {
        sleep_ms(1);
        //mpu6050_read_raw(acceleration, gyro, &temp);
        printf("%.8f \t %.8f \t %d \t %d \n", rc_pulseWidth_ch1, rc_pulseWidth_ch2, leftMotorTargetSpeed_,
               rightMotorTargetSpeed_);
        // // ACC X Y Z
        // printf("%d \t %d \t %d \t", acceleration[0], acceleration[1], acceleration[2]);

        // // GYRO X Y Z
        // printf("%d \t %d \t %d \t", gyro[0], gyro[1], gyro[2]);

        // // Temperature is simple so use the datasheet calculation to get deg C.
        // // Note this is chip temperature.

        // // Temp
        // printf("%f\n", (temp / 340.0) + 36.53);
    }
}

void setSpeedLeft(uint8_t dutyCycle)
{
    currentMotorSpeedLeft_ = dutyCycle;  // TODO: update speed from encoder read values

    uint16_t slice_num = pwm_gpio_to_slice_num(LEFT_MOTOR_PWM);
    uint16_t duty = dutyCycle;                        // duty cycle, in percent
    uint16_t level = (pwm_wrap_ - 1u) * duty / 100u;  // calculate channel level from given duty cycle in %
    pwm_set_chan_level(slice_num, pwm_gpio_to_channel(LEFT_MOTOR_PWM), level);
    pwm_set_enabled(slice_num, true);
}

void setSpeedRight(uint8_t dutyCycle)
{
    currentMotorSpeedRight_ = dutyCycle;  // TODO: update speed from encoder read values

    uint16_t slice_num = pwm_gpio_to_slice_num(RIGHT_MOTOR_PWM);
    uint16_t duty = dutyCycle;                        // duty cycle, in percent
    uint16_t level = (pwm_wrap_ - 1u) * duty / 100u;  // calculate channel level from given duty cycle in %
    pwm_set_chan_level(slice_num, pwm_gpio_to_channel(RIGHT_MOTOR_PWM), level);
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

    // Check if the rc sender is on
    if (pulsewidth[0] > 0)
    {
        // one clock cycle is 1/125000 ms
        rc_pulseWidth_ch1 = 2.0f*((floorf((2.0f * (float)pulsewidth[0] * 0.000008f) * 100.0f) / 100.0f) - 1.5f);
    }
    // read_dutycycle (between 0 and 1)
    rc_dutyCycle_ch1 = roundf(((float)pulsewidth[0] / (float)period[0]) * 100.0f) / 100.0f - 0.12f;

    return 0;
}

float rc_readCh2(void)
{
    // Convert from clock cycle to ms
    rc_period_ch2 = 2 * period[1] * 0.000008;

    // Check if the rc sender is on
    if (pulsewidth[0] > 0)
    {
        // one clock cycle is 1/125000 ms
        rc_pulseWidth_ch2 = 2.0f*((floorf((2.0f * (float)pulsewidth[1] * 0.000008f) * 100.0f) / 100.0f) - 1.5f);
    }

    // read_dutycycle (between 0 and 1)
    rc_dutyCycle_ch2 = roundf(((float)pulsewidth[1] / (float)period[1]) * 100.0f) / 100.0f - 0.12f;

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

void initialize_RC_PIO(uint* pin_list, uint num_of_pins)
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
    // gpio_pull_up(MPU6050_SDA);
    // gpio_pull_up(MPU6050_SCL);

    mpu6050_reset();
}
void mpu6050_reset()
{
    // Two byte reset. First byte register, second byte data
    // There are a load more options to set up the device in different ways that could be added here
    uint8_t buf[] = {0x6B, 0x80};
    i2c_write_blocking(mpu_i2c_, mpu_addr_, buf, 2, false);
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
    if (res == PICO_ERROR_GENERIC)
    {
        printf("No MPU Present!\n");
    }
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