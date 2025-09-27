#ifndef BSP_H
#define BSP_H

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "ht16k33.h"
#include "mma8452q.h"

/**
 * @brief Enable if CN1 should be configured for UART (Pins 0 and 1).
 */
#define CN1_UART

/**
 * @brief CS pin of the PSRAM chip.
 */
#define PSRAM_CS    8

/**
 * @brief LEDs directly connected to the MCU.
 */
#define LED_GREEN   25
#define LED_YELLOW  24
#define LED_RED     23

/**
 * @brief I2C pins used.
 */
#define I2C_SDA     20
#define I2C_SCL     21
#define I2C_PORT    i2c0

/**
 * @brief Shift register.
 */
#define SR_OE       9
#define SR_SD       11
#define SR_SHCP     10
#define SR_STCP     12
#define SPI_PORT    spi1

/**
 * @brief Push buttons.
 */
#define SW_5        22
#define SW_6        15
#define SW_7        16
#define SW_8        17

/**
 * @brief Switches.
 */
#define SW_10     26
#define SW_11     27
#define SW_12     28
#define SW_13     29
#define SW_14     2
#define SW_15     3
#define SW_16     13
#define SW_17     14

/**
 * @brief Interrupt pins of the accelerometer (main connection via I2C).
 */
#define ACC_INT1    18
#define ACC_INT2    19

/**
 * @brief Pins connected to CN1.
 *
 * CN1_0 -> GPIO4 | UART1_TX | I2C0_SDA | SPI0_RX
 * CN1_1 -> GPIO5 | UART1_RX | I2C0_SCL | SPI0_CSn
 * CN1_2 -> GPIO6 | I2C1_SDA | SPI0_SCK
 * CN1_3 -> GPIO7 | I2C1_SCL | SPI0_TX
 */
#define CN1_0   4
#define CN1_1   5
#define CN1_2   6
#define CN1_3   7

/**
 * @brief Baudrate used for UART1 on CN1 if enabled.
 */
#define CN1_UART_BAUD_RATE  115200

/**
 * @brief Enum used to select different axis of the accelerometer.
 *
 */
typedef enum {
    X_AXIS = 0,
    Y_AXIS,
    Z_AXIS
} axis_t;

/**
 * @brief Initialization function of the BSP.
 * Initializes all peripherals on the Lab-Kit.
 * - Shift register that connect to the LEDs.
 * - Directly connected LEDs.
 * - Buttons.
 * - Switches.
 * - 7-segment display driver.
 * - Accelerometer.
 */
void BSP_Init(void);

/**
 * @brief Writes 3 byte to the shift register LEDs.
 * The implementation corrects the order of the data for the current prototype PCBs.
 * @param data Pointer to the data.
 */
void BSP_ShiftRegWriteAll(uint8_t* data);

/**
 * @brief Sets the state of a single shift register LED.
 *
 * @param nr Number of the shift register LED.
 * @param state State of the LED.
 */
void BSP_ShiftRegisterSetLED(uint8_t nr, bool state);

/**
 * @brief Set the brightness of the shift register LEDs.
 *
 * @param value Brightness, between 0 and 100.
 */
void BSP_ShiftRegisterSetBrightness(uint8_t value);

/**
 * @brief Sets the value of an LED that is connected to a GPIO pin.
 *
 * @param gpio GPIO pin of the LED.
 * @param value State of the LED.
 */
void BSP_SetLED(uint32_t gpio, bool value);

/**
 * @brief Toggles the state of a LED that is connected to a GPIO pin.
 *
 * @param gpio GPIO pin of the LED.
 */
void BSP_ToggleLED(uint32_t gpio);

/**
 * @brief Function reads the state of a switch or button.
 *
 * @param gpio GPIO pin of the switch/button.
 * @return true
 * @return false
 */
bool BSP_GetInput(uint32_t gpio);

/**
 * @brief Returns the current acceleration (in g) for the selected axis.
 *
 * @param axis Axis to measure.
 * @return float Acceleration in g.
 */
float BSP_GetAxisAcceleration(axis_t axis);

/**
 * @brief Reads the acceleration of each axis and converts the values to g.
 *
 * @param x X-axis.
 * @param y Y-axis.
 * @param z Y-axis.
 * @return true Measurement successful.
 * @return false Measurement failed.
 */
bool BSP_GetAcceleration(float* x, float* y, float* z);

/**
 * @brief Reads the tap (single and double) and its direction. 
 * 
 * Note: Double tap not configured...
 * @return int8_t +/- 1 for single tap, and =/- 2 for double tap.
 */
int8_t BSP_GetTapCount(void);

/**
 * @brief Set the brightness of the 7-segment display.
 * Note that even 0 is not off!
 * @param level Brightness level [0-15].
 * @return true
 * @return false
 */
bool BSP_7SegBrightness(uint8_t level);

/**
 * @brief Clears the 7-segment display.
 *
 */
void BSP_7SegClear(void);

/**
 * @brief Write a string to the 7-segment display.
 *
 * @param string String to display, max length 4 characters!
 */
void BSP_7SegDispString(char* string);

/**
 * @brief Display an integer on the 7-segment display.
 *
 * @param value Value to display.
 */
void BSP_7SegDispInt(int32_t value);

/**
 * @brief Display a floating point number on the 7-segment display with two decimal places.
 *
 * @param value Number to display.
 */
void BSP_7SegDispFloat(float value);

/**
 * @brief Function to check if PSRAM is available.
 *
 * @return size_t Returns the PSRAM size or 0 if not available.
 */
size_t BSP_HasPSRAM(void);

/**
 * @brief Waiting function (clock cycles).
 *
 * @param args Number of clock cycles (uint32_t).
 */
void BSP_WaitClkCycles(uint32_t n);

#endif /* BSP_H */
