#include <stdio.h>
#include "hardware/pwm.h"
#include "hardware/uart.h"
#include "psram.h"
#include "bsp.h"

/* Add the pragma while debugging. */
#pragma GCC optimize ("O0")

/**
 * @brief Last data written to the shift registers.
 * State of the shift registers, used to be able to modify individual LEDs.
 */
static uint32_t sr_data;

/**
 * @brief Size of the PSRAM, if available, 0 otherwise.
 */
static size_t psram_size;

/**
 * @brief Inidcate if the accelerometer was successfully initialized.
 */
static bool mma8452q_initialized;

/**
 * @brief Accelerator instance.
 */
static mma8452_t acc;

void BSP_Init(void) {

    /*
     * Initialize the Pico SDK.
     */
    stdio_init_all();

    /*
     * Initialize the directly connected LEDs.
     */
    gpio_init(LED_GREEN);
    gpio_set_dir(LED_GREEN, GPIO_OUT);

    gpio_init(LED_YELLOW);
    gpio_set_dir(LED_YELLOW, GPIO_OUT);

    gpio_init(LED_RED);
    gpio_set_dir(LED_RED, GPIO_OUT);

    /*
     * Initialize the buttons.
     */
    gpio_init(SW_5);
    gpio_set_dir(SW_5, GPIO_IN);
    gpio_pull_up(SW_5);

    gpio_init(SW_6);
    gpio_set_dir(SW_6, GPIO_IN);
    gpio_pull_up(SW_6);

    gpio_init(SW_7);
    gpio_set_dir(SW_7, GPIO_IN);
    gpio_pull_up(SW_7);

    gpio_init(SW_8);
    gpio_set_dir(SW_8, GPIO_IN);
    gpio_pull_up(SW_8);

    /*
     * Initialize the switches.
     */
    gpio_init(SW_10);
    gpio_set_dir(SW_10, GPIO_IN);
    gpio_pull_up(SW_10);

    gpio_init(SW_11);
    gpio_set_dir(SW_11, GPIO_IN);
    gpio_pull_up(SW_11);

    gpio_init(SW_12);
    gpio_set_dir(SW_12, GPIO_IN);
    gpio_pull_up(SW_12);

    gpio_init(SW_13);
    gpio_set_dir(SW_13, GPIO_IN);
    gpio_pull_up(SW_13);

    gpio_init(SW_14);
    gpio_set_dir(SW_14, GPIO_IN);
    gpio_pull_up(SW_14);

    gpio_init(SW_15);
    gpio_set_dir(SW_15, GPIO_IN);
    gpio_pull_up(SW_15);

    gpio_init(SW_16);
    gpio_set_dir(SW_16, GPIO_IN);
    gpio_pull_up(SW_16);

    gpio_init(SW_17);
    gpio_set_dir(SW_17, GPIO_IN);
    gpio_pull_up(SW_17);

    /*
     * Initialize the shift register that connects to the LED square.
     */
    spi_init(SPI_PORT, 1 * 1000 * 1000);            /* 1MHz */
    gpio_set_function(SR_SD, GPIO_FUNC_SPI);    /* MOSI */
    gpio_set_function(SR_SHCP, GPIO_FUNC_SPI);  /* CLK */

    //gpio_init(SR_MR);
    //gpio_set_dir(SR_MR, GPIO_OUT);
    //gpio_put(SR_MR, true);                      /* Negated */


    gpio_init(SR_OE);
    gpio_set_dir(SR_OE, GPIO_OUT);
    gpio_set_function(SR_OE, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(SR_OE);
    pwm_set_wrap(slice_num, 100);
    pwm_set_gpio_level(SR_OE, 0);
    pwm_set_enabled(slice_num, true);
    //gpio_put(SR_OE, false);                     /* Negated */

    gpio_init(SR_STCP);
    gpio_set_dir(SR_STCP, GPIO_OUT);
    gpio_put(SR_OE, false);

    sr_data = 0x00;

    BSP_ShiftRegWriteAll((uint8_t*)&sr_data);

    /*
     * Initialize the I2C bus that connects to the 7-segment driver
     * and to the accelerometer.
     */
    i2c_init(I2C_PORT, 100 * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    /*
     * Initialize the 7-segment driver and the accelerometer.
     */
    ht16k33_init();

    if (mma8452q_init(&acc)) {
        mma8452q_initialized = true;
    }

#ifdef CN1_UART
    uart_init(uart1, CN1_UART_BAUD_RATE);
    gpio_set_function(CN1_0, UART_FUNCSEL_NUM(uart1, CN1_0));
    gpio_set_function(CN1_1, UART_FUNCSEL_NUM(uart1, CN1_1));
#endif

    /*
     * Initialize the PSRAM chip. If it is available, a size > 0 is returned indicating the PSRAM size in bytes.
     */
    psram_size = psram_setup(PSRAM_CS);

}
/*-----------------------------------------------------------*/

void BSP_ShiftRegWriteAll(uint8_t* data) {

    sr_data = data[2] | (data[1] << 8) | (data[0] << 16);

    spi_write_blocking (SPI_PORT, (uint8_t*) &sr_data, 3);
    gpio_put(SR_STCP, true);
    gpio_put(SR_STCP, false);
}
/*-----------------------------------------------------------*/

void BSP_ShiftRegisterSetLED(uint8_t nr, bool state) {
    /* SW fix to account for the wrong order of SR on prototype board.*/
    if (7 < nr && nr < 16) {
        nr = nr + 8;
    } if (15 < nr && nr < 24) {
        nr = nr - 8;
    }

    if (nr < 24) {  /* There are only 24 LED. */
        if (state == true) {
            sr_data = sr_data | (1 << nr);
        } else {
            sr_data = sr_data & ~(1 << nr);
        }

        /* Instead of calling BSP_ShiftRegWriteAll() we call SPI directly due ot the SW fix. */
        spi_write_blocking (SPI_PORT, (uint8_t*) &sr_data, 3);
        gpio_put(SR_STCP, true);
        gpio_put(SR_STCP, false);
    }
}
/*-----------------------------------------------------------*/

void BSP_ShiftRegisterSetBrightness(uint8_t value) {
    if (value > 100) value = 100;
    value = 100 - value;    /* As it is negated. */
    pwm_set_gpio_level(SR_OE, value);
}
/*-----------------------------------------------------------*/

void BSP_SetLED(uint32_t gpio, bool value) {
    gpio_put(gpio, value);
}
/*-----------------------------------------------------------*/

void BSP_ToggleLED(uint32_t gpio) {
   gpio_put(gpio, !gpio_get(gpio));
}
/*-----------------------------------------------------------*/

bool BSP_GetInput(uint32_t gpio) {
    return gpio_get(gpio);
}
/*-----------------------------------------------------------*/

float BSP_GetAxisAcceleration(axis_t axis) {
/*   uint16_t data;
    uint8_t axis_dir;

    if (mma8452q_initialized == false) return false;

    if (axis == X_AXIS) {
        axis_dir = MMA8452Q_AXIS_X;
    } else if (axis == Y_AXIS) {
        axis_dir = MMA8452Q_AXIS_Y;
    } else if (axis == Z_AXIS) {
        axis_dir = MMA8452Q_AXIS_Z;
    }

    if (mma8452q_read_axis_data(&data, axis_dir)) {
        return convert_to_g(data);
    } else {
        return 0.0f;
    }*/

    if (mma8452q_initialized == false) return false;

    if (axis == X_AXIS) {
        return mma8452q_getCalculatedX(&acc);
    } else if (axis == Y_AXIS) {
        return mma8452q_getCalculatedY(&acc);
    } else if (axis == Z_AXIS) {
        return mma8452q_getCalculatedZ(&acc);
    }
    

}
/*-----------------------------------------------------------*/

int8_t BSP_GetTapCount(void) {
    uint8_t val = mma8452q_readTap(&acc);
    int8_t num = 1;

    if (val & (1 << 6)) {   /* Check that it was the Z-axis that registered the tap. */

        /*if (val & (1 << 3)) {
            num = 2;
        }*/

        if (val & (1 << 2)) {
            return num;
        } else {
            return -1 * num;
        }
    }

    return 0;
}
/*-----------------------------------------------------------*/

bool BSP_GetAcceleration(float* x, float* y, float* z) {

    /* The commented version has some problem which requires debugging... Temporary fix for now.*/
    // uint16_t raw_x, raw_y, raw_z;

    // if (mma8452q_read_data(&raw_x, &raw_y, &raw_z)) {
    //     *x = convert_to_g(raw_x);
    //     *y = convert_to_g(raw_y);
    //     *z = convert_to_g(raw_z);
    //     return true;
    // }  else {
    //     return false;
    // }

    *x = BSP_GetAxisAcceleration(X_AXIS);
    *y = BSP_GetAxisAcceleration(Y_AXIS);
    *z = BSP_GetAxisAcceleration(Z_AXIS);

    return true;
}
/*-----------------------------------------------------------*/

bool BSP_7SegBrightness(uint8_t level) {
    if (level > 15) return false;

    ht16k33_set_brightness(level);

    return true;
}
/*-----------------------------------------------------------*/

void BSP_7SegClear(void) {
    ht16k33_clear_all();
}
/*-----------------------------------------------------------*/

void BSP_7SegDispString(char* string) {
    ht16k33_display_string(string);
}
/*-----------------------------------------------------------*/

void BSP_7SegDispInt(int32_t value) {
    char dspStrng[9]; /* with decimal points and null termination, the maximum array size needed is 9. */

    sprintf(dspStrng, "%i", value, sizeof(dspStrng));

}
/*-----------------------------------------------------------*/

void BSP_7SegDispFloat(float value) {
    char dspStrng[9]; /* with decimal points and null termination, the maximum array size needed is 9. */

    sprintf(dspStrng, "% 4.2f", value, sizeof(dspStrng));
}
/*-----------------------------------------------------------*/

size_t BSP_HasPSRAM(void) {
    return psram_size;
}
/*-----------------------------------------------------------*/

void BSP_WaitClkCycles( uint32_t n ) {
    uint32_t l;

    if (n == 0) return; /* No delay specified, will still take some cycles... */
    l = n/3;            /* Cycles per loop is 3. */
    asm volatile( "0:" "SUBS %[count], 1;" "BNE 0b;" :[count]"+r"(l) );
}
/*-----------------------------------------------------------*/
