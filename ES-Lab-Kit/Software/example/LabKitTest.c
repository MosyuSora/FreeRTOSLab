/* Add the pragma while debugging. */
#pragma GCC optimize ("O0")

#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "bsp.h"

/**
 * @brief Configure task periods here. All periods in ms.
 */
#define LED_PERIOD      200
#define INPUT_PERIOD    50
#define BLINK_PERIOD    1000
#define UART_PERIOD     50
#define CN1_TX_PERIOD   1000
#define PSRAM_PERIOD    500

/**
 * @brief How many samples are collected at UART_PERIOD for the accelerator moving average.
 */
#define ACC_SAMPLES 5

typedef struct {
    bool    state;
    uint8_t btn;
} btn_evt_t;
/*-----------------------------------------------------------*/

TaskHandle_t        ledTsk;                 /* Handle for the LED task. */
TaskHandle_t        dispTsk;                /* Handle for the display task. */
TaskHandle_t        inputTsk;               /* Handle for the input task. */
TaskHandle_t        blinkTsk;               /* Handle for the blink task. */
TaskHandle_t        uartTsk;                /* Handle for the UART task. */
TaskHandle_t        cn1RxTsk;               /* Handle for the task that sends data via CN1 UART. */
TaskHandle_t        cn1TxTsk;               /* Handle for the task that receives data via CN1 UART. */
TaskHandle_t        psramTsk;               /* Handle for the PSRAM task. */
QueueHandle_t       btnQueue;               /* Handle for the button queue. */

SemaphoreHandle_t   mutex_brightness;       /* Mutex to protect the global variable sr_brightness. */
int8_t              sr_brightness = 50;     /* Brightness of the shift register LEDs. */

uint32_t            valIn;                  /* Local value that is incremented and send over CN1 UART. */
uint32_t            valOut;                 /* Value that is received over CN1 UART. */
SemaphoreHandle_t   mutex_valIn;            /* Mutex to protect valIn. */
SemaphoreHandle_t   mutex_valOut;           /* Mutex to protect valOut. */

float               xSamples[ACC_SAMPLES];
float               ySamples[ACC_SAMPLES];
float               zSamples[ACC_SAMPLES];
uint32_t            lastPos = ACC_SAMPLES - 1;
float               g_xVal;
float               g_yVal;
float               g_zVal;
int32_t            tapCount = 0;
SemaphoreHandle_t   accMutex;

/*-----------------------------------------------------------*/

/**
 * @brief 
 * 
 * @param args Task period (uint32_t).
 */
void led_task(void *args);

/**
 * @brief 
 * 
 * @param args Task period (uint32_t).
 */
void disp_task(void* args);

/**
 * @brief The input task sends an event to the button queue if the state of a button changes and samples accelerometer data.
 * 
 * @param args Task period (uint32_t).
 */
void input_task(void* args);

/**
 * @brief The blink task blinks the red/green/yellow LEDs.
 * 
 * @param args Task period (uint32_t).
 */
void blink_task(void* args);

/**
 * @brief The UART task periodically sends the state of all inputs as CSV-string.
 * 
 * @param args Task period (uint32_t).
 */
void uart_task(void* args);

/**
 * @brief The task receives an integer from UART1 (CN1) and updates the global variable valIn.
 * 
 * @param args Task period (uint32_t).
 */
void cn1Rx_task(void* args);

/**
 * @brief The task increments valOut and sends it on UART1 (CN1).
 * 
 * @param args Task period (uint32_t).
 */
void cn1Tx_task(void* args);

/**
 * @brief Reads 1MB and saves the time to a global variable. 
 * 
 * @param args Task period (uint32_t).
 */
void psram_task(void* args);

/*-----------------------------------------------------------*/

/**
 * @brief Main function.
 * 
 * @return int 
 */
int main()
{
    /* Initialize all components on the lab-kit. */
    BSP_Init();    

    /* Create the queues. */
    btnQueue = xQueueCreate(5, sizeof(btn_evt_t));

    /* Create all mutex. */
    mutex_brightness = xSemaphoreCreateMutex();
    mutex_valIn = xSemaphoreCreateMutex();
    mutex_valOut = xSemaphoreCreateMutex();
    accMutex = xSemaphoreCreateMutex();

    /* Create the tasks. */
    xTaskCreate(led_task, "LED Task", 512, (void*) LED_PERIOD, 3, &ledTsk);
    xTaskCreate(disp_task, "Display Task", 512, (void*) LED_PERIOD, 2, &dispTsk);
    xTaskCreate(input_task, "Input Task", 512, (void*) INPUT_PERIOD, 5, &inputTsk);
    xTaskCreate(blink_task, "Blink Task", 512, (void*) BLINK_PERIOD, 1, &blinkTsk);
    xTaskCreate(uart_task, "UART Task", 512, (void*) UART_PERIOD, 6, &uartTsk);
    xTaskCreate(cn1Rx_task, "CN1 RX Task", 512, (void*) NULL, 1, &cn1RxTsk);
    xTaskCreate(cn1Tx_task, "CN1 TX Task", 512, (void*) CN1_TX_PERIOD, 1, &cn1TxTsk);

    if (BSP_HasPSRAM() > 0) {
        xTaskCreate(psram_task, "PSRAM Task", 512, (void*) PSRAM_PERIOD, 1, &psramTsk);
    }

    /* Start the scheduler. */
    vTaskStartScheduler();
    
    while (true) {
        sleep_ms(1000); /* Should not reach here... */
    }
}
/*-----------------------------------------------------------*/

void led_task(void *args) {
    TickType_t xLastWakeTime = 0;
    const TickType_t xPeriod = (int)args;   /* Get period (in ticks) from argument. */
    uint32_t data = 0x01;
    btn_evt_t msg;

    BSP_ShiftRegisterSetBrightness(5);

    for (;;) {

        if (xQueueReceive(btnQueue, &msg, 0)) {
            /* A new button event is received. */
            if (msg.state == false) {                                    /* Rising edge detected. */
                if (xSemaphoreTake(mutex_brightness, portMAX_DELAY)) {  /* Get the mutex, wait forever. */
                    if (msg.btn == SW_5) {                              /* Brightness up +5. */
                        sr_brightness += 5;
                        if (sr_brightness > 100) sr_brightness = 100;
                    } else if (msg.btn == SW_6) {                       /* Brightness down -5. */
                        sr_brightness -= 5;
                        if (sr_brightness < 0) sr_brightness = 0;
                    } else if (msg.btn == SW_7) {                       /* Brightness up +10. */
                        sr_brightness += 10;
                        if (sr_brightness > 100) sr_brightness = 100;
                    } else if (msg.btn == SW_8) {                       /* Brightness down -10. */
                        sr_brightness -= 10;
                        if (sr_brightness < 0) sr_brightness = 0;
                    }

                    BSP_ShiftRegisterSetBrightness(sr_brightness);
                    xSemaphoreGive(mutex_brightness);
                }
            }
        }
        data = data << 1;
        if (data == (1 << 24)) data = 1;
        BSP_ShiftRegWriteAll((uint8_t*)(&data));

        vTaskDelayUntil(&xLastWakeTime, xPeriod);   /* Wait for the next release. */
    }
}
/*-----------------------------------------------------------*/

void disp_task(void* args) {

    TickType_t xLastWakeTime = 0;
    const TickType_t xPeriod = (int)args;   /* Get period (in ticks) from argument. */

    float xSamples[ACC_SAMPLES];
    float ySamples[ACC_SAMPLES];
    float zSamples[ACC_SAMPLES];
    uint32_t pos = 0;

    float g_x, g_y, g_z;
    int32_t tc;

    char dspStrng[9];   /* Buffer for the display string. */
    uint8_t cnt = 0;

    for (;;) {

        if (xSemaphoreTake(accMutex, portMAX_DELAY)) {
            g_x = g_xVal;
            g_y = g_yVal;
            g_z = g_zVal;
            tc = tapCount;
            xSemaphoreGive(accMutex);
        }

        if (BSP_GetInput(SW_10) == true) {
            if (cnt == 0) {
                sprintf(dspStrng, "   .", sizeof(dspStrng));
            } else if (cnt == 1) {
                sprintf(dspStrng, "  . ", sizeof(dspStrng));
            } else if (cnt == 2) {
                sprintf(dspStrng, " .  ", sizeof(dspStrng));
            } else if (cnt == 3) {
                sprintf(dspStrng, ".   ", sizeof(dspStrng));
            }
            cnt = (cnt + 1) %4;
        } else if (BSP_GetInput(SW_11) == true) {
                sprintf(dspStrng, "% 4.2f", g_x, sizeof(dspStrng));
        }  else if (BSP_GetInput(SW_12) == true) {
                sprintf(dspStrng, "% 4.2f", g_y, sizeof(dspStrng));
        }  else if (BSP_GetInput(SW_13) == true) {
                sprintf(dspStrng, "% 4.2f", g_z, sizeof(dspStrng));
        }  else if (BSP_GetInput(SW_14) == true) {
            if (xSemaphoreTake(mutex_brightness, portMAX_DELAY)) {  /* Get the mutex, wait forever. */
                sprintf(dspStrng, "% 4i", sr_brightness, sizeof(dspStrng));
                xSemaphoreGive(mutex_brightness);
            }
        } else if (BSP_GetInput(SW_15) == true) {
            if (xSemaphoreTake(mutex_valOut, portMAX_DELAY)) {  /* Get the mutex, wait forever. */
                sprintf(dspStrng, "% 4i", valOut, sizeof(dspStrng));
                xSemaphoreGive(mutex_valOut);
            }
        } else if (BSP_GetInput(SW_16) == true) {
            if (xSemaphoreTake(mutex_valIn, portMAX_DELAY)) {  /* Get the mutex, wait forever. */
                sprintf(dspStrng, "% 4i", valIn, sizeof(dspStrng));
                xSemaphoreGive(mutex_valIn);
            }
        } else if (BSP_GetInput(SW_17) == true) {
            sprintf(dspStrng, "%4d", tc, sizeof(dspStrng));
        }

        BSP_7SegDispString(dspStrng);
        
        vTaskDelayUntil(&xLastWakeTime, xPeriod);   /* Wait for the next release. */
    }
}
/*-----------------------------------------------------------*/

void input_task(void* args) {
    TickType_t xLastWakeTime = 0;
    const TickType_t xPeriod = (int)args;   /* Get period (in ticks) from argument. */

    bool btn1 = BSP_GetInput(SW_5);
    bool btn2 = BSP_GetInput(SW_6);
    bool btn3 = BSP_GetInput(SW_7);
    bool btn4 = BSP_GetInput(SW_8);

    btn_evt_t msg;

    for (;;) {

        if (BSP_GetInput(SW_5) != btn1){
            btn1 = !btn1;

            msg.btn = SW_5;
            msg.state = btn1;

            xQueueSend(btnQueue, &msg, 0);
        } 
        if (BSP_GetInput(SW_6) != btn2) {
            btn2 = !btn2;


            msg.btn = SW_6;
            msg.state = btn2;
            
            xQueueSend(btnQueue, &msg, 0);
        } 
        if (BSP_GetInput(SW_7) != btn3) {
            btn3 = !btn3;


            msg.btn = SW_7;
            msg.state = btn3;
            
            xQueueSend(btnQueue, &msg, 0);
        } 
        if (BSP_GetInput(SW_8) != btn4) {
            btn4 = !btn4;


            msg.btn = SW_8;
            msg.state = btn4;
            
            xQueueSend(btnQueue, &msg, 0);
        } 

        int8_t newTapCount = BSP_GetTapCount();

        if (xSemaphoreTake(accMutex, portMAX_DELAY)) {
            lastPos = (lastPos + 1) % ACC_SAMPLES;
            xSamples[lastPos] = BSP_GetAxisAcceleration(X_AXIS);
            ySamples[lastPos] = BSP_GetAxisAcceleration(Y_AXIS);
            zSamples[lastPos] = BSP_GetAxisAcceleration(Z_AXIS);

            g_xVal = 0;
            g_yVal = 0;
            g_zVal = 0;

            for (int i = 0; i < ACC_SAMPLES; i++) {
                g_xVal += xSamples[i];
                g_yVal += ySamples[i];
                g_zVal += zSamples[i];
            }

            g_xVal = g_xVal / (float)ACC_SAMPLES;
            g_yVal = g_yVal / (float)ACC_SAMPLES;
            g_zVal = g_zVal / (float)ACC_SAMPLES;

            tapCount += newTapCount;
            
            xSemaphoreGive(accMutex);
        }

        vTaskDelayUntil(&xLastWakeTime, xPeriod);   /* Wait for the next release. */
    }
}
/*-----------------------------------------------------------*/

void blink_task(void* args) {
    TickType_t xLastWakeTime = 0;
    const TickType_t xPeriod = (int)args;   /* Get period (in ticks) from argument. */

    uint8_t cnt = 0;

    for (;;) {

        if (cnt == 0) {
            BSP_SetLED(LED_RED, true);
        } else if (cnt == 1) {
            BSP_SetLED(LED_YELLOW, true);
        } else if (cnt == 2) {
            BSP_SetLED(LED_GREEN, true);
        } else if (cnt == 3) {
            BSP_SetLED(LED_RED, false);
            BSP_SetLED(LED_YELLOW, false);
            BSP_SetLED(LED_GREEN, false);
        }

        cnt = (cnt + 1) % 4;
        vTaskDelayUntil(&xLastWakeTime, xPeriod);   /* Wait for the next release. */
    }
}
/*-----------------------------------------------------------*/

void uart_task(void* args) {
    TickType_t xLastWakeTime = 0;
    const TickType_t xPeriod = (int)args;   /* Get period (in ticks) from argument. */

    bool button1;   /* SW_5*/
    bool button2;   /* SW_6*/
    bool button3;   /* SW_7*/
    bool button4;   /* SW_8*/

    bool switch1;   /* SW_10*/
    bool switch2;   /* SW_11*/
    bool switch3;   /* SW_12*/
    bool switch4;   /* SW_13*/
    bool switch5;   /* SW_14*/
    bool switch6;   /* SW_15*/
    bool switch7;   /* SW_16*/
    bool switch8;   /* SW_17*/

    float acceleration_X;
    float acceleration_Y;
    float acceleration_Z;

    for (;;) {

        /* Read all inputs. */
        button1 = BSP_GetInput(SW_5);
        button2 = BSP_GetInput(SW_6);
        button3 = BSP_GetInput(SW_7);
        button4 = BSP_GetInput(SW_8);

        switch1 = BSP_GetInput(SW_10);
        switch2 = BSP_GetInput(SW_11);
        switch3 = BSP_GetInput(SW_12);
        switch4 = BSP_GetInput(SW_13);
        switch5 = BSP_GetInput(SW_14);
        switch6 = BSP_GetInput(SW_15);
        switch7 = BSP_GetInput(SW_16);
        switch8 = BSP_GetInput(SW_17);

        if (xSemaphoreTake(accMutex, portMAX_DELAY)) {
            acceleration_X = xSamples[lastPos];
            acceleration_Y = ySamples[lastPos];
            acceleration_Z = zSamples[lastPos];
            xSemaphoreGive(accMutex);
        }

        /* Send the values via UART. */
        printf("%.2f %.2f %.2f %d %d %d %d %d %d %d %d %d %d %d %d\r\n", 
            acceleration_X, 
            acceleration_Y, 
            acceleration_Z,
            button1, 
            button2, 
            button3, 
            button4, 
            switch1, 
            switch2, 
            switch3, 
            switch4, 
            switch5, 
            switch6, 
            switch7, 
            switch8);

        vTaskDelayUntil(&xLastWakeTime, xPeriod);   /* Wait for the next release. */
    }
}
/*-----------------------------------------------------------*/

void cn1Rx_task(void* args) {

    valIn = 0;  /* Initialize the value. */
    uint32_t tmp;

    for (;;) {

        uart_read_blocking (uart1, (uint8_t*)&tmp, sizeof(tmp));    /* Wait for a new value to arrive. */

        if (xSemaphoreTake(mutex_valIn, portMAX_DELAY)) {  /* Get the mutex, wait forever. */
            valIn = tmp;
            xSemaphoreGive(mutex_valIn);
        }
    }
}
/*-----------------------------------------------------------*/

void cn1Tx_task(void* args) {
    TickType_t xLastWakeTime = 0;
    const TickType_t xPeriod = (int)args;   /* Get period (in ticks) from argument. */

    valOut = 0; /* Initialize the value. */
    uint32_t tmp;

    for (;;) {

        if (xSemaphoreTake(mutex_valOut, portMAX_DELAY)) {  /* Get the mutex, wait forever. */
            valOut++;
            tmp = valOut;
            xSemaphoreGive(mutex_valOut);
        }

        uart_write_blocking (uart1, (uint8_t*)&tmp, sizeof(tmp));
        vTaskDelayUntil(&xLastWakeTime, xPeriod);   /* Wait for the next release. */
    }
}
/*-----------------------------------------------------------*/

void psram_task(void* args) {
    TickType_t xLastWakeTime = 0;
    const TickType_t xPeriod = (int)args;   /* Get period (in ticks) from argument. */

    for (;;) {

        vTaskDelayUntil(&xLastWakeTime, xPeriod);   /* Wait for the next release. */
    }
}
/*-----------------------------------------------------------*/
