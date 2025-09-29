#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "bsp.h"

#define STEP_TIME_MS 2000

// 使用两个独立的信号量
SemaphoreHandle_t semRedDone;
SemaphoreHandle_t semGreenDone;

void vTaskGreen(void *pvParameters);
void vTaskRed(void *pvParameters);

/*************************************************************/

int main()
{
    BSP_Init();             /* Initialize all components on the lab-kit. */
    
    semRedDone = xSemaphoreCreateBinary();
    semGreenDone = xSemaphoreCreateBinary();

    /* 初始状态：两个LED都开启 */
    BSP_SetLED(LED_RED, true);
    BSP_SetLED(LED_GREEN, true);

    xTaskCreate(vTaskRed, "RedTask", 256, NULL, 1, NULL);
    xTaskCreate(vTaskGreen, "GreenTask", 256, NULL, 1, NULL);

    vTaskStartScheduler();  /* Start the scheduler. */
    
    while (true) { 
        /* Should not reach here... */
    }
}
/*-----------------------------------------------------------*/

void vTaskRed(void *pvParameters) {
    while (1) {
        // 周期1: 红亮 (2秒)
        BSP_SetLED(LED_RED, true);
        vTaskDelay(pdMS_TO_TICKS(STEP_TIME_MS));
        xSemaphoreGive(semRedDone); // 通知绿灯任务红灯完成
        xSemaphoreTake(semGreenDone, portMAX_DELAY); // 等待绿灯任务完成
        
        // 周期2: 红亮 (2秒)
        BSP_SetLED(LED_RED, true);
        vTaskDelay(pdMS_TO_TICKS(STEP_TIME_MS));
        xSemaphoreGive(semRedDone);
        xSemaphoreTake(semGreenDone, portMAX_DELAY);
        
        // 周期3: 红灭 (2秒)
        BSP_SetLED(LED_RED, false);
        vTaskDelay(pdMS_TO_TICKS(STEP_TIME_MS));
        xSemaphoreGive(semRedDone);
        xSemaphoreTake(semGreenDone, portMAX_DELAY);
        
        // 周期4: 红灭 (2秒)
        BSP_SetLED(LED_RED, false);
        vTaskDelay(pdMS_TO_TICKS(STEP_TIME_MS));
        xSemaphoreGive(semRedDone);
        xSemaphoreTake(semGreenDone, portMAX_DELAY);
    }
}

void vTaskGreen(void *pvParameters) {
    while (1) {
        // 周期1: 绿亮 (2秒)
        BSP_SetLED(LED_GREEN, true);
        vTaskDelay(pdMS_TO_TICKS(STEP_TIME_MS));
        xSemaphoreGive(semGreenDone); // 通知红灯任务绿灯完成
        xSemaphoreTake(semRedDone, portMAX_DELAY); // 等待红灯任务完成
        
        // 周期2: 绿灭 (2秒)
        BSP_SetLED(LED_GREEN, false);
        vTaskDelay(pdMS_TO_TICKS(STEP_TIME_MS));
        xSemaphoreGive(semGreenDone);
        xSemaphoreTake(semRedDone, portMAX_DELAY);
        
        // 周期3: 绿灭 (2秒)
        BSP_SetLED(LED_GREEN, false);
        vTaskDelay(pdMS_TO_TICKS(STEP_TIME_MS));
        xSemaphoreGive(semGreenDone);
        xSemaphoreTake(semRedDone, portMAX_DELAY);
        
        // 周期4: 绿亮 (2秒)
        BSP_SetLED(LED_GREEN, true);
        vTaskDelay(pdMS_TO_TICKS(STEP_TIME_MS));
        xSemaphoreGive(semGreenDone);
        xSemaphoreTake(semRedDone, portMAX_DELAY);
    }
}