#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "bsp.h"


// shared memory
int sharedAddress = 0;


SemaphoreHandle_t xMutex;
SemaphoreHandle_t xSemaphoreA;//by which B notifies A data has been processed
SemaphoreHandle_t xSemaphoreB;//by which A notifies B data need to be processed

void vTaskA(void *pvParameters);
void vTaskB(void *pvParameters);

int main()
{
    BSP_Init();
   
    xMutex = xSemaphoreCreateMutex();
    xSemaphoreA = xSemaphoreCreateBinary();
    xSemaphoreB = xSemaphoreCreateBinary();
    

    xTaskCreate(vTaskA, "TaskA", 256, NULL, 1, NULL);
    xTaskCreate(vTaskB, "TaskB", 256, NULL, 1, NULL);
    
    vTaskStartScheduler();
    
    while(1) {}
}

void vTaskA(void *pvParameters)
{
    int number = 1;
    
    while(1)
    {
         // unpack mutex
        xSemaphoreTake(xMutex, portMAX_DELAY);
        
        // fill the data
        sharedAddress = number;
        printf("Sending   : %d\n", sharedAddress);
        
        //pack mutex
        xSemaphoreGive(xMutex);
        
        // pass to TaskB
        xSemaphoreGive(xSemaphoreB);
        

        //Blocking to wait semA ....


        xSemaphoreTake(xSemaphoreA, portMAX_DELAY);
        
        // unpack mutex
        xSemaphoreTake(xMutex, portMAX_DELAY);
        
        // peek share data
        printf("Receiving : %d\n", sharedAddress);


        //pack mutex
        xSemaphoreGive(xMutex);
        

        number++;
        vTaskDelay(500);
        

    }
}

void vTaskB(void *pvParameters)
{
    while(1)
    {
        // Wait for message from A
        xSemaphoreTake(xSemaphoreB, portMAX_DELAY);
 
        
        //process data
        xSemaphoreTake(xMutex, portMAX_DELAY);
        
        sharedAddress *= -1;
        
        xSemaphoreGive(xMutex);
        
        // Notify A
        xSemaphoreGive(xSemaphoreA);
        

    }
}