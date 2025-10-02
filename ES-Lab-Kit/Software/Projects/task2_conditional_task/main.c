/**
 * @file main.c
 * @author LuyuYang ZiheCao
 * @brief 
 * @version 0.1
 * @date 2025-09-30
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "bsp.h"
#include "hardware/clocks.h"
#include "math.h"

#define GAS_STEP 2  /* Defines how much the throttle is increased if GAS_STEP is asserted */

#define P 100
#define I 1
#define D 5

/* Definition of handles for tasks */
TaskHandle_t xWatchDog_handle;       /* Handle for watch dog task            */ 
TaskHandle_t xButton_handle;         /* Handle for the Button task           */ 
TaskHandle_t xControl_handle;        /* Handle for the Control task          */
TaskHandle_t xVehicle_handle;        /* Handle for the Vehicle task          */
TaskHandle_t xSlope_handle;  /* Handle for the Accelerometer         */
TaskHandle_t xDisplay_handle;        /* Handle for the Display task          */
TaskHandle_t xOverload_handle;       /* Handle for overload detecting task   */

/* Task Function Prototype */
void vWatchDogTask(void *arg);
void vButtonTask(void *args);
void vControlTask(void *args);
void vSlopeTask(void *args);
void vVehicleTask(void *args);
void vDisplayTask(void *args);
void vOverloadDetectionTask(void *arg);

/* Definition of handles for semaphore/mutex */
SemaphoreHandle_t xSemaphoreWatchDogFood;

/* Definition of handles for queues */
QueueHandle_t xQueueVelocity;
QueueHandle_t xQueueTargetVelocity;
QueueHandle_t xQueuePosition;
QueueHandle_t xQueueThrottle;
QueueHandle_t xQueueCruiseControl;
QueueHandle_t xQueueGasPedal;
QueueHandle_t xQueueBrakePedal;
QueueHandle_t xQueueSlope;


/**
 * @brief The function returns the new position depending on the input parameters.
 * 
 * ==> DO NOT CHANGE THIS FUNCTION !!!
 * 
 * @param position 
 * @param velocity 
 * @param acceleration 
 * @param time_interval 
 * @return 
 */
uint16_t adjust_position(uint16_t position, int16_t velocity,
                         int8_t acceleration, uint16_t time_interval)
{
    int16_t new_position = position + velocity * time_interval / 1000
                         + acceleration / 2 * (time_interval / 1000) * (time_interval / 1000);

    if (new_position > 24000) {
        new_position -= 24000;
    } else if (new_position < 0){
        new_position += 24000;
    }

    return new_position;
}


/**
 * @brief The function returns the new velocity depending on the input parameters.
 * 
 * ==> DO NOT CHANGE THIS FUNCTION !!! 
 *
 * @param velocity 
 * @param acceleration 
 * @param brake_pedal 
 * @param time_interval 
 * @return 
 */
int16_t adjust_velocity(int16_t velocity, int8_t acceleration,  
                        bool brake_pedal, uint16_t time_interval)
{
    int16_t new_velocity;
    uint8_t brake_retardation = 50;

    if (brake_pedal == false) {
        new_velocity = velocity + (float)((acceleration * time_interval) / 1000);
        if (new_velocity <= 0) {
            new_velocity = 0;
        }
    } else { 
        if ((float)(brake_retardation * time_interval) / 1000 > velocity) {
            new_velocity = 0;
        } else {
            new_velocity = velocity - (float)brake_retardation * time_interval / 1000;
        }
    }

    return new_velocity;
}


/**
 * Find throttle using PID controller
 * @param target_velocity
 * @param velocity
 * @return <uint16_t> throttle
 */
uint16_t calc_throttle_with_PID(uint16_t target_velocity, uint16_t velocity) {
    //I know it's not safe for thread... But there's only one task using this...
    static int32_t integral = 0;        
    static int16_t prev_error = 0;      

    int16_t error = (int16_t)target_velocity - (int16_t)velocity;
    integral += error;

    int16_t derivative = error - prev_error;
    prev_error = error;

    // PID output
    int32_t output = P * error + I * integral + D * derivative;

    // Limit with range 0-80
    if (output < 0) output = 0;
    if (output > 80) output = 80;

    return (uint16_t)output;
}


/**
 * Given position 0~24000, image from 1st led to 24th led as a uint_32
 *  1000 -> 0000...001(24 in total)
 * 24000 -> 1000...000(24 in total)
 * Then use BSP_ShiftRegWriteAll(uint8_t*) to write led
 * @param position
 * @return None
 */
void write_position(uint16_t position){
    uint32_t led_reg;
    int led_index = position / 1000; // 0~23
    if (led_index > 23) led_index = 23;  
    led_reg = (1u << led_index);//left shift by index
    BSP_ShiftRegWriteAll((uint8_t*)&led_reg);
}


//task functions


/**
 * Watch dog task which is feed by overload-detection task
 * 
 * Examine overload with period of 1000ms
 * 
 * @param args -> delay
 */
void vWatchDogTask(void *args){
    TickType_t xLastWakeTime = 0;
    const TickType_t xPeriod = (int)args;//1000ms
    bool isFeed;
    for(;;){
        isFeed = (xSemaphoreTake(xSemaphoreWatchDogFood, xPeriod) == pdTRUE);
        if(!isFeed){
            printf("System Overload!/n");
        }
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }
}


/**
 * @brief The button task shall monitor the input buttons and send the values to the
 *        other tasks 
 * 
 * ==> MODIFY THIS TASK! 
 *     Currently the buttons are ignored. Use busy wait I/O to monitor the buttons
 *     
 * @param args 
 */
void vButtonTask(void *args) {
    TickType_t xLastWakeTime = 0;
    const TickType_t xPeriod = (int)args;   /* Get period (in ticks) from argument. */
    bool btnGas ;
    bool btnBrake; 
    bool btnCruise = BSP_GetInput(SW_6);
    bool value_cruise_control= false;

    uint16_t now_velocity;
    for (;;) {
        btnGas = !BSP_GetInput(SW_7);//According to schematic plot, press btn->low
        btnBrake = !BSP_GetInput(SW_5);
        if ((BSP_GetInput(SW_6) != btnCruise) && (BSP_GetInput(SW_6) == false)){
            //at negative edge of SW_6, process cruise logic
            value_cruise_control ^= 1;//toggle crusie control
            //overwrite target velocity
            xQueuePeek(xQueueVelocity, &now_velocity, (TickType_t)0);
            xQueueOverwrite(xQueueTargetVelocity, &now_velocity);
        }
        btnCruise = BSP_GetInput(SW_6);
        if (btnBrake){
            value_cruise_control = false;//use brake to end crusie control
        }
        xQueueOverwrite(xQueueGasPedal, &btnGas);
        xQueueOverwrite(xQueueBrakePedal, &btnBrake);
        xQueueOverwrite(xQueueCruiseControl, &value_cruise_control);   
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }
}


/**
 * @brief The control tasks calculates the new throttle using your 
 *        control algorithm and the current values.
 * 
 * ==> MODIFY THIS TASK!
 *     Currently the throttle has a fixed value of 80
 *
 * @param args 
 */
void vControlTask(void *args) {
    TickType_t xLastWakeTime = 0;
    const TickType_t xPeriod = (int)args;   /* Get period (in ticks) from argument. */
    uint16_t throttle = 0;
    uint16_t velocity;
    uint16_t target_velocity;

    bool cruise_control;
    bool gas_pedal;
    bool brake_pedal;

    for (;;) {
        if (brake_pedal) {
            throttle = 0;     // Prevent some stupid-ass press brake and gas at the same time
        } else if (gas_pedal) {
            uint16_t t = throttle + GAS_STEP;
            throttle = (t > 80) ? 80 : t;
        } else if (cruise_control) {
            throttle = calc_throttle_with_PID(target_velocity, velocity);
        } else {
            throttle = 0;               
        }
        xQueuePeek(xQueueCruiseControl, &cruise_control, (TickType_t)0);
        xQueuePeek(xQueueGasPedal, &gas_pedal, (TickType_t)0);   
        xQueuePeek(xQueueVelocity, &velocity, (TickType_t)0);     
        xQueuePeek(xQueueBrakePedal, &brake_pedal, (TickType_t)0);
        xQueuePeek(xQueueTargetVelocity, &target_velocity, (TickType_t)0);
        
        xQueueOverwrite(xQueueThrottle, &throttle);

        vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }
}


/**
 * @brief The vehicle task continuously calculates the velocity of the vehicle 
 *
 * ==> DO NOT CHANGE THIS TASK !!!  
 *
 * @param args 
 */
void vVehicleTask(void *args) {
    TickType_t xLastWakeTime = 0;
    const TickType_t xPeriod = (int)args;   /* Get period (in ticks) from argument. */
    uint16_t throttle;
    bool brake_pedal;
                           /* Approximate values*/
    uint8_t acceleration;  /* Value between 40 and -20 (4.0 m/s^2 and -2.0 m/s^2) */
    uint8_t retardation;   /* Value between 20 and -10 (2.0 m/s^2 and -1.0 m/s^2) */
    uint16_t position = 0; /* Value between 0 and 24000 (0.0 m and 2400.0 m)  */
    uint16_t velocity = 0; /* Value between -200 and 700 (-20.0 m/s amd 70.0 m/s) */
    uint16_t wind_factor;   /* Value between -10 and 20 (2.0 m/s^2 and -1.0 m/s^2) */

    for (;;) {
        xQueuePeek(xQueueThrottle, &throttle, (TickType_t)0);
        xQueuePeek(xQueueBrakePedal, &brake_pedal, (TickType_t)0);

        /* Retardation : Factor of Terrain and Wind Resistance */
        if (velocity > 0)
            wind_factor = velocity * velocity / 10000 + 1;
        else 
            wind_factor = (-1) * velocity * velocity / 10000 + 1;

        if (position < 4000) 
            retardation = wind_factor; // even ground
        else if (position < 8000)
            retardation = wind_factor + 8; // traveling uphill
        else if (position < 12000)
            retardation = wind_factor + 16; // traveling steep uphill
        else if (position < 16000)
            retardation = wind_factor; // even ground
        else if (position < 20000)
            retardation = wind_factor - 8; //traveling downhill
        else
            retardation = wind_factor - 16 ; // traveling steep downhill

        acceleration = throttle / 2 - retardation;	  
        position = adjust_position(position, velocity, acceleration, xPeriod); 
        velocity = adjust_velocity(velocity, acceleration, brake_pedal, xPeriod);         

        xQueueOverwrite(xQueueVelocity, &velocity);
        xQueueOverwrite(xQueuePosition, &position); 
        vTaskDelayUntil(&xLastWakeTime, xPeriod);   /* Wait for the next release. */
    }
}


//Get x axis slope
void vSlopeTask(void *args){
    TickType_t xLastWakeTime = 0;
    const TickType_t xPeriod = (int)args;   /* Get period (in ticks) from argument. */

    const int AVG_SAMPLES = 10;   // 平滑窗口大小
    float sum = 0;
    int count = 0;
    float acc_in_g;
    uint16_t slope;

    for(;;){
        // 1. 读取X轴加速度 (单位 g)
        acc_in_g = BSP_GetAxisAcceleration(X_AXIS);

        // 2. 采样求和
        sum += acc_in_g;
        count++;
        // 3. 每 AVG_SAMPLES 次取平均，映射到 0~90 度
        if (count >= AVG_SAMPLES) {
            float avg_acc = (sum / count);     // 平滑加速度

            if(avg_acc <0) avg_acc = - avg_acc ;

            slope = (uint16_t)(avg_acc * 90); // 简单映射到 0~90°

            if (slope > 90)             slope = 90;
            if (slope < 0 )             slope = 0 ;
            

            xQueueOverwrite(xQueueSlope, &slope);

            // 4. 重新开始累计
            sum = 0;
            count = 0;
        }

        vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }
}

/**
 * @brief The display task shall show the information on 
 *          - the throttle and velocity on the seven segment display
 *          - the position on the 24 LEDs
 * 
 * ==> MODIFY THIS TASK!
 *     Currently the information is shown on the standard output (serial monitor in VSCode)
 * 
 * @param args 
 */
void vDisplayTask(void *args) {
    TickType_t xLastWakeTime = 0;
    const TickType_t xPeriod = (int)args;   /* Get period (in ticks) from argument. */

    uint16_t velocity;  
    uint16_t position;
    uint16_t slope;

    bool gas_pedal;
    bool brake_pedal;
    bool cruise_control;
    BSP_7SegClear();
    char dspStrng[9];   // buffer for display

    for (;;) {
        
        xQueuePeek(xQueueVelocity     ,&velocity      , (TickType_t)0);
        xQueuePeek(xQueuePosition     ,&position      , (TickType_t)0);
        xQueuePeek(xQueueSlope        ,&slope         , (TickType_t)0);
        xQueuePeek(xQueueGasPedal     ,&gas_pedal     , (TickType_t)0);
        xQueuePeek(xQueueBrakePedal   ,&brake_pedal   , (TickType_t)0);
        xQueuePeek(xQueueCruiseControl,&cruise_control, (TickType_t)0);
        
        sprintf(dspStrng, "%2d%2d", slope, velocity/10);
        printf("%s\n",dspStrng);

        write_position(position);
        BSP_SetLED(LED_GREEN, gas_pedal);
        BSP_SetLED(LED_RED, brake_pedal);
        BSP_SetLED(LED_YELLOW, cruise_control);
        BSP_7SegDispString(dspStrng);

        //serial debug
        printf("Slope: %2d\n", slope);
        printf("Velocity: %d\n", velocity);
        printf("Position: %d\n", position);
         
        vTaskDelayUntil(&xLastWakeTime, xPeriod); // 每500ms运行一次
    }
}


void vOverloadDetectionTask(void *arg){
    bool food = true;
    for(;;){
        xSemaphoreGive(xSemaphoreWatchDogFood);
    }
}


/**
 * @brief Main program that starts all the tasks and the scheduler
 * 
 * ==> MODIFY THE MAIN PROGRAM!
 *        - Convert the button and control tasks to periodic tasks
 *        - Adjust the priorities of the task so that they correspond
 *          to the rate-monotonic algorithm.
 * @return 
 */
int main()
{
    BSP_Init();  /* Initialize all components on the ES Lab-Kit. */

    /* Create the tasks. */
    xTaskCreate(vWatchDogTask, "Watch Dog Task", 512, (void*)1000, 10, &xWatchDog_handle);
    xTaskCreate(vButtonTask, "Button Task", 512, (void*)50, 9, &xButton_handle);
    xTaskCreate(vSlopeTask, "Slope Task", 512, (void*)100, 8, &xSlope_handle);
    xTaskCreate(vVehicleTask, "Vehicle Task", 512, (void*)100, 7, &xVehicle_handle); 
    xTaskCreate(vControlTask, "Control Task", 512, (void*)200, 6, &xControl_handle);
    xTaskCreate(vDisplayTask, "Display Task", 512, (void*)500, 5, &xDisplay_handle); 
    xTaskCreate(vOverloadDetectionTask, "Overload Detection Task", 512, NULL, 1, &xOverload_handle);

    /* Create the semaphore */
    xSemaphoreWatchDogFood = xSemaphoreCreateBinary();

    /* Create the message queues */
    xQueueCruiseControl   = xQueueCreate(1, sizeof(bool));
    xQueueGasPedal        = xQueueCreate(1, sizeof(bool));
    xQueueBrakePedal      = xQueueCreate(1, sizeof(bool));
    xQueueVelocity        = xQueueCreate(1, sizeof(uint16_t));
    xQueueTargetVelocity  = xQueueCreate(1, sizeof(uint16_t));
    xQueuePosition        = xQueueCreate(1, sizeof(uint16_t));
    xQueueThrottle        = xQueueCreate(1, sizeof(uint16_t));
    xQueueSlope           = xQueueCreate(1, sizeof(uint16_t));

    vTaskStartScheduler();  /* Start the scheduler. */
    
    return 0;
}
/*-----------------------------------------------------------*/