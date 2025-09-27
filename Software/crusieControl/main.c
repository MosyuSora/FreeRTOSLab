/**
 * @file main.c
 * @author Ingo Sander (ingo@kth.se)
 * @brief Skeleton for cruise control application
 *        The skeleton code runs on the ES-Lab-Kit, 
 *        has very limited functionality and needs to be
 *        modified.
 *          
 * @version 0.1
 * @date 2025-09-12
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

#define GAS_STEP 2  /* Defines how much the throttle is increased if GAS_STEP is asserted */

/* Definition of handles for tasks */
TaskHandle_t    xButton_handle; /* Handle for the Button task */
TaskHandle_t    xControl_handle; /* Handle for the Control task */
TaskHandle_t    xVehicle_handle; /* Handle for the Vehicle task */
TaskHandle_t    xDisplay_handle; /* Handle for the Display task */

/* Definition of handles for queues */
QueueHandle_t xQueueVelocity;
QueueHandle_t xQueuePosition;
QueueHandle_t xQueueThrottle;
QueueHandle_t xQueueCruiseControl;
QueueHandle_t xQueueGasPedal;
QueueHandle_t xQueueBrakePedal;

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
    bool value_gas_pedal = true;
    bool value_brake_pedal = false;
    bool value_cruise_control = false;

    for (;;) {
        xQueueOverwrite(xQueueGasPedal,&value_gas_pedal);
        xQueueOverwrite(xQueueBrakePedal,&value_brake_pedal);
        xQueueOverwrite(xQueueCruiseControl,&value_cruise_control);
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
    uint16_t throttle = 80;
    uint16_t velocity;
    bool cruise_control_button;
    bool cruise_control;
    bool gas_pedal;
    bool brake_pedal;

    for (;;) {
        if (gas_pedal) {
            throttle += GAS_STEP; 
            if (throttle > 80) {
                throttle = 80;
            }
        }
        else {
            if (cruise_control) { 
                ;
            }
            else throttle = 0;
        }
        xQueuePeek(xQueueCruiseControl, &cruise_control_button, ( TickType_t ) 0);
        xQueuePeek(xQueueGasPedal, &gas_pedal, ( TickType_t ) 0);   
        xQueuePeek(xQueueVelocity, &velocity, ( TickType_t ) 0);     
        xQueuePeek(xQueueBrakePedal, &brake_pedal, ( TickType_t ) 0);
        
        xQueueOverwrite(xQueueThrottle, &throttle);
    }
}

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
    + acceleration / 2  * (time_interval / 1000) * (time_interval / 1000);

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
    new_velocity = velocity  + (float) ((acceleration * time_interval) / 1000);
    if (new_velocity <= 0) {
        new_velocity = 0;
    }
  } 
  else { 
    if ((float) (brake_retardation * time_interval) / 1000 > velocity) {
       new_velocity = 0;
    }
    else {
      new_velocity = velocity - (float) brake_retardation * time_interval / 1000;
    }
  } 

  return new_velocity;
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
        xQueuePeek(xQueueThrottle, &throttle, ( TickType_t ) 0);
        xQueuePeek(xQueueBrakePedal, &brake_pedal, ( TickType_t ) 0);

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
    uint16_t throttle;  
    uint16_t position;

    for (;;) {
        xQueuePeek(xQueueVelocity, &velocity, ( TickType_t ) 0);
        xQueuePeek(xQueuePosition, &position, ( TickType_t ) 0);
        xQueuePeek(xQueueThrottle, &throttle, ( TickType_t ) 0);

        printf("Throttle: %d\n", throttle);
        printf("Velocity: %d\n", velocity);
        printf("Position: %d\n", position);
         
        vTaskDelayUntil(&xLastWakeTime, xPeriod);   /* Wait for the next release. */
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
    xTaskCreate(vButtonTask, "Button Task", 512, NULL, 4, &xButton_handle);
    xTaskCreate(vVehicleTask, "Vehicle Task", 512, (void*) 100, 7, &xVehicle_handle); 
    xTaskCreate(vControlTask, "Control Task", 512, (void*) NULL, 4, &xControl_handle);
    xTaskCreate(vDisplayTask, "Display Task", 512, (void*) 500, 5, &xDisplay_handle); 

    /* Create the message queues */
    xQueueCruiseControl = xQueueCreate( 1, sizeof(bool));
    xQueueGasPedal = xQueueCreate( 1, sizeof(bool));
    xQueueBrakePedal = xQueueCreate( 1, sizeof(bool));
    xQueueVelocity = xQueueCreate( 1, sizeof(uint16_t));
    xQueuePosition = xQueueCreate( 1, sizeof(uint16_t));
    xQueueThrottle = xQueueCreate( 1, sizeof(uint16_t));

    vTaskStartScheduler();  /* Start the scheduler. */
    
    return 0;
}
/*-----------------------------------------------------------*/
