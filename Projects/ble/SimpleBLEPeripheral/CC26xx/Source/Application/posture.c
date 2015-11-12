/* -----------------------------------------------------------------------------
*                                          Includes
* ------------------------------------------------------------------------------
*/

#include <xdc/std.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/System.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Queue.h>
#include "Board.h"

#include "posture.h"

/* -----------------------------------------------------------------------------
*                                           Constants
* ------------------------------------------------------------------------------
*/
// Task configuration
#define POSTURE_TASK_PRIORITY             1
#ifndef POSTURE_TASK_STACK_SIZE
#define POSTURE_TASK_STACK_SIZE           644
#endif

#define POSTURE_SAMPLE_TIME				  1000
// Task Events
#define POSTURE_KEY_CHANGE_EVT            0x0004


/* -----------------------------------------------------------------------------
*                           Local Variables
* ------------------------------------------------------------------------------
*/
PIN_Handle pinHandle;

// Task configuration
Task_Struct Posture_Task;
Char Posture_TaskStack[POSTURE_TASK_STACK_SIZE];



/* -----------------------------------------------------------------------------
*                           Local Functions
* ------------------------------------------------------------------------------
*/
void Posture_createTask(void);
static void Posture_taskFxn(UArg a0, UArg a1);
static void led_on(int16_t led);
static void led_off(int16_t led);

/* -----------------------------------------------------------------------------
*
* ------------------------------------------------------------------------------
*/
void Posture_createTask(void)
{
    Task_Params taskParams;
    Task_Params_init(&taskParams);
    taskParams.stack = Posture_TaskStack;
    taskParams.stackSize = POSTURE_TASK_STACK_SIZE;
    taskParams.priority = POSTURE_TASK_PRIORITY;
    Task_construct(&Posture_Task, Posture_taskFxn, &taskParams, NULL);
}

void led_on(int16_t led)
{
    PIN_setOutputValue(pinHandle, led, 1);
}

void led_off(int16_t led)
{
    PIN_setOutputValue(pinHandle, led, 0);
}

void Posture_taskFxn(UArg a0, UArg a1)
{
//    uint8_t data[18];
//
//    // Setup I2C for sensors
//    bspI2cInit();
//
//    // uart for debug
//    UART_init_non_default();
//
//    // Initialize the mpu9250
//    sensorMpu9250Init();

    while (1) {
//        Task_sleep(SAMPLE_TIME_MS * (1000 / Clock_tickPeriod));
        led_on(Board_LED1);
        led_off(Board_LED2);
    	Task_sleep(POSTURE_SAMPLE_TIME * (1000 / Clock_tickPeriod));
        led_off(Board_LED1);
        led_on(Board_LED2);
    	Task_sleep(POSTURE_SAMPLE_TIME * (1000 / Clock_tickPeriod));

    }
}
