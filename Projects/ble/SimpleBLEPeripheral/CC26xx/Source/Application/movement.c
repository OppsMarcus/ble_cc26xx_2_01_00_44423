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
#include <ICall.h>
#include "gatt.h"
#include "hci.h"
#include "gapgattserver.h"
#include "gattservapp.h"
#include "gatt_profile_uuid.h"
#include "devinfoservice.h"
//#include "battservice.h"
//#include "hidkbdservice.h"
//#include "hiddev.h"
#include "peripheral.h"
#include "gapbondmgr.h"
#include "osal_snv.h"
#include "ICallBleAPIMSG.h"
#include "util.h"
#include "board_key.h"
#include "Board.h"
//#include "Button.h"
//#include "Movement.h"
#include "bsp_i2c.h"
#include "sensor_mpu9250.h"
#include "movement.h"
//#include <ti/drivers/UART.h>
//#include <ti/drivers/uart/UARTCC26XX.h>
#include <math.h>


/* -----------------------------------------------------------------------------
*                                           Constants
* ------------------------------------------------------------------------------
*/
//#define DEBUG_UART
// Battery level is critical when it is less than this %
#define DEFAULT_BATT_CRITICAL_LEVEL     6
// Task configuration
#define BLINK_TASK_PRIORITY             1
#ifndef BLINK_TASK_STACK_SIZE
#define BLINK_TASK_STACK_SIZE           644
#endif
// Task Events
#define BLINK_KEY_CHANGE_EVT            0x0001
// Length of the data for this sensor
#define SENSOR_DATA_LEN                 MOVEMENT_DATA_LEN
//#define SIMPLEPROFILE_CHAR4                   3  // RW uint8 - Profile Characteristic 4 value
#define SENSOR_DATA                     0

// Accelerometer specifics
#define ACC_ANGLE                       20
#define ACC_X_OFFSET                    0
#define ACC_Y_OFFSET                    0

#define HIDEMUKBD_IMU_CHANGE_EVT        0x0002
#define SAMPLE_TIME_MS                  10
#define M_PI                            3.14159265358979323846  /* pi */

/* -----------------------------------------------------------------------------
*                           Local Variables
* ------------------------------------------------------------------------------
*/
static uint8_t sensorData[SENSOR_DATA_LEN];

typedef struct
{
    float x;
    float y;
    float z;
} Movement_ImuData_t;

typedef enum {LEFT, RIGHT, FLAT, NONE} direction_t;
direction_t dirX = NONE;
direction_t dirY = NONE;

typedef enum {ZERO, ONE, TWO, THREE} shakes_t;
shakes_t axisY = ZERO;

typedef int skipLoops_t;
skipLoops_t scaleDownAngleTime = 0;
skipLoops_t scaleDownShakeTime = 0;

static PIN_Handle pinHandleA;
// UART
//UART_Handle handle;
//UART_Params params;
// Task configuration
Task_Struct Movement_Task;
Char Movement_TaskStack[BLINK_TASK_STACK_SIZE];
// MPU9250
Movement_ImuData_t gyr;
Movement_ImuData_t acc;

/* -----------------------------------------------------------------------------
*                           Local Functions
* ------------------------------------------------------------------------------
*/
void Movement_createTask(void);
static void Mpu9250_taskFxn(UArg a0, UArg a1);
static bool Movement_AxisFlat(float axis);
static void Movement_ProcessVolumeEvent(float angX, float angY);
static void Movement_ComplementaryFilter(Movement_ImuData_t gyr, Movement_ImuData_t acc, float *angX, float *angY);
static bool Movement_ScaleDownTime(skipLoops_t time, float sec);
static void Movement_ProcessVolumeEvent(float angX, float angY);
static void Movement_ConvertSensorData();
static float Movement_LowPassFilter(float input);
static void Movement_ProcessShake(float ang);
static void led_on(int16_t led);
static void led_off(int16_t led);
static void FloatToStringNew(char *str, float f);
//static void UART_init_non_default();
char* itoa(int16_t num, char* str, int base);
int getLen(char *buffer);

/* -----------------------------------------------------------------------------
*
* ------------------------------------------------------------------------------
*/
void Movement_createTask(void)
{
    Task_Params taskParams;
    Task_Params_init(&taskParams);
    taskParams.stack = Movement_TaskStack;
    taskParams.stackSize = BLINK_TASK_STACK_SIZE;
    taskParams.priority = BLINK_TASK_PRIORITY;
    Task_construct(&Movement_Task, Mpu9250_taskFxn, &taskParams, NULL);
}

void led_on(int16_t led)
{
    PIN_setOutputValue(pinHandleA, led, 1);
}

void led_off(int16_t led)
{
    PIN_setOutputValue(pinHandleA, led, 0);
}

//void UART_init_non_default()
//{
//    // Init UART and specify non-default parameters
//    UART_Params_init(&params);
//    params.baudRate      = 115200;
////    params.writeDataMode = UART_DATA_BINARY;
//    // Open the UART and do the write
//    handle = UART_open(Board_UART, &params);
//}

void Movement_ComplementaryFilter(Movement_ImuData_t gyr, Movement_ImuData_t acc, float *angX, float *angY)
{
    float accRoll  = atan(acc.y / sqrt(acc.x * acc.x + acc.z * acc.z)) * 180 / M_PI;
    float accPitch = atan2(- acc.x, acc.z) * 180 / M_PI;
    *angX = (0.91 * (*angX + gyr.x * SAMPLE_TIME_MS /1000) + 0.09 * accRoll); // Calculate the angle using a Complimentary filter
    *angY = (0.91 * (*angY + gyr.y * SAMPLE_TIME_MS /1000) + 0.09 * accPitch);
}

bool Movement_AxisFlat(float axis)
{
    if((abs((int)axis) < (12))){
        return true;
    }
    else{
        return false;
    }
}

bool Movement_ScaleDownTime(skipLoops_t time, float sec)
{
    if(time > sec * (1000 / SAMPLE_TIME_MS)){
        return true;
    }
    else{
        return false;
    }
}

//void get_movementData(float *mangX, float *mangY)
//{
//	*mangX = sensorMpu9250AccConvert((int16_t)sensorData[6] << 8 | sensorData[7]);
//	*mangY = sensorMpu9250AccConvert((int16_t)sensorData[8] << 8 | sensorData[9]);
//}



void Movement_ProcessVolumeEvent(float angX, float angY)
{
    if((Movement_AxisFlat(angX) == true) && (Movement_AxisFlat(angY) == true)){
        dirX = FLAT;
        dirY = FLAT;
    }
    if(dirX == FLAT && dirY == FLAT)
    {
        if(((int)angX < -40) && (abs((int)angY) < 20)) {
            led_on(Board_LED1);
            if(Movement_ScaleDownTime(scaleDownAngleTime, 0.3)){
//                HidEmuKbd_enqueueMsg(HIDEMUKBD_IMU_CHANGE_EVT, HID_SUBSET_CONSUMER_VOLUME_DOWN, NULL);
                scaleDownAngleTime = 0;
            }
            scaleDownAngleTime++;
        }
        else if(((int)angX > 40) && (abs((int)angY) < 20)){
            led_on(Board_LED1);
            if(Movement_ScaleDownTime(scaleDownAngleTime, 0.3)){
//                HidEmuKbd_enqueueMsg(HIDEMUKBD_IMU_CHANGE_EVT, HID_SUBSET_CONSUMER_VOLUME_UP, NULL);
                scaleDownAngleTime = 0;
            }
            scaleDownAngleTime++;
        }
        else if((abs((int)angY) > 20)){
            dirX = NONE;
            dirY = NONE;
            led_off(Board_LED2);
            led_off(Board_LED1);
        }
        else{
            led_off(Board_LED1);
        }
    }
}

void Movement_ConvertSensorData()
{
//    gyr.x = sensorMpu9250GyroConvert((int16_t)sensorData[0] << 8 | sensorData[1]);
//    gyr.y = sensorMpu9250GyroConvert((int16_t)sensorData[2] << 8 | sensorData[3]);
//    gyr.z = sensorMpu9250GyroConvert((int16_t)sensorData[4] << 8 | sensorData[5]);
//
//    acc.x = sensorMpu9250AccConvert((int16_t)sensorData[6] << 8 | sensorData[7]);
//    acc.y = sensorMpu9250AccConvert((int16_t)sensorData[8] << 8 | sensorData[9]);
//    acc.z = sensorMpu9250AccConvert((int16_t)sensorData[10] << 8 | sensorData[11]);

    acc.x = sensorMpu9250AccConvert((int16_t)sensorData[0] << 8 | sensorData[1]);
    acc.y = sensorMpu9250AccConvert((int16_t)sensorData[2] << 8 | sensorData[3]);
    acc.z = sensorMpu9250AccConvert((int16_t)sensorData[4] << 8 | sensorData[5]);

}

float lastOutput = 0;
float Movement_LowPassFilter(float input)
{
    float alpha = 0.5;
    float output = ((1 - alpha) * input + (alpha * lastOutput));
    lastOutput = output;
    return output;
}

void Movement_ProcessShake(float ang)
{
    if(((ang) > 80) && (axisY == ZERO)){
        axisY = ONE;
        scaleDownShakeTime = 0;
        led_on(Board_LED2);
    }
    if(((ang) < -80) && (axisY == ONE) && (!Movement_ScaleDownTime(scaleDownShakeTime, 1.5))){
        axisY = TWO;
    }
    if(((ang) > 80) && (axisY == TWO) && (!Movement_ScaleDownTime(scaleDownShakeTime, 1.5))){
        axisY = THREE;
    }
    if(((ang) < -80) && (axisY == THREE) && (!Movement_ScaleDownTime(scaleDownShakeTime, 1.5))){
        axisY = ZERO;
        led_off(Board_LED2);
//        HidEmuKbd_enqueueMsg(HIDEMUKBD_IMU_CHANGE_EVT, HID_SUBSET_CONSUMER_PLAY_PAUSE, NULL);
    }
    if (Movement_ScaleDownTime(scaleDownShakeTime, 1)){
        axisY = ZERO;
        led_off(Board_LED2);
    }
    scaleDownShakeTime++;
}

void Mpu9250_taskFxn(UArg a0, UArg a1)
{
    uint8_t data[18];

    // Setup I2C for sensors
    bspI2cInit();

    // uart for debug
//    UART_init_non_default();

    // Initialize the mpu9250
    sensorMpu9250Init();

    while (1) {
//        Task_sleep(SAMPLE_TIME_MS * (1000 / Clock_tickPeriod));
        Task_sleep(1000 * (1000 / Clock_tickPeriod));

        get_movementData(data);
    }
}

int32_t lastz = 0;
void get_movementData(uint8_t *data)
{
	int16_t xx,yy,zz,rawzz,intz;
	int16_t beta = 4;
	char accstr[18];

//    sensorMpu9250GyroRead((uint16_t*)sensorData[6]);
    // Read accelerometer data
    sensorMpu9250AccRead((uint16_t*)sensorData);

//        sensorMpuSleep();

    // convert from uint8_t to float
    Movement_ConvertSensorData();
    // apply low pass filter to accelerometer
//    float accLowPassFilterY = Movement_LowPassFilter(acc.y) * 180 / M_PI;

    // Create the 16 bit integer values for the Accelerometer values
    xx = sensorData[1];
	xx *= 256;
	xx+= sensorData[0];

	yy = sensorData[3];
	yy *= 256;
	yy+= sensorData[2];

	rawzz = sensorData[5];
	rawzz *= 256;
	rawzz+= sensorData[4];

	// Value read seems to be 1/2 of what I expect.
//	rawzz *=2;

	intz = rawzz/1000;
	zz = rawzz-(intz*1000);


	// Low pass filter
	lastz = (lastz<< beta) - lastz;

	lastz += rawzz;
	lastz >>= beta;

    acc.z = sensorMpu9250AccConvert(rawzz);
	FloatToStringNew(accstr, acc.z);
	//	xx = (int)(acc.x * 100);
//	yy = (int)(acc.y * 100);
//	zz = (int)(acc.z * 100);

	// Note that System_printf does not support float and does not add a CR to the LF in \n
	System_printf("Z %s, RawZ %d, Filtered Z: %d\n", accstr, rawzz, lastz);

    memcpy(data,sensorData,sizeof(sensorData));

}


/* A utility function to reverse a string  */
void reverse(char str[], int length)
{
    int start = 0;
    char tmp;
    int end = length -1;
    while (start < end)
    {
//        swap(*(str+start), *(str+end));
        tmp = *(str+start);
        *(str+start) = *(str+end);
        *(str+end) = tmp;
        start++;
        end--;
    }
}

char* itoa(int16_t num, char* str, int base)
{
    int i = 0;
    bool isNegative = false;

    /* Handle 0 explicitely, otherwise empty string is printed for 0 */
    if (num == 0)
    {
        str[i++] = '0';
        str[i] = '\0';
        return str;
    }

    // In standard itoa(), negative numbers are handled only with
    // base 10. Otherwise numbers are considered unsigned.
    if (num < 0 && base == 10)
    {
        isNegative = true;
        num = -num;
    }

    // Process individual digits
    while (num != 0)
    {
        int rem = num % base;
        str[i++] = (rem > 9)? (rem-10) + 'a' : rem + '0';
        num = num/base;
    }

    // If number is negative, append '-'
    if (isNegative)
        str[i++] = '-';

    str[i] = '\0'; // Append string terminator

    // Reverse the string
    reverse(str, i);

    return str;
}

int getLen(char *buffer)
{
	int i = 1;
	while (*buffer !=0)
		{
			i++;
			buffer++;
		}
		return i;
}

void FloatToStringNew(char *str, float f)
{
 	char pos;  // position in string
 	char len;  // length of decimal part of result
 	char curr[3];  // temp holder for next digit
 	int value;  // decimal digit(s) to convert
 	pos = 0;  // initialize pos, just to be sure

 	value = (int)f;  // truncate the floating point number
 	itoa(value,str,10);  // this is kinda dangerous depending on the length of str
 	// now str array has the digits before the decimal

 	if (f < 0 )  // handle negative numbers
 	{
 		f *= -1;
 		value *= -1;
 	}

 	len = getLen(str);  // find out how big the integer part was
 	pos = len - 1;  // position the pointer to the end of the integer part
 	str[pos++] = '.';  // add decimal point to string

 	while(pos < (len + 1 + 3) )  // process remaining digits
 	{
 		f = f - (float)value;  // hack off the whole part of the number
 		f *= 10;  // move next digit over
 		value = (int)f;  // get next digit
 		itoa(value, curr,10); // convert digit to string
 		str[pos++] = curr[0]; // add digit to result string and increment pointer
 	}
}

