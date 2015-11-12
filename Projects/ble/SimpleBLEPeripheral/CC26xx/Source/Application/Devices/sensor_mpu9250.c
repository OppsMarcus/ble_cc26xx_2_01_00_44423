/*******************************************************************************
*  Filename:       sensor_mpu9250.c
*  Revised:        $Date: 2015-02-05 10:47:02 +0100 (on, 05 feb 2015) $
*  Revision:       $Revision: 12066 $
*
*  Description:    Driver for the InvenSense MPU9250 Motion processing Unit
*
*  Copyright (C) 2015 Texas Instruments Incorporated - http://www.ti.com/
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*    Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*
*    Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the distribution.
*
*    Neither the name of Texas Instruments Incorporated nor the names of
*    its contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
*  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

/* -----------------------------------------------------------------------------
*                                          Includes
* ------------------------------------------------------------------------------
*/
#include "Board.h"
#include "sensor_mpu9250.h"
//#include "sensor_opt3001.h" // For reset of I2C bus
//#include "sensor.h"
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include "bsp_i2c.h"

#include <xdc/runtime/System.h>
#include <math.h>


/* -----------------------------------------------------------------------------
*                                           Constants
* ------------------------------------------------------------------------------
*/
// Sensor I2C address
#define SENSOR_I2C_ADDRESS            0x68
#define SENSOR_MAG_I2_ADDRESS         0x0C

// Registers
#define SELF_TEST_X_GYRO              0x00 // R/W
#define SELF_TEST_Y_GYRO              0x01 // R/W
#define SELF_TEST_Z_GYRO              0x02 // R/W
#define SELF_TEST_X_ACCEL             0x0D // R/W
#define SELF_TEST_Z_ACCEL             0x0E // R/W
#define SELF_TEST_Y_ACCEL             0x0F // R/W

#define XG_OFFSET_H                   0x13 // R/W
#define XG_OFFSET_L                   0x14 // R/W
#define YG_OFFSET_H                   0x15 // R/W
#define YG_OFFSET_L                   0x16 // R/W
#define ZG_OFFSET_H                   0x17 // R/W
#define ZG_OFFSET_L                   0x18 // R/W

#define SMPLRT_DIV                    0x19 // R/W
#define CONFIG                        0x1A // R/W
#define GYRO_CONFIG                   0x1B // R/W
#define ACCEL_CONFIG                  0x1C // R/W
#define ACCEL_CONFIG_2                0x1D // R/W
#define LP_ACCEL_ODR                  0x1E // R/W
#define WOM_THR                       0x1F // R/W
#define FIFO_EN                       0x23 // R/W

// .. registers 0x24 - 0x36 are not applicable to the SensorTag HW configuration

#define INT_PIN_CFG                   0x37 // R/W
#define INT_ENABLE                    0x38 // R/W
#define INT_STATUS                    0x3A // R
#define ACCEL_XOUT_H                  0x3B // R
#define ACCEL_XOUT_L                  0x3C // R
#define ACCEL_YOUT_H                  0x3D // R
#define ACCEL_YOUT_L                  0x3E // R
#define ACCEL_ZOUT_H                  0x3F // R
#define ACCEL_ZOUT_L                  0x40 // R
#define TEMP_OUT_H                    0x41 // R
#define TEMP_OUT_L                    0x42 // R
#define GYRO_XOUT_H                   0x43 // R
#define GYRO_XOUT_L                   0x44 // R
#define GYRO_YOUT_H                   0x45 // R
#define GYRO_YOUT_L                   0x46 // R
#define GYRO_ZOUT_H                   0x47 // R
#define GYRO_ZOUT_L                   0x48 // R

// .. registers 0x49 - 0x60 are not applicable to the SensorTag HW configuration
// .. registers 0x63 - 0x67 are not applicable to the SensorTag HW configuration

#define SIGNAL_PATH_RESET             0x68 // R/W
#define ACCEL_INTEL_CTRL              0x69 // R/W
#define USER_CTRL                     0x6A // R/W
#define PWR_MGMT_1                    0x6B // R/W
#define PWR_MGMT_2                    0x6C // R/W
#define FIFO_COUNT_H                  0x72 // R/W
#define FIFO_COUNT_L                  0x73 // R/W
#define FIFO_R_W                      0x74 // R/W
#define WHO_AM_I                      0x75 // R/W

// Masks is mpuConfig valiable
#define ACC_CONFIG_MASK               0x38
#define GYRO_CONFIG_MASK              0x07

// Values PWR_MGMT_1
#define MPU_SLEEP                     0x4F  // Sleep + stop all clocks
#define MPU_WAKE_UP                   0x09  // Disable temp. + intern osc

// Values PWR_MGMT_2
#define ALL_AXES                      0x3F
#define GYRO_AXES                     0x07
#define ACC_AXES                      0x38

// Data sizes
#define DATA_SIZE                     6

// Output data rates
#define INV_LPA_0_3125HZ              0
#define INV_LPA_0_625HZ               1
#define INV_LPA_1_25HZ                2
#define INV_LPA_2_5HZ                 3
#define INV_LPA_5HZ                   4
#define INV_LPA_10HZ                  5
#define INV_LPA_20HZ                  6
#define INV_LPA_40HZ                  7
#define INV_LPA_80HZ                  8
#define INV_LPA_160HZ                 9
#define INV_LPA_320HZ                 10
#define INV_LPA_640HZ                 11
#define INV_LPA_STOPPED               255

// Bit values
#define BIT_ANY_RD_CLR                0x10
#define BIT_RAW_RDY_EN                0x01
#define BIT_WOM_EN                    0x40
#define BIT_LPA_CYCLE                 0x20
#define BIT_STBY_XA                   0x20
#define BIT_STBY_YA                   0x10
#define BIT_STBY_ZA                   0x08
#define BIT_STBY_XG                   0x04
#define BIT_STBY_YG                   0x02
#define BIT_STBY_ZG                   0x01
#define BIT_STBY_XYZA                 (BIT_STBY_XA | BIT_STBY_YA | BIT_STBY_ZA)
#define BIT_STBY_XYZG                 (BIT_STBY_XG | BIT_STBY_YG | BIT_STBY_ZG)

// User control register
#define BIT_LATCH_EN                  0x20
#define BIT_ACTL                      0x80

// INT Pin / Bypass Enable Configuration
#define BIT_BYPASS_EN                 0x02
#define BIT_AUX_IF_EN                 0x20

// Magnetometer registers
#define MAG_WHO_AM_I                  0x00  // Should return 0x48
#define MAG_INFO                      0x01
#define MAG_ST1                       0x02  // Data ready status: bit 0
#define MAG_XOUT_L	                  0x03  // Data array
#define MAG_XOUT_H	                  0x04
#define MAG_YOUT_L	                  0x05
#define MAG_YOUT_H	                  0x06
#define MAG_ZOUT_L	                  0x07
#define MAG_ZOUT_H	                  0x08
#define MAG_ST2                       0x09  // Overflow(bit 3), read err(bit 2)
#define MAG_CNTL1                     0x0A  // Mode bits 3:0, resolution bit 4
#define MAG_CNTL2                     0x0B  // System reset, bit 0
#define MAG_ASTC                      0x0C  // Self test control
#define MAG_I2CDIS                    0x0F  // I2C disable
#define MAG_ASAX                      0x10  // x-axis sensitivity adjustment
#define MAG_ASAY                      0x11  // y-axis sensitivity adjustment
#define MAG_ASAZ                      0x12  // z-axis sensitivity adjustment

#define MAG_DEVICE_ID                 0x48

// Mode
#define MAG_MODE_OFF                  0x00
#define MAG_MODE_SINGLE               0x01
#define MAG_MODE_CONT1                0x02
#define MAG_MODE_CONT2                0x06
#define MAG_MODE_FUSE                 0x0F

// Resolution
#define MFS_14BITS                    0     // 0.6 mG per LSB
#define MFS_16BITS                    1     // 0.15 mG per LSB

// Sensor selection/de-selection
#define SENSOR_SELECT()               bspI2cSelect(BSP_I2C_INTERFACE_1,SENSOR_I2C_ADDRESS)
#define SENSOR_SELECT_MAG()           bspI2cSelect(BSP_I2C_INTERFACE_1,SENSOR_MAG_I2_ADDRESS)
#define SENSOR_DESELECT()             bspI2cDeselect()

/* Self test assertion; return false (failed) if condition is not met */
#define ST_ASSERT(cond) st( if (!(cond)) {bspI2cDeselect(); return false;} )
#define ST_ASSERT_V(cond) st( if (!(cond)) {bspI2cDeselect(); return;} )
#define st(x)      do { x } while (__LINE__ == -1)
/* Delay */
#ifdef TI_DRIVERS_I2C_INCLUDED
#define delay_ms(i) Task_sleep( ((i) * 1000) / Clock_tickPeriod )
#define MS_2_TICKS(ms) ( ((ms) * 1000) / Clock_tickPeriod )
#else
#define delay_ms(i) ( CPUdelay(8000*(i)) )
#endif


/* Register values */
#define MANUFACTURER_ID                 0x5449  // TI
#define DEVICE_ID                       0x3001  // Opt 3001
#define REG_MANUFACTURER_ID             0x7E
#define REG_DEVICE_ID                   0x7F

/* Register length */
#define REGISTER_LENGTH                 2

/* -----------------------------------------------------------------------------
*                           Typedefs
* ------------------------------------------------------------------------------
*/

/* -----------------------------------------------------------------------------
*                           Macros
* ------------------------------------------------------------------------------
*/

/* -----------------------------------------------------------------------------
*                           Local Functions
* ------------------------------------------------------------------------------
*/
//static void sensorMpuSleep(void);
//static void sensorMpu9250WakeUp(void);
static void sensorMpu9250SelectAxes(void);
static void sensorMpu9250_Callback(PIN_Handle handle, PIN_Id pinId);
static void sensorMagInit(void);
static void sensorMagEnable(bool);
static bool sensorMpu9250SetBypass(void);
static void MPU9250SelfTest();
static void initMPU9250();


/* -----------------------------------------------------------------------------
*                           Local Variables
* ------------------------------------------------------------------------------
*/
static uint8_t mpuConfig;
static uint8_t magStatus;
static uint8_t accRange;
static uint8_t accRangeReg;
static uint8_t val;

// Magnetometer calibration
static int16_t calX;
static int16_t calY;
static int16_t calZ;

// Magnetometer control
static uint8_t scale = MFS_16BITS;      // 16 bit resolution
static uint8_t mode = MAG_MODE_SINGLE;  // Operating mode

// Pins that are used by the MPU9250
static PIN_Config MpuPinTable[] =
{
    Board_MPU_INT    | PIN_INPUT_EN | PIN_PULLDOWN | PIN_IRQ_DIS | PIN_HYSTERESIS,
    Board_MPU_POWER  | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX,

    PIN_TERMINATE
};
static PIN_State pinGpioState;
static PIN_Handle hMpuPin;

// The application may register a callback to handle interrupts
static MpuCallbackFn_t isrCallbackFn = NULL;


static uint8_t buffer[32];
/*******************************************************************************
 * @fn          sensorReadReg
 *
 * @brief       This function implements the I2C protocol to read from a sensor.
 *              The sensor must be selected before this routine is called.
 *
 * @param       addr - which register to read
 * @param       pBuf - pointer to buffer to place data
 * @param       nBytes - number of bytes to read
 *
 * @return      TRUE if the required number of bytes are received
 ******************************************************************************/
bool sensorReadReg(uint8_t addr, uint8_t *pBuf, uint8_t nBytes)
{
  return bspI2cWriteRead(&addr,1,pBuf,nBytes);
}
/*******************************************************************************
* @fn          sensorWriteReg
* @brief       This function implements the I2C protocol to write to a sensor.
*              The sensor must be selected before this routine is called.
*
* @param       addr - which register to write
* @param       pBuf - pointer to buffer containing data to be written
* @param       nBytes - number of bytes to write
*
* @return      TRUE if successful write
*/
bool sensorWriteReg(uint8_t addr, uint8_t *pBuf, uint8_t nBytes)
{
  uint8_t i;
  uint8_t *p = buffer;
  /* Copy address and data to local buffer for burst write */
  *p++ = addr;
  for (i = 0; i < nBytes; i++)
  {
    *p++ = *pBuf++;
  }
  nBytes++;
  /* Send data */
  return bspI2cWrite(buffer,nBytes);
}

/*******************************************************************************
* @fn          sensorMpu9250PowerOn
*
* @brief       This function turns on the power supply to MPU9250
*
* @return      none
*/
void sensorMpu9250PowerOn(void)
{
  // Turn on power supply
  PIN_setOutputValue(hMpuPin,Board_MPU_POWER, Board_MPU_POWER_ON);
  System_printf("sensor_mpu9250: sensorMpu9250PowerOn\n");
  delay_ms(100);
  sensorMpu9250Reset();
}

/*******************************************************************************
* @fn          sensorMpu9250PowerOff
*
* @brief       This function turns off the power supply to MPU9250
*
* @return      none
*/
void sensorMpu9250PowerOff(void)
{
  // Make sure pin interrupt is disabled
  PIN_setInterrupt(hMpuPin, PIN_ID(Board_MPU_INT)|PIN_IRQ_DIS);

  // Turn off power supply
  PIN_setOutputValue(hMpuPin,Board_MPU_POWER, Board_MPU_POWER_OFF);

  // Force an access on I2C bus #0 (sets the I2C lines to a defined state)
  sensorOpt3001Test();
  System_printf("sensor_mpu9250: sensorMpu9250PowerOff\n");
}

/*******************************************************************************
* @fn          sensorMpu9250PowerIsOn
*
* @brief       Return 'true' if MPU power is on
*
* @return      state of MPU power
*/
bool sensorMpu9250PowerIsOn(void)
{
 return PIN_getOutputValue(Board_MPU_POWER) == Board_MPU_POWER_ON;
}

/*******************************************************************************
* @fn          sensorMpu9250RegisterCallback
*
* @brief       Register a call-back for interrupt processing
*
* @return      none
*/
void sensorMpu9250RegisterCallback(MpuCallbackFn_t pfn)
{
  isrCallbackFn = pfn;
}

/*******************************************************************************
* @fn          sensorMpu9250Init
*
* @brief       This function initializes the MPU abstraction layer.
*
* @return      True if success
*/
bool sensorMpu9250Init(void)
{
  System_printf("sensor_mpu9250: sensorMpu9250Init\n");

  // Pins used by MPU
  hMpuPin = PIN_open(&pinGpioState, MpuPinTable);

  // Register MPU interrupt
  PIN_registerIntCb(hMpuPin, sensorMpu9250_Callback);

  // Application callback initially NULL
  isrCallbackFn = NULL;

  return sensorMpu9250Reset();
}


/*******************************************************************************
* @fn          sensorMpu9250Reset
*
* @brief       This function resets the MPU
*
* @return      True if success
*/
bool sensorMpu9250Reset(void)
{
  bool ret;
  System_printf("sensor_mpu9250: sensorMpu9250Reset\n");

  // Make sure pin interrupt is disabled
  PIN_setInterrupt(hMpuPin, PIN_ID(Board_MPU_INT)|PIN_IRQ_DIS);

  accRange = ACC_RANGE_INVALID;
  mpuConfig = 0;   // All axes off
  magStatus = 0;

  if (!SENSOR_SELECT())
  {
    return false;
  }

  // Device reset
  val = 0x80;
  sensorWriteReg(PWR_MGMT_1, &val, 1);
  SENSOR_DESELECT();

  delay_ms(100);

  ret = sensorMpu9250Test();
  if (ret)
  {
    // Initial configuration
    sensorMpu9250AccSetRange(ACC_RANGE_8G);
    sensorMagInit();

    // Power save
//    sensorMpuSleep();		// Can't seem to wake it up after it is put to sleep

  }

  return ret;
}


/*******************************************************************************
* @fn          sensorMpu9250WomEnable
*
* @brief       Enable Wake On Motion functionality
*
* @param       threshold - wake-up trigger threshold (unit: 4 mg, max 1020mg)
*
* @return      True if success
*/
bool sensorMpu9250WomEnable(uint8_t threshold)
{
  ST_ASSERT(sensorMpu9250PowerIsOn());
  System_printf("sensor_mpu9250: sensorMpu9250WomEnable\n");

  if (!SENSOR_SELECT())
  {
    return false;
  }

  // Make sure accelerometer is running
  val = 0x09;
  ST_ASSERT(sensorWriteReg(PWR_MGMT_1, &val, 1));

  // Enable accelerometer, disable gyro
  val = 0x07;
  ST_ASSERT(sensorWriteReg(PWR_MGMT_2, &val, 1));

  // Set Accel LPF setting to 184 Hz Bandwidth
  val = 0x01;
  ST_ASSERT(sensorWriteReg(ACCEL_CONFIG_2, &val, 1));

  // Enable Motion Interrupt
  val = BIT_WOM_EN;
  ST_ASSERT(sensorWriteReg(INT_ENABLE, &val, 1));

  // Enable Accel Hardware Intelligence
  val = 0xC0;
  ST_ASSERT(sensorWriteReg(ACCEL_INTEL_CTRL, &val, 1));

  // Set Motion Threshold
  val = threshold;
  ST_ASSERT(sensorWriteReg(WOM_THR, &val, 1));

  // Set Frequency of Wake-up
  val = INV_LPA_20HZ;
  ST_ASSERT(sensorWriteReg(LP_ACCEL_ODR, &val, 1));

  // Enable Cycle Mode (Accel Low Power Mode)
  val = 0x29;
  ST_ASSERT(sensorWriteReg(PWR_MGMT_1, &val, 1));

  // Select the current range
  ST_ASSERT(sensorWriteReg(ACCEL_CONFIG, &accRangeReg, 1));

  // Clear interrupt
  sensorReadReg(INT_STATUS,&val,1);

  SENSOR_DESELECT();

  mpuConfig = 0;

  // Enable pin for wake-on-motion interrupt
  PIN_setInterrupt(hMpuPin, PIN_ID(Board_MPU_INT)|PIN_IRQ_POSEDGE);

  return true;
}

/*******************************************************************************
* @fn          sensorMpu9250IntStatus
*
* @brief       Check whether a data or wake on motion interrupt has occurred
*
* @return      Return interrupt status
*/
uint8_t sensorMpu9250IntStatus(void)
{
  uint8_t intStatus;

  intStatus = 0;
  ST_ASSERT(sensorMpu9250PowerIsOn());

  if (SENSOR_SELECT())
  {
    if (!sensorReadReg(INT_STATUS,&intStatus,1))
    {
      intStatus = 0;
    }
    SENSOR_DESELECT();
  }

  return intStatus;
}

/*******************************************************************************
* @fn          sensorMpu9250Enable
*
* @brief       Enable accelerometer readout
*
* @param       Axes: Gyro bitmap [0..2], X = 1, Y = 2, Z = 4. 0 = gyro off
* @                  Acc  bitmap [3..5], X = 8, Y = 16, Z = 32. 0 = accelerometer off
*                    MPU  bit [6], all axes
*
* @return      None
*/
void sensorMpu9250Enable(uint16_t axes)
{
  ST_ASSERT_V(sensorMpu9250PowerIsOn());
  System_printf("sensor_mpu9250: sensorMpu9250Enable 0x%x\n", axes);

  if (mpuConfig == 0 && axes != 0)
  {
    // Wake up the sensor if it was off
    sensorMpu9250WakeUp();
  }

  mpuConfig = axes;

  if (mpuConfig != 0)
  {
    // Enable gyro + accelerometer + magnetometer readout
    sensorMpu9250SelectAxes();
  }
  else if (mpuConfig == 0)
  {
     sensorMpuSleep();
  }
}


/*******************************************************************************
* @fn          sensorMpu9250AccSetRange
*
* @brief       Set the range of the accelerometer
*
* @param       newRange: ACC_RANGE_2G, ACC_RANGE_4G, ACC_RANGE_8G, ACC_RANGE_16G
*
* @return      true if write succeeded
*/
bool sensorMpu9250AccSetRange(uint8_t newRange)
{
  bool success;
  System_printf("sensor_mpu9250: sensorMpu9250SetRange 0x%x\n", newRange);

  if (newRange == accRange)
  {
    return true;
  }

  ST_ASSERT(sensorMpu9250PowerIsOn());

  if (!SENSOR_SELECT())
  {
    return false;
  }

  accRangeReg = (newRange << 3);

  // Apply the range
  success = sensorWriteReg(ACCEL_CONFIG, &accRangeReg, 1);
  SENSOR_DESELECT();

  if (success)
  {
    accRange = newRange;
  }

  return success;
}

/*******************************************************************************
* @fn          sensorMpu9250AccReadRange
*
* @brief       Apply the selected accelerometer range
*
* @param       none
*
* @return      range: ACC_RANGE_2G, ACC_RANGE_4G, ACC_RANGE_8G, ACC_RANGE_16G
*/
uint8_t sensorMpu9250AccReadRange(void)
{
  ST_ASSERT(sensorMpu9250PowerIsOn());

  if (!SENSOR_SELECT())
  {
    return false;
  }

  // Apply the range
  sensorReadReg(ACCEL_CONFIG, &accRangeReg, 1);
  SENSOR_DESELECT();

  accRange = (accRangeReg>>3) & 3;

  return accRange;
}


/*******************************************************************************
* @fn          sensorMpu9250AccRead
*
* @brief       Read data from the accelerometer - X, Y, Z - 3 words
*
* @return      True if data is valid
*/
bool sensorMpu9250AccRead(uint16_t *data )
{
  bool success;

  ST_ASSERT(sensorMpu9250PowerIsOn());

  // Burst read of all accelerometer values
  if (!SENSOR_SELECT())
  {
    return false;
  }

  success = sensorReadReg(ACCEL_XOUT_H, (uint8_t*)data, DATA_SIZE);
  SENSOR_DESELECT();

  if (success)
  {
    convertToLe((uint8_t*)data,DATA_SIZE);
  }

  return success;
}

/*******************************************************************************
* @fn          sensorMpu9250GyroRead
*
* @brief       Read data from the gyroscope - X, Y, Z - 3 words
*
* @return      TRUE if valid data, FALSE if not
*/
bool sensorMpu9250GyroRead(uint16_t *data )
{
  bool success;

  ST_ASSERT(sensorMpu9250PowerIsOn());

  // Select this sensor
  if (!SENSOR_SELECT())
  {
    return false;
  }

  // Burst read of all gyroscope values
  success = sensorReadReg(GYRO_XOUT_H, (uint8_t*)data, DATA_SIZE);

  SENSOR_DESELECT();

  if (success)
  {
    convertToLe((uint8_t*)data,DATA_SIZE);
  }

  return success;
}


/*******************************************************************************
 * @fn          sensorMpu9250Test
 *
 * @brief       Run a sensor self-test
 *
 * @return      TRUE if passed, FALSE if failed
 */
bool sensorMpu9250Test(void)
{
  static bool first = true;

  ST_ASSERT(sensorMpu9250PowerIsOn());

  // Select Gyro/Accelerometer
  if (!SENSOR_SELECT())
  {
    return false;
  }

  // Make sure power is ramped up
  if (first)
  {
    delay_ms(100);
    first = false;
  }

  // Check the WHO AM I register
  ST_ASSERT(sensorReadReg(WHO_AM_I, &val, 1));
  ST_ASSERT(val == 0x71);
  System_printf("sensor_mpu9250: sensorMpu9250Test - WHO_AM_I 0x%x\n", val);

  SENSOR_DESELECT();
//  MPU9250SelfTest();
//  initMPU9250();

  return true;
}


/*******************************************************************************
 * @fn          sensorMpu9250AccConvert
 *
 * @brief       Convert raw data to G units
 *
 * @param       rawData - raw data from sensor
 *
 * @return      Converted value
 ******************************************************************************/
float sensorMpu9250AccConvert(int16_t rawData)
{
  float v;

  switch (accRange)
  {
  case ACC_RANGE_2G:
    //-- calculate acceleration, unit G, range -2, +2
    v = (rawData * 1.0) / (32768/2);
    break;

  case ACC_RANGE_4G:
    //-- calculate acceleration, unit G, range -4, +4
    v = (rawData * 1.0) / (32768/4);
    break;

  case ACC_RANGE_8G:
    //-- calculate acceleration, unit G, range -8, +8
    v = (rawData * 1.0) / (32768/8);
    break;

  case ACC_RANGE_16G:
    //-- calculate acceleration, unit G, range -16, +16
    v = (rawData * 1.0) / (32768/16);
    break;
  }
//  System_printf("sensor_mpu9250: sensorMpu9250AccConvert Raw:%d,  Converted %d\n",rawData, (int)(v*1000));

  return v;
}

/*******************************************************************************
 * @fn          sensorMpu9250GyroConvert
 *
 * @brief       Convert raw data to deg/sec units
 *
 * @param       data - raw data from sensor
 *
 * @return      none
 ******************************************************************************/
float sensorMpu9250GyroConvert(int16_t data)
{
  //-- calculate rotation, unit deg/s, range -250, +250
  return (data * 1.0) / (65536 / 500);
}



/*******************************************************************************
* @fn          sensorMpuSleep
*
* @brief       Place the MPU in low power mode
*
* @return
*/
void sensorMpuSleep(void)
{
  bool success;
  System_printf("sensor_mpu9250: sensorMpu9250Sleep\n");

  ST_ASSERT_V(sensorMpu9250PowerIsOn());

  if (!SENSOR_SELECT())
  {
    return;
  }

  val = ALL_AXES;
  success = sensorWriteReg(PWR_MGMT_2, &val, 1);
  if (success)
  {
    val = MPU_SLEEP;
    success = sensorWriteReg(PWR_MGMT_1, &val, 1);
  }

  SENSOR_DESELECT();
}


/*******************************************************************************
* @fn          sensorMpu9250WakeUp
*
* @brief       Exit low power mode
*
* @return      none
*/

void sensorMpu9250WakeUp(void)
{
  bool success;
  System_printf("sensor_mpu9250: sensorMpu9250Wakeup\n");

  ST_ASSERT_V(sensorMpu9250PowerIsOn());

  if (!SENSOR_SELECT())
  {
    return;
  }

  val = MPU_WAKE_UP;
  success = sensorWriteReg(PWR_MGMT_1, &val, 1);

  if (success)
  {
    // All axis initially disabled
    val = ALL_AXES;
    success = sensorWriteReg(PWR_MGMT_2, &val, 1);
    mpuConfig = 0;
  }

  if (success)
  {
    // Restore the range
    success = sensorWriteReg(ACCEL_CONFIG, &accRangeReg, 1);

    if (success)
    {
      // Clear interrupts
      success = sensorReadReg(INT_STATUS,&val,1);
    }
  }

  SENSOR_DESELECT();
}


/*******************************************************************************
* @fn          sensorMpu9250SelectAxes
*
* @brief       Select gyro, accelerometer, magnetometer
*
* @return      none
*/
static void sensorMpu9250SelectAxes(void)
{
  val = ~mpuConfig;
  System_printf("sensor_mpu9250: sensorMpu9250SelectAxis 0x%x\n",val);

  if (!SENSOR_SELECT())
  {
    return;
  }

  sensorWriteReg(PWR_MGMT_2, &val, 1);

  SENSOR_DESELECT();

  sensorMagEnable(!!(mpuConfig & MPU_AX_MAG));
}


/*******************************************************************************
* @fn          sensorMpu9250SetBypass
*
* @brief       Allow the I2C bus to control the compass
*
* @return      none
*/
static bool sensorMpu9250SetBypass(void)
{
  bool success;

  if (SENSOR_SELECT())
  {
    val = BIT_BYPASS_EN | BIT_LATCH_EN;
    success = sensorWriteReg(INT_PIN_CFG, &val, 1);
    delay_ms(10);

    SENSOR_DESELECT();
  }
  else
  {
    success = false;
  }

  return success;
}


/*******************************************************************************
* @fn          sensorMagInit
*
* @brief       Initialize the compass
*
* @return      none
*/
static void sensorMagInit(void)
{
  ST_ASSERT_V(sensorMpu9250PowerIsOn());

  if (!sensorMpu9250SetBypass())
  {
    return;
  }

  if (SENSOR_SELECT_MAG())
  {
    static uint8_t rawData[3];

    // Enter Fuse ROM access mode
    val = MAG_MODE_FUSE;
    sensorWriteReg(MAG_CNTL1, &val, 1);
    delay_ms(10);

    // Get calibration data
    if (sensorReadReg(MAG_ASAX, &rawData[0], 3))
    {
      calX =  (int16_t)rawData[0] + 128;   // Return x-axis sensitivity adjustment values, etc.
      calY =  (int16_t)rawData[1] + 128;
      calZ =  (int16_t)rawData[2] + 128;
    }

    // Turn off the sensor by doing a reset
    val = 0x01;
    sensorWriteReg(MAG_CNTL2, &val, 1);

    SENSOR_DESELECT();
  }
}


/*******************************************************************************
 * @fn          sensorMpu9250MagReset
 *
 * @brief       Reset the magnetometer
 *
 * @return      none
 */
void sensorMpu9250MagReset(void)
{
  ST_ASSERT_V(sensorMpu9250PowerIsOn());

  if (sensorMpu9250SetBypass())
  {
    if (SENSOR_SELECT_MAG())
    {
      // Turn off the sensor by doing a reset
      val = 0x01;
      sensorWriteReg(MAG_CNTL2, &val, 1);
      delay_ms(10);

      // Re-enable if already active
      if (mpuConfig & MPU_AX_MAG)
      {
        val = (scale << 4) | mode;
        sensorWriteReg(MAG_CNTL1, &val, 1); // Set magnetometer data resolution and sample ODR
      }
      SENSOR_DESELECT();
    }
  }
}


/*******************************************************************************
 * @fn          sensorMagEnable
 *
 * @brief       Enable or disable the compass part of the MPU9250
 *
 * @return      none
 */
static void sensorMagEnable(bool enable)
{
  uint8_t val;

  ST_ASSERT_V(sensorMpu9250PowerIsOn());

  if (!sensorMpu9250SetBypass())
  {
    return;
  }

  if (SENSOR_SELECT_MAG())
  {
    if (enable)
    {
      // Set magnetometer data resolution and sample ODR
      val = (scale << 4) | mode;
    }
    else
    {
      // Power down magnetometer
      val = 0x00;
    }
    sensorWriteReg(MAG_CNTL1, &val, 1);

    SENSOR_DESELECT();
  }
}

/*******************************************************************************
 * @fn          sensorMpu9250MagTest
 *
 * @brief       Run a magnetometer self test
 *
 * @return      TRUE if passed, FALSE if failed
 */
bool sensorMpu9250MagTest(void)
{
  ST_ASSERT(sensorMpu9250PowerIsOn());

  // Connect magnetometer internally in MPU9250
  sensorMpu9250SetBypass();

  // Select magnetometer
  SENSOR_SELECT_MAG();

  // Check the WHO AM I register
  val = 0xFF;
  ST_ASSERT(sensorReadReg(MAG_WHO_AM_I, &val, 1));
  ST_ASSERT(val == MAG_DEVICE_ID);

  SENSOR_DESELECT();

  return true;
}

/*******************************************************************************
* @fn          sensorMpu9250MagRead
*
* @brief       Read data from the compass - X, Y, Z - 3 words
*
* @return      Magnetometer status
*/
uint8_t sensorMpu9250MagRead(int16_t *data)
{
  uint8_t val;
  uint8_t rawData[7];  // x/y/z compass register data, ST2 register stored here,
                       // must read ST2 at end of data acquisition
  magStatus = MAG_NO_POWER;
  ST_ASSERT(sensorMpu9250PowerIsOn());

  magStatus = MAG_STATUS_OK;

  // Connect magnetometer internally in MPU9250
  SENSOR_SELECT();
  val = BIT_BYPASS_EN | BIT_LATCH_EN;
  if (!sensorWriteReg(INT_PIN_CFG, &val, 1))
  {
    magStatus = MAG_BYPASS_FAIL;
  }
  SENSOR_DESELECT();

  if (magStatus != MAG_STATUS_OK)
  {
    return false;
  }

  // Select this sensor
  SENSOR_SELECT_MAG();

  if (sensorReadReg(MAG_ST1,&val,1))
  {
    // Check magnetometer data ready bit
    if (val & 0x01)
    {
      // Burst read of all compass values + ST2 register
      if (sensorReadReg(MAG_XOUT_L, &rawData[0],7))
      {
        val = rawData[6]; // ST2 register

        // Check if magnetic sensor overflow set, if not then report data
        if(!(val & 0x08))
        {
          data[0] = ((int16_t)rawData[1] << 8) | rawData[0];  // Turn the MSB and LSB into a signed 16-bit value
          data[1] = ((int16_t)rawData[3] << 8) | rawData[2];  // Data stored as little Endian
          data[2] = ((int16_t)rawData[5] << 8) | rawData[4];

          // Sensitivity adjustment
          data[0] = data[0] * calX >> 8;
          data[1] = data[1] * calY >> 8;
          data[2] = data[2] * calZ >> 8;
        }
        else
        {
          magStatus = MAG_OVERFLOW;
        }
      }
      else
      {
        magStatus = MAG_READ_DATA_ERR;
      }
    }
    else
    {
      magStatus = MAG_DATA_NOT_RDY;
    }
  }
  else
  {
    magStatus = MAG_READ_ST_ERR;
  }

  // Start new conversion
  val = (scale << 4) | mode;
  sensorWriteReg(MAG_CNTL1, &val, 1); // Set magnetometer data resolution and sample ODR

  SENSOR_DESELECT();

  return magStatus;
}


/*******************************************************************************
* @fn          sensorMpu9250MagStatus
*
* @brief       Return the magnetometer status
*
* @return      mag status
*/
uint8_t sensorMpu9250MagStatus(void)
{
  return magStatus;
}

/*******************************************************************************
 *  @fn         sensorMpu9250_Callback
 *
 *  Interrupt service routine for the MPU
 *
 *  @param      handle PIN_Handle connected to the callback
 *
 *  @param      pinId  PIN_Id of the DIO triggering the callback
 *
 *  @return     none
 ******************************************************************************/
static void sensorMpu9250_Callback(PIN_Handle handle, PIN_Id pinId)
{
  if (pinId == Board_MPU_INT)
  {
    if (isrCallbackFn != NULL)
    {
      isrCallbackFn();
    }
  }
}

/*******************************************************************************
 * @fn      convertToLe
 *
 * @brief   Convert 16-bit words form big-endian to little-endian
 *
 * @param   none
 *
 * @return  none
 */
void convertToLe(uint8_t *data, uint8_t len)
{
  uint8_t i;

  for (i=0; i<len; i+=2)
  {
    uint8_t tmp;
    tmp = data[i];
    data[i] = data[i+1];
    data[i+1] = tmp;
  }
}

/*******************************************************************************
 * @fn          sensorOpt3001Test
 *
 * @brief       Run a sensor self-test
 *
 * @return      TRUE if passed, FALSE if failed
 ******************************************************************************/
bool sensorOpt3001Test(void)
{
  uint16_t val;

  // Select this sensor on the I2C bus
  if (!SENSOR_SELECT())
    return false;

  // Check manufacturer ID
  ST_ASSERT(sensorReadReg(REG_MANUFACTURER_ID, (uint8_t *)&val, REGISTER_LENGTH));
  val = (LO_UINT16(val) << 8) | HI_UINT16(val);
  ST_ASSERT(val == MANUFACTURER_ID);

  // Check device ID
  ST_ASSERT(sensorReadReg(REG_DEVICE_ID, (uint8_t *)&val, REGISTER_LENGTH));
  val = (LO_UINT16(val) << 8) | HI_UINT16(val);
  ST_ASSERT(val == DEVICE_ID);

  SENSOR_DESELECT();

  return true;
}

// Accelerometer and gyroscope self test; check calibration wrt factory settings
void MPU9250SelfTest() // Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass
{
   uint8_t rawData[6] = {0, 0, 0, 0, 0, 0};
   uint8_t selfTest[6];
   int16_t gAvg[3], aAvg[3], aSTAvg[3], gSTAvg[3];
   float factoryTrim[6];
   float destination[6];
   uint8_t FS = 0;
   uint8_t i,ii;

  val = 0x00;
  sensorWriteReg(SMPLRT_DIV, &val,1);    // Set gyro sample rate to 1 kHz
  val = 0x02;
  sensorWriteReg(CONFIG, &val,1);        // Set gyro sample rate to 1 kHz and DLPF to 92 Hz
  val = 1<<FS;
  sensorWriteReg(GYRO_CONFIG, &val,1);  // Set full scale range for the gyro to 250 dps
  val = 0x02;
  sensorWriteReg(ACCEL_CONFIG_2, &val,1); // Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
  val = 0x1<<FS;
  sensorWriteReg(ACCEL_CONFIG, &val,1); // Set full scale range for the accelerometer to 2 g

  for(ii = 0; ii < 200; ii++)
  {  // get average current values of gyro and acclerometer

	  sensorReadReg(ACCEL_XOUT_H, &rawData[0],6);        // Read the six raw data registers into data array
	  aAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
	  aAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
	  aAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;

		sensorReadReg(GYRO_XOUT_H, &rawData[0],6);       // Read the six raw data registers sequentially into data array
	  gAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
	  gAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
	  gAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;
  }

  for (ii =0; ii < 3; ii++) {  // Get average of 200 values and store as average current readings
  aAvg[ii] /= 200;
  gAvg[ii] /= 200;
  }

// Configure the accelerometer for self-test
  val = 0xe0;
  sensorWriteReg(ACCEL_CONFIG, &val,1); // Enable self test on all three axes and set accelerometer range to +/- 2 g
   sensorWriteReg(GYRO_CONFIG, &val,1); // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
   delay_ms(25);  // Delay a while to let the device stabilize

  for(ii = 0; ii < 200; ii++) {  // get average self-test values of gyro and acclerometer

  sensorReadReg(ACCEL_XOUT_H, &rawData[0],6);  // Read the six raw data registers into data array
  aSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
  aSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
  aSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;

    sensorReadReg(GYRO_XOUT_H, &rawData[0],6);  // Read the six raw data registers sequentially into data array
  gSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
  gSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
  gSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;
  }

  for (ii =0; ii < 3; ii++) {  // Get average of 200 values and store as average self-test readings
  aSTAvg[ii] /= 200;
  gSTAvg[ii] /= 200;
  }

 // Configure the gyro and accelerometer for normal operation
  val = 0x00;
   sensorWriteReg(ACCEL_CONFIG, &val,1);
   sensorWriteReg(GYRO_CONFIG,  &val,1);
   delay_ms(25);  // Delay a while to let the device stabilize

   // Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
   selfTest[0] = sensorReadReg(SELF_TEST_X_ACCEL,&val,1); // X-axis accel self-test results
   selfTest[1] = sensorReadReg(SELF_TEST_Y_ACCEL,&val,1); // Y-axis accel self-test results
   selfTest[2] = sensorReadReg(SELF_TEST_Z_ACCEL,&val,1); // Z-axis accel self-test results
   selfTest[3] = sensorReadReg(SELF_TEST_X_GYRO,&val,1);  // X-axis gyro self-test results
   selfTest[4] = sensorReadReg(SELF_TEST_Y_GYRO,&val,1);  // Y-axis gyro self-test results
   selfTest[5] = sensorReadReg(SELF_TEST_Z_GYRO,&val,1);  // Z-axis gyro self-test results

  // Retrieve factory self-test value from self-test code reads
   factoryTrim[0] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[0] - 1.0) )); // FT[Xa] factory trim calculation
   factoryTrim[1] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[1] - 1.0) )); // FT[Ya] factory trim calculation
   factoryTrim[2] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[2] - 1.0) )); // FT[Za] factory trim calculation
   factoryTrim[3] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[3] - 1.0) )); // FT[Xg] factory trim calculation
   factoryTrim[4] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[4] - 1.0) )); // FT[Yg] factory trim calculation
   factoryTrim[5] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[5] - 1.0) )); // FT[Zg] factory trim calculation

 // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
 // To get percent, must multiply by 100
   for (i = 0; i < 3; i++) {
     destination[i]   = 100.0*((float)(aSTAvg[i] - aAvg[i]))/factoryTrim[i];   // Report percent differences
     destination[i+3] = 100.0*((float)(gSTAvg[i] - gAvg[i]))/factoryTrim[i+3]; // Report percent differences
   }

   System_printf("sensor_mpu9250: MPU9250init x accel trim within %d percent\n",(int)(destination[0]));
   System_printf("sensor_mpu9250: MPU9250init y accel trim within %d percent\n",(int)(destination[1]));
   System_printf("sensor_mpu9250: MPU9250init accel trim within %d percent\n",(int)(destination[2]));
   System_printf("sensor_mpu9250: MPU9250init x gyro trim within %d percent\n",(int)(destination[3]));
   System_printf("sensor_mpu9250: MPU9250init y gyro trim within %d percent\n",(int)(destination[4]));
   System_printf("sensor_mpu9250: MPU9250init z gyro trim within %d percent\n",(int)(destination[5]));

}

void initMPU9250()
{
	uint8_t c;
	uint8_t Gscale = 0;
	uint8_t Ascale = ACC_RANGE_8G;

 // wake up device
	val = 0x00;
  sensorWriteReg(PWR_MGMT_1, &val,1); // Clear sleep mode bit (6), enable all sensors
  delay_ms(100); // Wait for all registers to reset

 // get stable time source
	val = 0x01;
  sensorWriteReg(PWR_MGMT_1, &val,1);  // Auto select clock source to be PLL gyroscope reference if ready else
  delay_ms(200);

 // Configure Gyro and Thermometer
 // Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively;
 // minimum delay time for this setting is 5.9 ms, which means sensor fusion update rates cannot
 // be higher than 1 / 0.0059 = 170 Hz
 // DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
 // With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz, or 1 kHz
	val = 0x03;
  sensorWriteReg(CONFIG, &val,1);

 // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
	val = 0x04;
sensorWriteReg(SMPLRT_DIV, &val,1);  // Use a 200 Hz rate; a rate consistent with the filter update rate
                                    // determined inset in CONFIG above

 // Set gyroscope full scale range
 // Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
  sensorReadReg(GYRO_CONFIG,&c,1);
//  writeRegister(GYRO_CONFIG, c & ~0xE0); // Clear self-test bits [7:5]
	val = c & ~0x02;
  sensorWriteReg(GYRO_CONFIG, &val,1); // Clear Fchoice bits [1:0]
	val = c & ~0x18;
  sensorWriteReg(GYRO_CONFIG, &val,1); // Clear AFS bits [4:3]
	val = c | Gscale << 3;
  sensorWriteReg(GYRO_CONFIG, &val,1); // Set full scale range for the gyro
 // writeRegister(GYRO_CONFIG, c | 0x00); // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of GYRO_CONFIG

 // Set accelerometer full-scale range configuration
  sensorReadReg(ACCEL_CONFIG,&c,1);
//  writeRegister(ACCEL_CONFIG, c & ~0xE0); // Clear self-test bits [7:5]
	val = c & ~0x18;
  sensorWriteReg(ACCEL_CONFIG, &val,1); // Clear AFS bits [4:3]
	val = c | Ascale << 3;
  sensorWriteReg(ACCEL_CONFIG, &val,1); // Set full scale range for the accelerometer

 // Set accelerometer sample rate configuration
 // It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
 // accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
  sensorReadReg(ACCEL_CONFIG_2,&c,1);
  val = c & ~0x0F;
  sensorWriteReg(ACCEL_CONFIG_2, &val,1); // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
  val = c | 0x03;
  sensorWriteReg(ACCEL_CONFIG_2, &val,1); // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz

 // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
 // but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

  // Configure Interrupts and Bypass Enable
  // Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
  // clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips
  // can join the I2C bus and all can be controlled by the Arduino as master
  val = 0x22;
   sensorWriteReg(INT_PIN_CFG, &val,1);
   val = 0x01;
   sensorWriteReg(INT_ENABLE, &val,1);  // Enable data ready (bit 0) interrupt
   delay_ms(100);
}
