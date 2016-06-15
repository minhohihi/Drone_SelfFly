
#define __DEBUG__                           (1)
#if (__DEBUG__)
    #define __PRINT_DEBUG__                 (0)
    #define __PROFILE__                     (0)
    #define __EXTERNAL_READ__               (1)
    #define SERIAL_BAUDRATE                 (115200)
#else
    #define __PRINT_DEBUG__                 (1)
    #define __PROFILE__                     (0)
#endif


/*----------------------------------------------------------------------------------------
 File Inclusions
 ----------------------------------------------------------------------------------------*/
#include <EEPROM.h>
#include <I2Cdev.h>
#include <Wire.h>
//#include <MPU6050_6Axis_MotionApps20.h>
#include <HMC5883L.h>
#include <MS561101BA.h>
#include <math.h>


/*----------------------------------------------------------------------------------------
 Constant Definitions
 ----------------------------------------------------------------------------------------*/
#include "CommHeader.h"


/*----------------------------------------------------------------------------------------
 Macro Definitions
 ----------------------------------------------------------------------------------------*/
#if __DEBUG__
    #define Serialprint(...)                Serial.print(__VA_ARGS__)
    #define Serialprintln(...)              Serial.println(__VA_ARGS__)
#else
    #define Serialprint(...)
    #define Serialprintln(...)
#endif


/*----------------------------------------------------------------------------------------
 Type Definitions
 ----------------------------------------------------------------------------------------*/
typedef enum _RC_CH_Type
{
    CH_TYPE_ROLL                = 0,
    CH_TYPE_PITCH,
    CH_TYPE_THROTTLE,
    CH_TYPE_YAW,
    CH_TYPE_TAKE_LAND,
    CH_TYPE_MAX,
}RC_CH_Type;

typedef enum _DroneStatus
{
    DRONESTATUS_STOP            = 0,
    DRONESTATUS_READY,
    DRONESTATUS_START,
}DroneStatus;

typedef struct _AxisErrRate_T
{
    float           nP_ErrRate;
    float           nI_ErrRate;
    float           nD_ErrRate;
    float           nPrevErrRate;
    float           nBalance;
}AxisErrRate_T;

byte                nEEPROMData[EEPROM_DATA_MAX];

// For Accelerator & Gyroscope Sensor
double              nRawGyro[3];
double              nRawAccel[3];
double              nCalibMeanGyro[3];
double              nCalibMeanAccel[3];

// For Magnetometer Sensor
HMC5883L            nMagHndl;                               // HMC5883 Magnetic Interface
float               nRawMag[3];
double              nMagHeadingRad;
double              nMagHeadingDeg;
double              nSmoothHeadingDegrees;
double              nPrevHeadingDegrees;
double              nDeclinationAngle;

// For Barometer Sensor
MS561101BA          nBaroHndl;                              // MS5611 Barometer Interface
double              nRawTemp;                               // Raw Temperature Data
double              nRawPressure;                           // Raw Pressure Data
double              nRawAbsoluteAltitude;                   // Estimated Absolute Altitude
double              nAvgPressure;                           // Average Pressure Data
double              nAvgTemp;                               // Average Temperature Data
double              nAvgAbsoluteAltitude;                   // Average Absolute Altitude Data
double              nRelativeAltitude;                      // Relative Absolute Altitude Data
double              nPrevAvgAbsoluteAltitude;               // Average Absolute Altitude Data
double              nRefAbsoluteAltitude;                   // Reference Absolute Altitude Data
double              nVerticalSpeed;                         // Estimated Vertical Speed

// For Sonar Sensor
double              nRawDist;                               // Indicate Distance Calculated From Sensor
double              nDistFromGnd;                           // Indicate istance from Ground

// For PID Control
AxisErrRate_T       nRPY_PID[3];
//float               nPrevDErr_Roll, nPrevDErr_Pitch, nPrevDErr_Yaw;
//float               nIErr_Roll, nIErr_Pitch, nIErr_Yaw;
//float               nBalance_Roll, nBalance_Pitch, nBalance_Yaw;

// For Motor Control
int                 nLowRC[CH_TYPE_MAX];
int                 nCenterRC[CH_TYPE_MAX];
int                 nHighRC[CH_TYPE_MAX];
int                 nRCReverseFlag[CH_TYPE_MAX];
int                 nESCOutput[MAX_CH_ESC];
int                 nAxisReverseFlag[3];
byte                nRcvChFlag[CH_TYPE_MAX];
unsigned long       nRcvChHighTime[CH_TYPE_MAX];
int                 nRcvChVal[CH_TYPE_MAX];
int                 nCompensatedRCVal[CH_TYPE_MAX];

// For Estimated Status of Drone
float               nEstimatedRPY[3];
//float               nFineGyro[3];
//float               nQuaternion[4];                         // quaternion
//float               nEstGravity[3];                         // estimated gravity direction
//bool                bIsInitializeRPY;
//float               nRPYOffset[3];

// For Control Interval
unsigned long       nESCLoopTimer;
unsigned long       nPrevSensorCapTime;
unsigned long       nCurrSensorCapTime;
unsigned long       nCurrTime;
#if __PROFILE__
unsigned long       nProfileStartTime;
unsigned long       nProfileEndTime;
#endif
double              nDiffTime;

// For Status of Drone
DroneStatus         nDroneStatus;

// For Battery Status
int                 nCurrBatteryVolt;

// For LED Control
int                 nLED_Status;
unsigned long       nPrevBlinkTime;


/*----------------------------------------------------------------------------------------
 Static Function
 ----------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------
 Static Variable
 ----------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------
 Global Variable
 ----------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------
 Function Implementation
 ----------------------------------------------------------------------------------------*/
#include "CommHeader.h"
#include "LED_Control.h"
#include "RC_Control.h"
#include "ESC_Control.h"
#include "MPU6050_Control.h"
//#include "HMC5883L_Control.h"
//#include "HS5611_Control.h"
//#include "SR04_Control.h"
#include "Misc.h"
#include "AHRS_Control.h"
#include "PID_Control.h"
#include "ExtComm_Control.h"
#include "Debugger.h"


void setup()
{
    int                 i;

    Serialprintln(F(" . ")); Serialprintln(F(" . ")); Serialprintln(F(" . ")); Serialprintln(F(" . "));
    Serialprintln(F("   **********************************************   "));
    Serialprintln(F("   **********************************************   "));

    // Set I2C Enable
    Wire.begin();
    
    #if __DEBUG__
    Serial.begin(SERIAL_BAUDRATE);
    Serial.flush();
    #endif

    // Read EEPROM Data
    _Read_EEPROM();

    // Initialize LED
    _LED_Initialize();
      
    // Initialize ESCs
    _ESC_Initialize();

    // Initialize RemoteController
    _RC_Initialize();

    // Initialize Gyro_Accel
    _AccelGyro_Initialize();

    // Initialize Magnetic
    //_Mag_Initialize();

    // Initialize Barometer
    //_Barometer_Initialize();

    // Initialize Sonar Sensor
    //_Sonar_Initialize();

    nESCLoopTimer = 0;
    nDroneStatus = DRONESTATUS_STOP;
    nCurrBatteryVolt = (analogRead(PIN_CHECK_POWER_STAT) + 65) * 1.2317;
    nLED_Status = 0;

    Serialprintln(F("   **********************************************   "));
    Serialprintln(F("   **********************************************   "));
    Serialprintln(F("   ")); Serialprintln(F("   ")); Serialprintln(F("   ")); Serialprintln(F("   "));
}


void loop()
{
    int                     i = 0;

    // Get Receiver Input
    // Then Mapping Actual Reciever Value to 1000 ~ 2000
    for(i=0 ; i<=CH_TYPE_TAKE_LAND ; i++)
        _RC_Compensate(i);

    // Get Sensor (Gyro / Accel / Megnetic / Baro / Temp)
    _GetRawSensorData();

    // Calculate Roll, Pitch, and Yaw by Quaternion
    _Get_RollPitchYaw();

    // Check Battery Voltage Status
    _Check_BatteryVolt();

    // Check Drone Status
    _Check_Drone_Status();

    #if __EXTERNAL_READ__
    {
        if(Serial.available())
        {
            char ch = Serial.read();
            
            if(ch == 'a')
            {
                nPIDGainTable[0][0] -= 0.01;
                nPIDGainTable[1][0] -= 0.01;
            }
            else if(ch == 'z')
            {
                nPIDGainTable[0][0] += 0.01;
                nPIDGainTable[1][0] += 0.01;
            }
            else if(ch == 's')
            {
                nPIDGainTable[0][1] -= 0.01;
                nPIDGainTable[1][1] -= 0.01;
            }
            else if(ch == 'x')
            {
                nPIDGainTable[0][1] += 0.01;
                nPIDGainTable[1][1] += 0.01;
            }
            else if(ch == 'd')
            {
                nPIDGainTable[0][2] -= 0.01;
                nPIDGainTable[1][2] -= 0.01;
            }
            else if(ch == 'c')
            {
                nPIDGainTable[0][2] += 0.01;
                nPIDGainTable[1][2] += 0.01;
            }
        }
    }
    #endif

    // PID Computation
    _CalculatePID();

    // Throttle Calculation
    _CalculateThrottleVal();

    // Update BLDCs
    _UpdateESCs();

    #if __PRINT_DEBUG__ || __EXTERNAL_READ__
        _print_Data();
    #endif
}



