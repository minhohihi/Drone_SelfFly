
#define __DEBUG__                           (0)
#if (__DEBUG__)
    #define __PRINT_DEBUG__                 (1)
    #define __PROFILE__                     (1)
    #define __EXTERNAL_READ__               (0)
    #define SERIAL_BAUDRATE                 (115200)
#else
    #define __PRINT_DEBUG__                 (1)
    #define __PROFILE__                     (0)
#endif


/*----------------------------------------------------------------------------------------
 File Inclusions
 ----------------------------------------------------------------------------------------*/
#include "CommHeader.h"
#include <EEPROM.h>
#include <I2Cdev.h>
#include <Wire.h>
#if USE_MPU6050_DMP
    #include <MPU6050_6Axis_MotionApps20.h>
#endif
#include <HMC5883L.h>
#include <MS561101BA.h>
#include <math.h>


/*----------------------------------------------------------------------------------------
 Constant Definitions
 ----------------------------------------------------------------------------------------*/


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

byte                _gEEPROMData[EEPROM_DATA_MAX] = {0, };

// For Accelerator & Gyroscope Sensor
double              _gRawGyro[3] = {0.0, };
long                _gRawAccel[3] = {0, };
long                _gAccTotalVector = 0.0;
float               _gAngleRollAcc = 0.0;
float               _gAnglePitchAcc = 0.0;
float               _gAngleYawAcc = 0.0;
float               _gAngleRoll = 0.0;
float               _gAnglePitch = 0.0;
float               _gAngleYaw = 0.0;
float               _gPitchLevelAdjust = 0.0;
float               _gRollLevelAdjust = 0.0;
double              _gCalibMeanGyro[3] = {0.0, };
double              _gCalibMeanAccel[3] = {0.0, };
byte                _gGyroAccelAxis[3] = {0, };

// For Magnetometer Sensor
HMC5883L            _gMagHndl;                              // HMC5883 Magnetic Interface
float               _gRawMag[3] = {0.0f, };
float               _gMagHeadingRad = 0.0;
float               _gMagHeadingDeg = 0.0;
float               _gSmoothHeadingDegrees = 0.0;
float               _gPrevHeadingDegrees = 0.0;
float               _gDeclinationAngle = 0.0;

// For Barometer Sensor
MS561101BA          _gBaroHndl;                             // MS5611 Barometer Interface
float               _gRawTemp = 0.0;                        // Raw Temperature Data
float               _gRawPressure = 0.0;                    // Raw Pressure Data
float               _gRawAbsoluteAltitude = 0.0;            // Estimated Absolute Altitude
float               _gAvgPressure = 0.0;                    // Average Pressure Data
float               _gAvgTemp = 0.0;                        // Average Temperature Data
float               _gAvgAbsoluteAltitude = 0.0;            // Average Absolute Altitude Data
float               _gRelativeAltitude = 0.0;               // Relative Absolute Altitude Data
float               _gPrevAvgAbsoluteAltitude = 0.0;        // Average Absolute Altitude Data
float               _gRefAbsoluteAltitude = 0.0;            // Reference Absolute Altitude Data
float               _gVerticalSpeed = 0.0;                  // Estimated Vertical Speed

// For Sonar Sensor
float               _gRawDist = 0.0;                        // Indicate Distance Calculated From Sensor
float               _gDistFromGnd = 0.0;                    // Indicate istance from Ground

// For PID Control
AxisErrRate_T       _gRPY_PID[3] = {0, };

// For Motor Control
int                 _gRCSignal_L[CH_TYPE_MAX] = {0, };
int                 _gRCSignal_M[CH_TYPE_MAX] = {0, };
int                 _gRCSignal_H[CH_TYPE_MAX] = {0, };
byte                _gRCReverseFlag[CH_TYPE_MAX] = {0, };
int                 _gESCOutput[MAX_CH_ESC] = {0, };
int                 _gAxisReverseFlag[3] = {0, };
byte                _gRCRisingFlag = 0;
unsigned long       _gRCChRisingTime[CH_TYPE_MAX] = {0, };
int                 _gRCSignalVal[CH_TYPE_MAX] = {0, };
byte                _gRCChAxis[CH_TYPE_MAX] = {0, };
int                 _gCompensatedRCVal[CH_TYPE_MAX] = {0, };

// For Estimated Status of Drone
float               _gEstimatedRPY[3] = {0, };
//float               _gFineGyro[3] = {0.0, };
//float               _gQuat[4] = {0.0, };                  // quaternion
//float               _gEstGravity[3] = {0.0, };            // estimated gravity direction
//bool                _gbIsInitializeRPY = 0.0;
//float               _gRPYOffset[3] = {0.0, };

// For Control Interval
unsigned long       _gESCLoopTimer = 0;
unsigned long       _gPrevSensorCapTime = 0;
unsigned long       _gCurrSensorCapTime = 0;
unsigned long       _gCurrTime = 0;
#if __PROFILE__
unsigned long       _gProfileStartTime = 0;
unsigned long       _gProfileEndTime = 0;
#endif
float               _gDiffTime = 0;

// For Status of Drone
DroneStatus         _gDroneStatus = DRONESTATUS_STOP;

// For Battery Status
int                 _gCurrBatteryVolt = 0;

// For LED Control
int                 _gLED_Status = 0;
unsigned long       _gPrevBlinkTime = 0;


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
    Serialprintln(F("********************************************************************"));
    Serialprintln(F("********************************************************************"));

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

    _gESCLoopTimer = 0;
    _gDroneStatus = DRONESTATUS_STOP;
    _gCurrBatteryVolt = (analogRead(PIN_CHECK_POWER_STAT) + 65) * 1.2317;
    _gLED_Status = 0;

    Serialprintln(F("********************************************************************"));
    Serialprintln(F("********************************************************************"));
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



