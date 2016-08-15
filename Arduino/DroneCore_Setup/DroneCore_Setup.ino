#define __DEBUG__                           (1)
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

    // Initialize RemoteController
    //_RC_Initialize();

    // Initialize Gyro_Accel
    _AccelGyro_Initialize();

    // Initialize Magnetic
    //_Mag_Initialize();

    // Initialize Barometer
    //_Barometer_Initialize();

    // Initialize Sonar Sensor
    //_Sonar_Initialize();

    Serialprintln(F("********************************************************************"));
    Serialprintln(F("********************************************************************"));
    Serialprintln(F("   ")); Serialprintln(F("   ")); Serialprintln(F("   ")); Serialprintln(F("   "));

    _gESCLoopTimer = micros();
}

unsigned long cnt = 0;
void loop()
{
    cnt++;

    _GetRawSensorData();
    
    // Get Asix of Each RC Channel
    Serialprintln(F("********************************************************************"));
    Serialprintln(F("********************************************************************"));
    Serialprintln(F("   Step 1. Get Axis Type of RC Channel"));
    Serialprintln(F("********************************************************************"));
    Serialprintln(F("********************************************************************"));
    Serialprintln(F(""));
    //_RC_CheckAxis(0);
    //_RC_CheckAxis(1);
    //_RC_CheckAxis(2);
    //_RC_CheckAxis(3);

    // Get Range of RC Signal
    Serialprintln(F("********************************************************************"));
    Serialprintln(F("********************************************************************"));
    Serialprintln(F("   Step 2. Get Range of Each RC Channel"));
    Serialprintln(F("********************************************************************"));
    Serialprintln(F("********************************************************************"));
    Serialprintln(F(""));
    //_RC_GetRCRange();

    // Calibration Gyro
    Serialprintln(F("********************************************************************"));
    Serialprintln(F("********************************************************************"));
    Serialprintln(F("   Step 3. Gyro & Accel Calibration"));
    Serialprintln(F("********************************************************************"));
    Serialprintln(F("********************************************************************"));
    Serialprintln(F(""));
    //_AccelGyro_Calibration();

    // Get Axis Type of Gyro & Accel
    Serialprintln(F("********************************************************************"));
    Serialprintln(F("********************************************************************"));
    Serialprintln(F("   Step 4. Get Axis Type of Gyro & Accel"));
    Serialprintln(F("********************************************************************"));
    Serialprintln(F("********************************************************************"));
    Serialprintln(F(""));
    _AccelGyro_CheckAxis(0);
    _AccelGyro_CheckAxis(1);
    _AccelGyro_CheckAxis(2);

    _Get_RollPitchYaw();

    if(0 == (cnt % 300))
    {
        Serialprint(F("             Gyro:"));
        Serialprint(F("  X:")); Serialprint(_gAngleRoll);
        Serialprint(F("  Y:")); Serialprint(_gAnglePitch);
        Serialprint(F("  Z:")); Serialprintln(_gAngleYaw);
    }

    while(micros() - _gESCLoopTimer < 4000);
    _gESCLoopTimer = micros();
}


















