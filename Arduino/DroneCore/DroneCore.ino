
#define __DEBUG__                           (0)
#if (__DEBUG__)
    #define __PRINT_DEBUG__                 (1)
    #define __PROFILE__                     (0)
    #define SERIAL_BAUDRATE                 (115200)
#else
    #define __PRINT_DEBUG__                 (1)
    #define __PROFILE__                     (0)
#endif


/*----------------------------------------------------------------------------------------
 File Inclusions
 ----------------------------------------------------------------------------------------*/
#include <I2Cdev.h>
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include <Wire.h>
#endif
#include <MPU6050_6Axis_MotionApps20.h>
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
    CH_TYPE_ROLL,
    CH_TYPE_PITCH,
    CH_TYPE_THROTTLE,
    CH_TYPE_YAW,
    CH_TYPE_TAKE_LAND,
}RC_CH_Type;

typedef enum _DroneStatus
{
    DRONESTATUS_STOP,
    DRONESTATUS_READY,
    DRONESTATUS_START,
}DroneStatus;

typedef struct _AxisErrRate_T
{
    float               nAngleErr;
    float               nCurrErrRate;
    float               nPrevErrRate;
    float               nP_ErrRate;
    float               nI_ErrRate;
    float               nD_ErrRate;
    float               nBalance;
    float               nTorque;
}AxisErrRate_T;

typedef struct _AccelGyroParam_T
{
    float               nRawGyro[3];
    float               nRawAccel[3];
    float               nRawTemp;
    float               nBaseAccel[3];
    float               nFineAngle[3];                          // Filtered Angles
}AccelGyroParam_T;

typedef struct _MagParam_T
{
    float               nRawMag[3];
    float               nMagHeadingRad;
    float               nMagHeadingDeg;
    float               nSmoothHeadingDegrees;
    float               nPrevHeadingDegrees;
    float               nDeclinationAngle;
}MagneticParam_T;

typedef struct _BaroParam_T
{
    float               nRawTemp;                               // Raw Temperature Data
    float               nRawPressure;                           // Raw Pressure Data
    float               nRawAbsoluteAltitude;                   // Estimated Absolute Altitude

    float               nAvgPressure;                           // Average Pressure Data
    float               nAvgTemp;                               // Average Temperature Data
    float               nAvgAbsoluteAltitude;                   // Average Absolute Altitude Data
    float               nRelativeAltitude;                      // Relative Absolute Altitude Data
    float               nPrevAvgAbsoluteAltitude;               // Average Absolute Altitude Data
    float               nRefAbsoluteAltitude;                   // Reference Absolute Altitude Data
    float               nVerticalSpeed;                         // Estimated Vertical Speed
}BaroParam_T;

typedef struct _SonicParam_T
{
    float               nRawDist;                               // Indicate Distance Calculated From Sensor
    float               nDistFromGnd;                           // Indicate istance from Ground
}SonicParam_T;

typedef struct _SelfFly_T
{
    // For Accelerator & Gyroscope Sensor
    MPU6050             nAccelGyroHndl;                         // MPU6050 Gyroscope Interface
    AccelGyroParam_T    nAccelGyroParam;
    //int                 nCalibMean_AX, nCalibMean_AY, nCalibMean_AZ;
    //int                 nCalibMean_GX, nCalibMean_GY, nCalibMean_GZ;

    // For Magnetometer Sensor
    HMC5883L            nMagHndl;                               // HMC5883 Magnetic Interface
    MagneticParam_T     nMagParam;

    // For Barometer Sensor
    MS561101BA          nBaroHndl;                              // MS5611 Barometer Interface
    BaroParam_T         nBaroParam;

    // For Sonar Sensor
    SonicParam_T        SonicParam;                             // HC-SR04 Sonar Sensor

    // For PID Control
    #if USE_NEW_PID
    AxisErrRate_T       nRPY_PID[3];
    #else
    AxisErrRate_T       nPitch;
    AxisErrRate_T       nRoll;
    AxisErrRate_T       nYaw;
    #endif

    // For Motor Control
    long                nCapturedRCVal[MAX_CH_RC];              // RC channel inputs
    long                nUsingRCVal[MAX_CH_RC];                 // RC channel inputs
    unsigned long       nRCPrevChangeTime[MAX_CH_RC];
    unsigned long       nThrottle[MAX_CH_ESC];
    volatile bool       nLock;

    // For Estimated Status of Drone
    float               nFineRPY[3];
    float               nFineGyro[3];
    float               nQuaternion[4];                         // quaternion
    float               nEstGravity[3];                         // estimated gravity direction
    bool                bIsInitializeRPY;
    unsigned long       nInitializedTime;
    float               nRPYOffset[3];

    // For Control Interval
    unsigned long       nCurrSensorCapTime;
    unsigned long       nPrevSensorCapTime;
    float               nDiffTime;
    float               nSampleFreq;                            // half the sample period expressed in seconds
    float               nBeta;

    // For Status of Drone
    DroneStatus         nDroneStatus;

    // For Battery Status
    float               nCurrBatteryVolt;

    // For LED Control
    int                 nPrevR;
    int                 nPrevG;
    int                 nPrevB;
    unsigned long       nPrevBlinkTime;
}SelfFly_T;


/*----------------------------------------------------------------------------------------
 Static Function
 ----------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------
 Static Variable
 ----------------------------------------------------------------------------------------*/
static SelfFly_T                   *pSelfFlyHndl = NULL;                           // SelfFly Main Handle


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
#include "HMC5883L_Control.h"
#include "HS5611_Control.h"
#include "SR04_Control.h"
#include "Misc.h"
#include "AHRS_Control.h"
#include "PID_Control.h"
#include "ExtComm_Control.h"
#include "Debugger.h"


void setup()
{
    int32_t                 i = 0;
    uint8_t                 nDevStatus;                         // return status after each device operation (0 = success, !0 = error)

    Serialprintln("");    Serialprintln("");    Serialprintln("");    Serialprintln("");
    Serialprintln("   **********************************************   ");
    Serialprintln("   **********************************************   ");

    pSelfFlyHndl = (SelfFly_T *) malloc(sizeof(SelfFly_T));

    memset(pSelfFlyHndl, 0, sizeof(SelfFly_T));

    Wire.begin();
    TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)

    #if __DEBUG__
    Serial.begin(SERIAL_BAUDRATE);
    Serial.flush();

    while(!Serial); // wait for Leonardo enumeration, others continue immediately
    #endif

    // Initialize RemoteController
    _RC_Initialize();

    // Initialize ESCs
    _ESC_Initialize();

    // Initialize Gyro_Accel
    _AccelGyro_Initialize();

    // Initialize Magnetic
    _Mag_Initialize();

    // Initialize Barometer
    _Barometer_Initialize();

    // Initialize Sonar Sensor
    //_Sonar_Initialize();

    // Initialize LED
    _LED_Initialize();

    pSelfFlyHndl->nQuaternion[0] = 1.0f;
    pSelfFlyHndl->nBeta = BETADEF;
    pSelfFlyHndl->nDroneStatus = DRONESTATUS_STOP;
    pSelfFlyHndl->nCurrBatteryVolt = (float)(analogRead(PIN_CHECK_POWER_STAT) + 65) * 1.2317;
    pSelfFlyHndl->nInitializedTime = 0;
    pSelfFlyHndl->bIsInitializeRPY = 0;

    Serialprintln("   **********************************************   ");
    Serialprintln("   **********************************************   ");
    Serialprintln("");    Serialprintln("");    Serialprintln("");    Serialprintln("");
}


void loop()
{
    #if __PROFILE__
    float                   nStartTime0 = 0.0f, nEndTime = 0.0f;

    nStartTime0 = micros();
    #endif

    _LED_Blink();

    // Check Drone Status
    _Check_Drone_Status();
    if(DRONESTATUS_STOP == pSelfFlyHndl->nDroneStatus)
        return;

    // Check Battery Voltage Status
    _Check_BatteryVolt();

    {
        if(Serial.available())
        {
            char ch = Serial.read();
            
            if(ch == 'a')
                nPIDGainTable[0][0] -= 0.1;
            else if(ch == 'z')
                nPIDGainTable[0][0] += 0.1;
            else if(ch == 's')
                nPIDGainTable[0][1] -= 0.1;
            else if(ch == 'x')
                nPIDGainTable[0][1] += 0.1;
            else if(ch == 'd')
                nPIDGainTable[0][2] -= 0.1;
            else if(ch == 'c')
                nPIDGainTable[0][2] += 0.1;
        }
    }

    // Get Sensor (Gyro / Accel / Megnetic / Baro / Temp)
    _GetSensorRawData();

    // Calculate Roll, Pitch, and Yaw by Quaternion
    _Get_RollPitchYaw();

    // PID Computation
    _CalculatePID();

    // Throttle Calculation
    _CalculateThrottleVal();

    #if __PRINT_DEBUG__
    //_print_Gyro_Signals();
    //_print_MagData();
    //_print_BarometerData();
    //_print_RPY_Signals();
    //_print_CaturedRC_Signals();
    //_print_UsingRC_Signals();
    //_print_Throttle_Signals();
    //_print_SonarData();
    _print_AllData();
    Serialprintln(" ");
    #endif

    // Update BLDCs
    _UpdateESCs();

    #if __PROFILE__
    nEndTime = micros();
    Serialprint(" Loop Duration: "); Serialprintln((nEndTime - nStartTime0)/1000);
    #endif
}


