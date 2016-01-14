
#define __DEBUG__                           (0)
#if (__DEBUG__)
    #define __PRINT_DEBUG__                 (0)
    #define __PROFILE__                     (1)
    #define SERIAL_BAUDRATE                 (115200)
#else
    #define __PRINT_DEBUG__                 (1)
    #define __PROFILE__                     (0)
#endif

#define USE_DIRECTACCESS_REG                (1)


/*----------------------------------------------------------------------------------------
 File Inclusions
 ----------------------------------------------------------------------------------------*/
#include <I2Cdev.h>
#if !USE_DIRECTACCESS_REG
#include <Servo.h>
#include <PinChangeInt.h>
#endif
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
// Define Axis
#define X_AXIS                              (0)
#define Y_AXIS                              (1)
#define Z_AXIS                              (2)

// Define Max Number of Each Field
#define MAX_CH_RC                           (5)
#define MAX_CH_ESC                          (4)

// Arduino Pin configuration
#define PIN_SONAR_TRIG                      (13)
#define PIN_SONAR_ECHO                      (12)
#define PIN_ESC_CH2                         (11)
#define PIN_ESC_CH1                         (10)
#define PIN_ESC_CH3                         (9)
#define PIN_ESC_CH0                         (8)
#define PIN_RESERVED_D02                    (7)
#define PIN_RC_CH0                          (6)
#define PIN_RC_CH1                          (5)
#define PIN_RC_CH2                          (4)
#define PIN_RC_CH3                          (3)
#define PIN_RC_CH4                          (2)
#define PIN_RESERVED_D01                    (1)
#define PIN_RESERVED_D00                    (0)
#define PIN_RESERVED_A07                    (A7)
#define PIN_RESERVED_A06                    (A6)
#define PIN_RESERVED_A05                    (A5)
#define PIN_RESERVED_A04                    (A4)
#define PIN_RESERVED_A03                    (A3)
#define PIN_RESERVED_A02                    (A2)
#define PIN_RESERVED_A01                    (A1)
#define PIN_CHECK_POWER_STAT                (A0)

// ESC configuration
#define ESC_MIN                             (1000)
#define ESC_MAX                             (2000)
#define ESC_TAKEOFF_OFFSET                  (999)
#define ESC_ARM_DELAY                       (1000)

// RC configuration
#define RC_CH0_HIGH                         (1900)
#define RC_CH0_LOW                          (1050)
#define RC_CH1_HIGH                         (1900)
#define RC_CH1_LOW                          (1050)
#define RC_CH2_HIGH                         (1900)
#define RC_CH2_LOW                          (1050)
#define RC_CH3_HIGH                         (1900)
#define RC_CH3_LOW                          (1050)
#define RC_CH4_HIGH                         (1900)
#define RC_CH4_LOW                          (1050)

// PID configuration
#define PITCH_OUTER_P_GAIN                  (4.750)                         // angle control
#define PITCH_INNER_P_GAIN                  (2.933)                         // rate control
#define PITCH_INNER_I_GAIN                  (0.440)
#define PITCH_INNER_D_GAIN                  (0.335)
#define ROLL_OUTER_P_GAIN                   (4.750)                         // angle control
#define ROLL_INNER_P_GAIN                   (2.833)                         // rate control
#define ROLL_INNER_I_GAIN                   (0.440)
#define ROLL_INNER_D_GAIN                   (0.335)
#define YAW_P_GAIN                          (2.325)                         // yaw -> rate control
#define YAW_I_GAIN                          (0.650)

#define GYRO_FS_PRECISIOM                   (MPU6050_GYRO_FS_250)
#define GYRO_FS                             (131.0f)                        // (2^15 - 1) / (250 * (1 << GYRO_FS_PRECISIOM))
#define ACCEL_FS_PRECISIOM                  (MPU6050_ACCEL_FS_2)            //  MPU6050_ACCEL_FS_4  MPU6050_ACCEL_FS_8  MPU6050_ACCEL_FS_16
#define ACCEL_STD_DENOM                     (16384.0f / (1 << ACCEL_FS_PRECISIOM))

// Flight parameters
#define PITCH_ANG_MIN                       (-30)
#define PITCH_ANG_MAX                       (30)
#define ROLL_ANG_MIN                        (-30)
#define ROLL_ANG_MAX                        (30)
#define YAW_RATE_MIN                        (-20)
#define YAW_RATE_MAX                        (20)

// Offset values
#define PITCH_ANG_OFFSET                    (-4)
#define ROLL_ANG_OFFSET                     (1.6)

#define SAMPLEFREQ                          (133.0f)                        // sample frequency in Hz
#define BETADEF                             (1.1f)

// AccelGyro Offset Value
#define MPU6050_GYRO_OFFSET_X               (65)
#define MPU6050_GYRO_OFFSET_Y               (-42)
#define MPU6050_GYRO_OFFSET_Z               (-3)
#define MPU6050_ACCEL_OFFSET_X              (-73)
#define MPU6050_ACCEL_OFFSET_Y              (-737)
#define MPU6050_ACCEL_OFFSET_Z              (0)


// Sonar sensor
#define SONAR_MAX_WAIT                      (30000)                         // Unit: microsecond

#define ROUNDING_BASE                       (50)
#define SAMPLING_TIME                       (0.01)                          // Unit: Seconds

#define RAD_TO_DEG_SCALE                    (57.2958f)                      // = 180 / PI
#define DEG_TO_RAD_SCALE                    (0.0175f)                       // = PI / 180
#define SINGLE_RADIAN                       (3.141592)                      // = PI
#define DOUBLE_RADIAN                       (6.283184)                      // = 2 * PI
#define BARO_SEA_LEVEL_BASE                 (1013.25)                       // Base Sea Level

#define DRONE_STOP_TIME_TH                  (3000)                          // Unit: num of loop() count, About 30 Sec.


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

typedef struct _AxisInt16_T
{
    int16_t             XAxis;
    int16_t             YAxis;
    int16_t             ZAxis;
}AxisInt16_T;

typedef struct _AxisFloat_T
{
    float               XAxis;
    float               YAxis;
    float               ZAxis;
}AxisFloat_T;

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
    int                 nCalibMean_AX, nCalibMean_AY, nCalibMean_AZ;
    int                 nCalibMean_GX, nCalibMean_GY, nCalibMean_GZ;

    // For Magnetometer Sensor
    HMC5883L            nMagHndl;                               // HMC5883 Magnetic Interface
    MagneticParam_T     nMagParam;

    // For Barometer Sensor
    MS561101BA          nBaroHndl;                              // MS5611 Barometer Interface
    BaroParam_T         nBaroParam;

    // For Sonar Sensor
    SonicParam_T        SonicParam;                             // HC-SR04 Sonar Sensor

    // For PID Control
    AxisErrRate_T       nPitch;
    AxisErrRate_T       nRoll;
    AxisErrRate_T       nYaw;

    // For Motor Control
    #if !USE_DIRECTACCESS_REG
    Servo               *pESC[MAX_CH_ESC];
    #endif
    float               nRCCh[MAX_CH_RC];                       // RC channel inputs
    unsigned long       nRCPrevChangeTime[MAX_CH_RC];
    unsigned long       nThrottle[MAX_CH_ESC];

    // For Estimated Status of Drone
    float               nFineRPY[3];
    float               nFineGyro[3];
    float               nQuaternion[4];                         // quaternion
    float               nEstGravity[3];                         // estimated gravity direction

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
}SelfFly_T;


/*----------------------------------------------------------------------------------------
 Static Function
 ----------------------------------------------------------------------------------------*/
void _Check_Drone_Status();
void _ESC_Initialize();
void _RC_Initialize();
void _AccelGyro_Initialize();
void _AccelGyro_GetData();
void _AccelGyro_GetMeanSensor();
void _Mag_Initialize();
void _Mag_GetData();
void _Mag_CalculateDirection();
void _Barometer_Initialize();
void _Barometer_GetData();
void _Barometer_CalculateData();
void _Sonar_Initialize();
void _Sonar_GetData();
void _GetSensorRawData();
void _Check_BatteryVolt();
void _Get_RollPitchYaw();
void _AHRSupdate();
inline void _CalculatePID();
inline void _CalculateThrottleVal();
inline void _UpdateESCs();
#if !USE_DIRECTACCESS_REG
inline void _nRCInterrupt_CB0();
inline void _nRCInterrupt_CB1();
inline void _nRCInterrupt_CB2();
inline void _nRCInterrupt_CB3();
inline void _nRCInterrupt_CB4();
#endif
float _Clip3Float(const float nValue, const int MIN, const int MAX);
int _Clip3Int(const int nValue, const int MIN, const int MAX);
float _InvSqrt(float nNumber);
#if __PRINT_DEBUG__
void _print_RC_Signals();
void _print_Gyro_Signals();
void _print_Throttle_Signals();
void _print_RPY_Signals();
void _print_MagData();
void _print_BarometerData();
void _print_SonarData();
#endif


/*----------------------------------------------------------------------------------------
 Static Variable
 ----------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------
 Global Variable
 ----------------------------------------------------------------------------------------*/
SelfFly_T               nSelfFlyHndl = {0, };                           // SelfFly Main Handle


/*----------------------------------------------------------------------------------------
 Function Implementation
 ----------------------------------------------------------------------------------------*/
void setup()
{
    int32_t                 i = 0;
    uint8_t                 nDevStatus;                         // return status after each device operation (0 = success, !0 = error)

    Serialprintln("");    Serialprintln("");    Serialprintln("");    Serialprintln("");
    Serialprintln("   **********************************************   ");
    Serialprintln("   **********************************************   ");

    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
    #endif

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
    _Sonar_Initialize();

    nSelfFlyHndl.nQuaternion[0] = 1.0f;
    nSelfFlyHndl.nBeta = BETADEF;
    nSelfFlyHndl.nDroneStatus = DRONESTATUS_STOP;
    nSelfFlyHndl.nCurrBatteryVolt = (float)(analogRead(PIN_CHECK_POWER_STAT) + 65) * 1.2317;

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

    // Check Drone Status
    //_Check_Drone_Status();
    //if(DRONESTATUS_STOP == nSelfFlyHndl.nDroneStatus)
    //    return;

    // Check Battery Voltage Status
    _Check_BatteryVolt();
    
    // Get Sensor (Gyro / Accel / Megnetic / Baro / Temp)
    _GetSensorRawData();

    // Calculate Roll, Pitch, and Yaw by Quaternion
    _Get_RollPitchYaw();

    // PID Computation
    _CalculatePID();

    // Throttle Calculation
    _CalculateThrottleVal();

    // Update BLDCs
    _UpdateESCs();

    //delay(4);

    #if __PRINT_DEBUG__
    //_print_Gyro_Signals();
    //_print_MagData();
    //_print_BarometerData();
    //_print_RPY_Signals();
    //_print_RC_Signals();
    //_print_Throttle_Signals();
    //_print_SonarData();
    #endif

    #if __PROFILE__
    nEndTime = micros();
    Serialprint(" Loop Duration: "); Serialprintln((nEndTime - nStartTime0)/1000);
    #endif
}


void _Check_Drone_Status()
{
    if(DRONESTATUS_STOP == nSelfFlyHndl.nDroneStatus)
    {
        int                 nLoopCnt = 0;

        do
        {
            // Should be Set Lowest Throttle & Highest Right Yaw for About 2 Sec. to Start Drone
            if((1100 >= nSelfFlyHndl.nRCCh[CH_TYPE_THROTTLE]) &&
               (1800 < nSelfFlyHndl.nRCCh[CH_TYPE_YAW]))
                nLoopCnt++;
            else
                nLoopCnt = 0;

            delay(50);
        }while(nLoopCnt < 40);

        nSelfFlyHndl.nDroneStatus = DRONESTATUS_READY;

        return;
    }
    else if(DRONESTATUS_STOP < nSelfFlyHndl.nDroneStatus)
    {
        static int      nLoopCnt = 0;

        if(DRONESTATUS_READY == nSelfFlyHndl.nDroneStatus)
        {
            if(1100 >= nSelfFlyHndl.nRCCh[CH_TYPE_THROTTLE])
                nLoopCnt++;
            else
            {
                nLoopCnt = 0;
                nSelfFlyHndl.nDroneStatus = DRONESTATUS_START;
            }

            if(nLoopCnt > DRONE_STOP_TIME_TH)
            {
                nLoopCnt = 0;
                nSelfFlyHndl.nDroneStatus = DRONESTATUS_STOP;
            }
        }
        else
        {
            if(1100 >= nSelfFlyHndl.nRCCh[CH_TYPE_THROTTLE])
            {
                nLoopCnt = 0;
                nSelfFlyHndl.nDroneStatus = DRONESTATUS_READY;
            }
        }
    }
}


void _ESC_Initialize()
{
    int                     i = 0;

    #if !USE_DIRECTACCESS_REG
    for(i=0 ; i<MAX_CH_ESC ; i++)
        nSelfFlyHndl.pESC[i] = new Servo;

    nSelfFlyHndl.pESC[0]->attach(PIN_ESC_CH0, ESC_MIN, ESC_MAX);
    nSelfFlyHndl.pESC[1]->attach(PIN_ESC_CH1, ESC_MIN, ESC_MAX);
    nSelfFlyHndl.pESC[2]->attach(PIN_ESC_CH2, ESC_MIN, ESC_MAX);
    nSelfFlyHndl.pESC[3]->attach(PIN_ESC_CH3, ESC_MIN, ESC_MAX);

    delay(300);

    for(i=0 ; i<MAX_CH_ESC ; i++)
        nSelfFlyHndl.pESC[i]->writeMicroseconds(0);

    delay(ESC_ARM_DELAY);
    #else
    // Set Digital Port 8, 9, 10, and 11 as Output
    DDRB |= B00011110;
    
    delay(100);
    
    // Set Value of Digital Port 8, 9, 10, and 11 as Low to Initialize ESCs
    PORTB |= B00000000;
    #endif
}


void _RC_Initialize()
{
    #if !USE_DIRECTACCESS_REG
    PCintPort::attachInterrupt(PIN_RC_CH0, _nRCInterrupt_CB0, CHANGE);
    PCintPort::attachInterrupt(PIN_RC_CH1, _nRCInterrupt_CB1, CHANGE);
    PCintPort::attachInterrupt(PIN_RC_CH2, _nRCInterrupt_CB2, CHANGE);
    PCintPort::attachInterrupt(PIN_RC_CH3, _nRCInterrupt_CB3, CHANGE);
    PCintPort::attachInterrupt(PIN_RC_CH4, _nRCInterrupt_CB4, CHANGE);
    #else
    PCICR |= (1 << PCIE2);
    PCMSK2 |= (1 << PCINT18);
    PCMSK2 |= (1 << PCINT19);
    PCMSK2 |= (1 << PCINT20);
    PCMSK2 |= (1 << PCINT21);
    PCMSK2 |= (1 << PCINT22);
    #endif
}


void _AccelGyro_Initialize()
{
    nSelfFlyHndl.nAccelGyroHndl = MPU6050();

    Serialprintln(F(" Initializing MPU..."));
    nSelfFlyHndl.nAccelGyroHndl.initialize();

    // Verify Vonnection
    Serialprint(F("    Testing device connections..."));
    Serialprintln(nSelfFlyHndl.nAccelGyroHndl.testConnection() ? F("  MPU6050 connection successful") : F("  MPU6050 connection failed"));

    nSelfFlyHndl.nAccelGyroHndl.setI2CMasterModeEnabled(false);
    nSelfFlyHndl.nAccelGyroHndl.setI2CBypassEnabled(true);
    nSelfFlyHndl.nAccelGyroHndl.setSleepEnabled(false);

    // Calibrate GyroAccel
    //nSelfFlyHndl.nAccelGyroHndl.doCalibration();
    nSelfFlyHndl.nAccelGyroHndl.setXGyroOffset(MPU6050_GYRO_OFFSET_X);
    nSelfFlyHndl.nAccelGyroHndl.setYGyroOffset(MPU6050_GYRO_OFFSET_Y);
    nSelfFlyHndl.nAccelGyroHndl.setZGyroOffset(MPU6050_GYRO_OFFSET_Z);
    nSelfFlyHndl.nAccelGyroHndl.setXAccelOffset(MPU6050_ACCEL_OFFSET_X);
    nSelfFlyHndl.nAccelGyroHndl.setYAccelOffset(MPU6050_ACCEL_OFFSET_Y);
    nSelfFlyHndl.nAccelGyroHndl.setZAccelOffset(MPU6050_ACCEL_OFFSET_Z);

    // supply your own gyro offsets here, scaled for min sensitivity
    nSelfFlyHndl.nAccelGyroHndl.setRate(1);                                            // Sample Rate (500Hz = 1Hz Gyro SR / 1+1)
    nSelfFlyHndl.nAccelGyroHndl.setDLPFMode(MPU6050_DLPF_BW_20);                       // Low Pass filter 20hz
    nSelfFlyHndl.nAccelGyroHndl.setFullScaleGyroRange(GYRO_FS_PRECISIOM);              // 250? / s (MPU6050_GYRO_FS_250)
    nSelfFlyHndl.nAccelGyroHndl.setFullScaleAccelRange(ACCEL_FS_PRECISIOM);            // +-2g (MPU6050_ACCEL_FS_2)

    Serialprintln(F(" MPU Initialized!!!"));

    return;
}


void _AccelGyro_GetData()
{
    AccelGyroParam_T        *pAccelGyroParam = &(nSelfFlyHndl.nAccelGyroParam);
    float                   *pRawGyro = &(pAccelGyroParam->nRawGyro[X_AXIS]);
    float                   *pRawAccel = &(pAccelGyroParam->nRawAccel[X_AXIS]);

    // Read Gyro and Accelerate Data
    Wire.beginTransmission(MPU6050_ADDRESS_AD0_LOW);
    Wire.write(MPU6050_RA_ACCEL_XOUT_H);  // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDRESS_AD0_LOW, 14, true);  // request a total of 14 registers

    pRawAccel[X_AXIS] = (float)(Wire.read()<<8 | Wire.read());               // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
    pRawAccel[Y_AXIS] = (float)(Wire.read()<<8 | Wire.read());               // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    pRawAccel[Z_AXIS] = (float)(Wire.read()<<8 | Wire.read());               // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    pAccelGyroParam->nRawTemp = (float)(Wire.read()<<8 | Wire.read());       // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)        340 per degrees Celsius, -512 at 35 degrees
    pRawGyro[X_AXIS]= (float)(Wire.read()<<8 | Wire.read());                 // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    pRawGyro[Y_AXIS]= (float)(Wire.read()<<8 | Wire.read());                 // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    pRawGyro[Z_AXIS]= (float)(Wire.read()<<8 | Wire.read());                 // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
}


void _Mag_Initialize()
{
    HMC5883L            *pMagHndl = NULL;
    
    nSelfFlyHndl.nMagHndl = HMC5883L();
    pMagHndl = &(nSelfFlyHndl.nMagHndl);
    
    // initialize Magnetic
    Serialprintln(F(" Initializing Magnetic..."));
    pMagHndl->initialize();

    // Verify Vonnection
    Serialprint(F("    Testing device connections..."));
    Serialprintln(pMagHndl->testConnection() ? F("  HMC5883L connection successful") : F("  HMC5883L connection failed"));

    // Calibrate Magnetic
    //Serialprint(F("    Start Calibration of Magnetic Sensor (HMC5883L) "));
    //pMagHndl->calibrate();
    //pMagHndl->calibration_offset(1);
    //Serialprintln(F("Done"));

    pMagHndl->setMode(HMC5883L_MODE_CONTINUOUS);
    pMagHndl->setGain(HMC5883L_GAIN_1090);
    pMagHndl->setDataRate(HMC5883L_RATE_75);
    pMagHndl->setSampleAveraging(HMC5883L_AVERAGING_8);

    // Date: 2015-11-05
    // Location: Seoul, South Korea
    // Latitude: 37.0000° North
    // Longitude: 126.0000° East
    // Magnetic declination: 7° 59.76' West
    // Annual Change (minutes/year): 3.9 '/y West
    // http://www.geomag.nrcan.gc.ca/calc/mdcal-en.php
    // http://www.magnetic-declination.com/
    nSelfFlyHndl.nMagParam.nDeclinationAngle = (7.0 + (59.76 / 60.0)) * DEG_TO_RAD_SCALE;

    Serialprintln(F(" Done"));

    // Reference WebSite
    // http://www.meccanismocomplesso.org/en/arduino-magnetic-magnetic-magnetometer-hmc5883l/
}


void _Mag_GetData()
{
    float                   *pRawMag = &(nSelfFlyHndl.nMagParam.nRawMag[X_AXIS]);

    nSelfFlyHndl.nMagHndl.getScaledHeading(&(pRawMag[X_AXIS]), &(pRawMag[Y_AXIS]), &(pRawMag[Z_AXIS]));

    // Calculate Heading
    _Mag_CalculateDirection();
}


void _Mag_CalculateDirection()
{
    int                     i = 0;
    MagneticParam_T         *pMagParam = &(nSelfFlyHndl.nMagParam);

    pMagParam->nMagHeadingRad = atan2(pMagParam->nRawMag[Y_AXIS], pMagParam->nRawMag[X_AXIS]);
    pMagParam->nMagHeadingRad -= pMagParam->nDeclinationAngle;      // If East, then Change Operation to PLUS

    if(pMagParam->nMagHeadingRad < 0)
        pMagParam->nMagHeadingRad += DOUBLE_RADIAN;

    if(pMagParam->nMagHeadingRad > DOUBLE_RADIAN)
        pMagParam->nMagHeadingRad -= DOUBLE_RADIAN;

    pMagParam->nMagHeadingDeg = pMagParam->nMagHeadingRad * RAD_TO_DEG_SCALE;

    if(pMagParam->nMagHeadingDeg >= 1 && pMagParam->nMagHeadingDeg < 240)
        pMagParam->nMagHeadingDeg = map(pMagParam->nMagHeadingDeg, 0, 239, 0, 179);
    else if(pMagParam->nMagHeadingDeg >= 240)
        pMagParam->nMagHeadingDeg = map(pMagParam->nMagHeadingDeg, 240, 360, 180, 360);

    // Smooth angles rotation for +/- 3deg
    pMagParam->nSmoothHeadingDegrees = round(pMagParam->nMagHeadingDeg);

    if((pMagParam->nSmoothHeadingDegrees < (pMagParam->nPrevHeadingDegrees + 3)) &&
       (pMagParam->nSmoothHeadingDegrees > (pMagParam->nPrevHeadingDegrees - 3)))
        pMagParam->nSmoothHeadingDegrees = pMagParam->nPrevHeadingDegrees;

    pMagParam->nPrevHeadingDegrees = pMagParam->nSmoothHeadingDegrees;
}


void _Barometer_Initialize()
{
    int                     i = 0;
    BaroParam_T             *pBaroParam = &(nSelfFlyHndl.nBaroParam);
    MS561101BA              *pBaroHndl = &(nSelfFlyHndl.nBaroHndl);

    Serialprint(F(" Initializing Barometer Sensor (MS5611)..."));

    pBaroHndl->init(MS561101BA_ADDR_CSB_LOW);

    for(i=0 ; i<50 ; i++)
    {
        _Barometer_GetData();
        delay(20);
    }

    // Get Average Pressure & Temperature
    pBaroParam->nAvgTemp = pBaroHndl->getAvgTemp();
    pBaroParam->nAvgPressure = pBaroHndl->getAvgPressure();

    // Get Reference Altitude
    pBaroParam->nRefAbsoluteAltitude = pBaroHndl->getAltitude(pBaroParam->nAvgPressure, pBaroParam->nAvgTemp);

    Serialprintln(F(" Done"));
}


void _Barometer_GetData()
{
    BaroParam_T             *pBaroParam = &(nSelfFlyHndl.nBaroParam);
    MS561101BA              *pBaroHndl = &(nSelfFlyHndl.nBaroHndl);

    pBaroParam->nRawTemp = pBaroHndl->getTemperature(MS561101BA_OSR_512);
    pBaroParam->nRawPressure = pBaroHndl->getPressure(MS561101BA_OSR_512);

    // Push to Array to Get Average Pressure & Temperature
    pBaroHndl->pushTemp(pBaroParam->nRawTemp);
    pBaroHndl->pushPressure(pBaroParam->nRawPressure);
    
    // Calculate Altitude
    _Barometer_CalculateData();
}


void _Barometer_CalculateData()
{
    BaroParam_T             *pBaroParam = &(nSelfFlyHndl.nBaroParam);
    MS561101BA              *pBaroHndl = &(nSelfFlyHndl.nBaroHndl);

    // Get Average Pressure & Temperature
    pBaroParam->nAvgTemp = pBaroHndl->getAvgTemp();
    pBaroParam->nAvgPressure = pBaroHndl->getAvgPressure();

    // Get Altitude
    pBaroParam->nRawAbsoluteAltitude = pBaroHndl->getAltitude(pBaroParam->nAvgPressure, pBaroParam->nAvgTemp);

    // Push to Array to Get Average Altitude
    pBaroHndl->pushAltitude(pBaroParam->nRawAbsoluteAltitude);

    // Get Average Pressure & Temperature
    pBaroParam->nAvgAbsoluteAltitude = pBaroHndl->getAvgAltitude();

    // Get Vertical Speed
    pBaroParam->nVerticalSpeed = abs(pBaroParam->nAvgAbsoluteAltitude - pBaroParam->nPrevAvgAbsoluteAltitude) / (nSelfFlyHndl.nDiffTime);
    pBaroParam->nRelativeAltitude = pBaroParam->nAvgAbsoluteAltitude - pBaroParam->nRefAbsoluteAltitude;

    pBaroParam->nPrevAvgAbsoluteAltitude = pBaroParam->nAvgAbsoluteAltitude;
}


void _Sonar_Initialize()
{
    int                     i = 0;

    // Set PinMode for Sonar Sensor (HC-SR04)
    pinMode(PIN_SONAR_TRIG, OUTPUT);
    pinMode(PIN_SONAR_ECHO, INPUT);

    // Calibrate Sonar Sensor
    for(i=0 ; i<50 ; i++)
    {
        _Sonar_GetData();
        delay(20);
    }
}


void _Sonar_GetData()
{
    digitalWrite(PIN_SONAR_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(PIN_SONAR_TRIG, LOW);

    // Get Raw Distance Value
    nSelfFlyHndl.SonicParam.nRawDist = pulseIn(PIN_SONAR_ECHO, HIGH, SONAR_MAX_WAIT);

    // Calculate Distance From Ground
    nSelfFlyHndl.SonicParam.nDistFromGnd = nSelfFlyHndl.SonicParam.nRawDist * 0.017; // (340(m/s) * 1000(mm) / 1000000(microsec) / 2(oneway))
}


void _GetSensorRawData()
{
    nSelfFlyHndl.nPrevSensorCapTime = nSelfFlyHndl.nCurrSensorCapTime;
    nSelfFlyHndl.nCurrSensorCapTime = micros();

    nSelfFlyHndl.nDiffTime = (nSelfFlyHndl.nCurrSensorCapTime - nSelfFlyHndl.nPrevSensorCapTime) / 1000000.0;
    nSelfFlyHndl.nSampleFreq = 1000000.0 / ((nSelfFlyHndl.nCurrSensorCapTime - nSelfFlyHndl.nPrevSensorCapTime));

    // Get AccelGyro Raw Data
    _AccelGyro_GetData();

    // Get Magnetic Raw Data
    _Mag_GetData();

    // Get Barometer Raw Data
    _Barometer_GetData();

    // Get Sonar Raw Data
    _Sonar_GetData();
}


void _Check_BatteryVolt()
{
    nSelfFlyHndl.nCurrBatteryVolt = (0.92 * nSelfFlyHndl.nCurrBatteryVolt) + (float)(analogRead(PIN_CHECK_POWER_STAT) + 65) * 0.09853;

    //Serialprintln(nSelfFlyHndl.nCurrBatteryVolt);
}


void _Get_RollPitchYaw()
{
    float           *pEstGravity = &(nSelfFlyHndl.nEstGravity[X_AXIS]);
    float           *pQ = &(nSelfFlyHndl.nQuaternion[0]);
    float           *pFineRPY = &(nSelfFlyHndl.nFineRPY[0]);

    // Calculate Roll & Pitch & Yaw
    _AHRSupdate();
    
    {
        const float nSquareQ0 = pQ[0] * pQ[0];
        const float nSquareQ1 = pQ[1] * pQ[1];
        const float nSquareGravZ = pEstGravity[Z_AXIS] * pEstGravity[Z_AXIS];
        
        // Estimate Gravity
        pEstGravity[X_AXIS] = 2 * ((pQ[1] * pQ[3]) - (pQ[0] * pQ[2]));
        pEstGravity[Y_AXIS] = 2 * ((pQ[0] * pQ[1]) + (pQ[2] * pQ[3]));
        pEstGravity[Z_AXIS] = (nSquareQ0) - (nSquareQ1) - (pQ[2] * pQ[2]) + (pQ[3] * pQ[3]);
        
        // Calculate Roll, Pitch, and Yaw
        pFineRPY[0] = atan(pEstGravity[Y_AXIS] / sqrt((pEstGravity[X_AXIS] * pEstGravity[X_AXIS]) + (nSquareGravZ)));
        pFineRPY[1] = atan(pEstGravity[X_AXIS] / sqrt((pEstGravity[Y_AXIS] * pEstGravity[Y_AXIS]) + (nSquareGravZ)));
        pFineRPY[2] = atan2((2 * pQ[1] * pQ[2]) - (2 * pQ[0] * pQ[3]), (2 * nSquareQ0) + (2 * nSquareQ1) - 1);
    }

    // Convert Radian to Degree
    pFineRPY[0] *= RAD_TO_DEG_SCALE;
    pFineRPY[1] *= RAD_TO_DEG_SCALE;
    pFineRPY[2] *= RAD_TO_DEG_SCALE;
}


// Reference Site
// http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/
void _AHRSupdate()
{
    float                   recipNorm;
    float                   s0, s1, s2, s3;
    float                   qDot1, qDot2, qDot3, qDot4;
    float                   hx, hy;
    float                   _2q0mx, _2q0my, _2q0mz, _2q1mx;
    float                   _2bx, _2bz, _4bx, _4bz;
    float                   _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3;
    float                   q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
    const float             nDiffTime = nSelfFlyHndl.nDiffTime;
    float                   *pQ = &(nSelfFlyHndl.nQuaternion[0]);
    float                   *pRawGyro = &(nSelfFlyHndl.nAccelGyroParam.nRawGyro[X_AXIS]);
    float                   *pRawAccel = &(nSelfFlyHndl.nAccelGyroParam.nRawAccel[X_AXIS]);
    float                   *pRawMag = &(nSelfFlyHndl.nMagParam.nRawMag[X_AXIS]);
    const float             nGyroOffset = DEG_TO_RAD_SCALE / GYRO_FS;
    
    pRawGyro[X_AXIS] = pRawGyro[X_AXIS] * nGyroOffset;
    pRawGyro[Y_AXIS] = pRawGyro[Y_AXIS] * nGyroOffset;
    pRawGyro[Z_AXIS] = pRawGyro[Z_AXIS] * nGyroOffset;

    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-pQ[1] * pRawGyro[X_AXIS] - pQ[2] * pRawGyro[Y_AXIS] - pQ[3] * pRawGyro[Z_AXIS]);
    qDot2 = 0.5f * ( pQ[0] * pRawGyro[X_AXIS] + pQ[2] * pRawGyro[Z_AXIS] - pQ[3] * pRawGyro[Y_AXIS]);
    qDot3 = 0.5f * ( pQ[0] * pRawGyro[Y_AXIS] - pQ[1] * pRawGyro[Z_AXIS] + pQ[3] * pRawGyro[X_AXIS]);
    qDot4 = 0.5f * ( pQ[0] * pRawGyro[Z_AXIS] + pQ[1] * pRawGyro[Y_AXIS] - pQ[2] * pRawGyro[X_AXIS]);

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if(!((pRawAccel[X_AXIS] == 0.0f) && (pRawAccel[Y_AXIS] == 0.0f) && (pRawAccel[Z_AXIS] == 0.0f)))
    {
        // Normalise accelerometer measurement
        recipNorm = _InvSqrt(pRawAccel[X_AXIS] * pRawAccel[X_AXIS] + pRawAccel[Y_AXIS] * pRawAccel[Y_AXIS] + pRawAccel[Z_AXIS] * pRawAccel[Z_AXIS]);
        pRawAccel[X_AXIS] *= recipNorm;
        pRawAccel[Y_AXIS] *= recipNorm;
        pRawAccel[Z_AXIS] *= recipNorm;

        // Normalise magnetometer measurement
        recipNorm = _InvSqrt(pRawMag[X_AXIS] * pRawMag[X_AXIS] + pRawMag[Y_AXIS] * pRawMag[Y_AXIS] + pRawMag[Z_AXIS] * pRawMag[Z_AXIS]);
        pRawMag[X_AXIS] *= recipNorm;
        pRawMag[Y_AXIS] *= recipNorm;
        pRawMag[Z_AXIS] *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        _2q0mx = 2.0f * pQ[0] * pRawMag[X_AXIS];
        _2q0my = 2.0f * pQ[0] * pRawMag[Y_AXIS];
        _2q0mz = 2.0f * pQ[0] * pRawMag[Z_AXIS];
        _2q1mx = 2.0f * pQ[1] * pRawMag[X_AXIS];
        _2q0 = 2.0f * pQ[0];
        _2q1 = 2.0f * pQ[1];
        _2q2 = 2.0f * pQ[2];
        _2q3 = 2.0f * pQ[3];
        _2q0q2 = 2.0f * pQ[0] * pQ[2];
        _2q2q3 = 2.0f * pQ[2] * pQ[3];
        q0q0 = pQ[0] * pQ[0];
        q0q1 = pQ[0] * pQ[1];
        q0q2 = pQ[0] * pQ[2];
        q0q3 = pQ[0] * pQ[3];
        q1q1 = pQ[1] * pQ[1];
        q1q2 = pQ[1] * pQ[2];
        q1q3 = pQ[1] * pQ[3];
        q2q2 = pQ[2] * pQ[2];
        q2q3 = pQ[2] * pQ[3];
        q3q3 = pQ[3] * pQ[3];

        // Reference direction of Earth's magnetic field
        hx = pRawMag[X_AXIS] * q0q0 - _2q0my * pQ[3] + _2q0mz * pQ[2] + pRawMag[X_AXIS] * q1q1
                    + _2q1 * pRawMag[Y_AXIS] * pQ[2] + _2q1 * pRawMag[Z_AXIS] * pQ[3]
                    - pRawMag[X_AXIS] * q2q2 - pRawMag[X_AXIS] * q3q3;
        hy = _2q0mx * pQ[3] + pRawMag[Y_AXIS] * q0q0 - _2q0mz * pQ[1] + _2q1mx * pQ[2]
                    - pRawMag[Y_AXIS] * q1q1 + pRawMag[Y_AXIS] * q2q2 + _2q2 * pRawMag[Z_AXIS] * pQ[3]
                    - pRawMag[Y_AXIS] * q3q3;

        _2bx = sqrt(hx * hx + hy * hy);
        _2bz = -_2q0mx * pQ[2] + _2q0my * pQ[1] + pRawMag[Z_AXIS] * q0q0 + _2q1mx * pQ[3]
                    - pRawMag[Z_AXIS] * q1q1 + _2q2 * pRawMag[Y_AXIS] * pQ[3]
                    - pRawMag[Z_AXIS] * q2q2 + pRawMag[Z_AXIS] * q3q3;

        _4bx = 2.0f * _2bx;
        _4bz = 2.0f * _2bz;

        // Gradient decent algorithm corrective step
        s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - pRawAccel[X_AXIS]) + _2q1 * (2.0f * q0q1 + _2q2q3 - pRawAccel[Y_AXIS])
                    - _2bz * pQ[2] * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - pRawMag[X_AXIS])
                    + (-_2bx * pQ[3] + _2bz * pQ[1]) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - pRawMag[Y_AXIS])
                    + _2bx * pQ[2] * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - pRawMag[Z_AXIS]);

        s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - pRawAccel[X_AXIS]) + _2q0 * (2.0f * q0q1 + _2q2q3 - pRawAccel[Y_AXIS])
                    - 4.0f * pQ[1] * (1 - 2.0f * q1q1 - 2.0f * q2q2 - pRawAccel[Z_AXIS]) + _2bz * pQ[3] * (_2bx * (0.5f - q2q2 - q3q3)
                    + _2bz * (q1q3 - q0q2) - pRawMag[X_AXIS]) + (_2bx * pQ[2] + _2bz * pQ[0]) * (_2bx * (q1q2 - q0q3)
                    + _2bz * (q0q1 + q2q3) - pRawMag[Y_AXIS]) + (_2bx * pQ[3] - _4bz * pQ[1]) * (_2bx * (q0q2 + q1q3)
                    + _2bz * (0.5f - q1q1 - q2q2) - pRawMag[Z_AXIS]);

        s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - pRawAccel[X_AXIS]) + _2q3 * (2.0f * q0q1 + _2q2q3 - pRawAccel[Y_AXIS])
                    - 4.0f * pQ[2] * (1 - 2.0f * q1q1 - 2.0f * q2q2 - pRawAccel[Z_AXIS]) + (-_4bx * pQ[2] - _2bz * pQ[0]) * (_2bx * (0.5f - q2q2 - q3q3)
                    + _2bz * (q1q3 - q0q2) - pRawMag[X_AXIS]) + (_2bx * pQ[1] + _2bz * pQ[3]) * (_2bx * (q1q2 - q0q3)
                    + _2bz * (q0q1 + q2q3) - pRawMag[Y_AXIS]) + (_2bx * pQ[0] - _4bz * pQ[2]) * (_2bx * (q0q2 + q1q3)
                    + _2bz * (0.5f - q1q1 - q2q2) - pRawMag[Z_AXIS]);

        s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - pRawAccel[X_AXIS]) + _2q2 * (2.0f * q0q1 + _2q2q3 - pRawAccel[Y_AXIS])
                    + (-_4bx * pQ[3] + _2bz * pQ[1]) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - pRawMag[X_AXIS])
                    + (-_2bx * pQ[0] + _2bz * pQ[2]) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - pRawMag[Y_AXIS])
                    + _2bx * pQ[1] * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - pRawMag[Z_AXIS]);

        recipNorm = _InvSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        // Apply feedback step
        qDot1 -= nSelfFlyHndl.nBeta * s0;
        qDot2 -= nSelfFlyHndl.nBeta * s1;
        qDot3 -= nSelfFlyHndl.nBeta * s2;
        qDot4 -= nSelfFlyHndl.nBeta * s3;
    }

    // Integrate rate of change of quaternion to yield quaternion
    pQ[0] += qDot1 * nDiffTime;
    pQ[1] += qDot2 * nDiffTime;
    pQ[2] += qDot3 * nDiffTime;
    pQ[3] += qDot4 * nDiffTime;

    // Normalise quaternion
    recipNorm = _InvSqrt(pQ[0] * pQ[0] + pQ[1] * pQ[1] + pQ[2] * pQ[2] + pQ[3] * pQ[3]);
    pQ[0] *= recipNorm;
    pQ[1] *= recipNorm;
    pQ[2] *= recipNorm;
    pQ[3] *= recipNorm;
}


inline void _CalculatePID()
{
    static float            nRCPrevCh[5] = {0, };              // Filter variables
    static float            nPrevFineRPY[3];
    AxisErrRate_T           *pPitch = &(nSelfFlyHndl.nPitch);
    AxisErrRate_T           *pRoll = &(nSelfFlyHndl.nRoll);
    AxisErrRate_T           *pYaw = &(nSelfFlyHndl.nYaw);
    float                   *pRCCh = &(nSelfFlyHndl.nRCCh[0]);
    float                   *pFineGyro = &(nSelfFlyHndl.nFineGyro[0]);
    float                   *pFineRPY = &(nSelfFlyHndl.nFineRPY[0]);
    const float             nDiffTime = nSelfFlyHndl.nDiffTime;

    pRCCh[CH_TYPE_ROLL] = floor(pRCCh[CH_TYPE_ROLL] / ROUNDING_BASE) * ROUNDING_BASE;
    pRCCh[CH_TYPE_PITCH] = floor(pRCCh[CH_TYPE_PITCH] / ROUNDING_BASE) * ROUNDING_BASE;
    pRCCh[CH_TYPE_YAW] = floor(pRCCh[CH_TYPE_YAW] / ROUNDING_BASE) * ROUNDING_BASE;

    pRCCh[CH_TYPE_ROLL] = map(pRCCh[CH_TYPE_ROLL], RC_CH0_LOW, RC_CH0_HIGH, ROLL_ANG_MIN, ROLL_ANG_MAX) - 1;
    pRCCh[CH_TYPE_PITCH] = map(pRCCh[CH_TYPE_PITCH], RC_CH1_LOW, RC_CH1_HIGH, PITCH_ANG_MIN, PITCH_ANG_MAX) - 1;
    pRCCh[CH_TYPE_YAW] = map(pRCCh[CH_TYPE_YAW], RC_CH3_LOW, RC_CH3_HIGH, YAW_RATE_MIN, YAW_RATE_MAX);

    if((pRCCh[CH_TYPE_ROLL] < ROLL_ANG_MIN) || (pRCCh[CH_TYPE_ROLL] > ROLL_ANG_MAX))
        pRCCh[CH_TYPE_ROLL] = nRCPrevCh[CH_TYPE_ROLL];
    
    if((pRCCh[CH_TYPE_PITCH] < PITCH_ANG_MIN) || (pRCCh[CH_TYPE_PITCH] > PITCH_ANG_MAX))
        pRCCh[CH_TYPE_PITCH] = nRCPrevCh[CH_TYPE_PITCH];
    
    if((pRCCh[CH_TYPE_YAW] < YAW_RATE_MIN) || (pRCCh[CH_TYPE_YAW] > YAW_RATE_MAX))
        pRCCh[CH_TYPE_YAW] = nRCPrevCh[CH_TYPE_YAW];

    nRCPrevCh[CH_TYPE_ROLL] = pRCCh[CH_TYPE_ROLL];
    nRCPrevCh[CH_TYPE_PITCH] = pRCCh[CH_TYPE_PITCH];
    nRCPrevCh[CH_TYPE_YAW] = pRCCh[CH_TYPE_YAW];

    if(abs(pFineRPY[0] - nPrevFineRPY[0]) > 30)
        pFineRPY[0] = nPrevFineRPY[0];

    if(abs(pFineRPY[1] - nPrevFineRPY[1]) > 30)
        pFineRPY[1] = nPrevFineRPY[1];

    //ROLL control
    pRoll->nAngleErr = pRCCh[CH_TYPE_ROLL] - pFineRPY[0];
    pRoll->nCurrErrRate = pRoll->nAngleErr * ROLL_OUTER_P_GAIN - pFineGyro[0];
    pRoll->nP_ErrRate = pRoll->nCurrErrRate * ROLL_INNER_P_GAIN;
    pRoll->nI_ErrRate = _Clip3Float((pRoll->nI_ErrRate + (pRoll->nCurrErrRate * ROLL_INNER_I_GAIN) * nDiffTime), -100, 100);
    pRoll->nD_ErrRate = (pRoll->nCurrErrRate - pRoll->nPrevErrRate) * ROLL_INNER_D_GAIN / nDiffTime;
    pRoll->nBalance = pRoll->nP_ErrRate + pRoll->nI_ErrRate + pRoll->nD_ErrRate;

    //PITCH control
    pPitch->nAngleErr = pRCCh[CH_TYPE_PITCH] - pFineRPY[1];
    pPitch->nCurrErrRate = pPitch->nAngleErr * PITCH_OUTER_P_GAIN + pFineGyro[1];
    pPitch->nP_ErrRate = pPitch->nCurrErrRate * PITCH_INNER_P_GAIN;
    pPitch->nI_ErrRate = _Clip3Float((pPitch->nI_ErrRate + (pPitch->nCurrErrRate * PITCH_INNER_I_GAIN) * nDiffTime), -100, 100);
    pPitch->nD_ErrRate = (pPitch->nCurrErrRate - pPitch->nPrevErrRate) * PITCH_INNER_D_GAIN / nDiffTime;
    pPitch->nBalance = pPitch->nP_ErrRate + pPitch->nI_ErrRate + pPitch->nD_ErrRate;

    //YAW control
    pYaw->nCurrErrRate = pRCCh[CH_TYPE_YAW] + pFineGyro[2];// - pFineRPY[2];
    pYaw->nP_ErrRate = pYaw->nCurrErrRate * YAW_P_GAIN;
    pYaw->nI_ErrRate = _Clip3Float((pYaw->nI_ErrRate + pYaw->nCurrErrRate * YAW_I_GAIN * nDiffTime), -50, 50);
    pYaw->nTorque = pYaw->nP_ErrRate + pYaw->nI_ErrRate;

    // Backup for Next
    pRoll->nPrevErrRate = pRoll->nCurrErrRate;
    pPitch->nPrevErrRate = pPitch->nCurrErrRate;
    nPrevFineRPY[0] = pFineRPY[0];
    nPrevFineRPY[1] = pFineRPY[1];
    nPrevFineRPY[2] = pFineRPY[2];
}


inline void _CalculateThrottleVal()
{
    float                   nEstimatedThrottle = 0.0f;
    static float            nPrevEstimatedThrottle = 0.0f;
    AxisErrRate_T           *pPitch = &(nSelfFlyHndl.nPitch);
    AxisErrRate_T           *pRoll = &(nSelfFlyHndl.nRoll);
    AxisErrRate_T           *pYaw = &(nSelfFlyHndl.nYaw);
    unsigned long           *pThrottle = &(nSelfFlyHndl.nThrottle[0]);
    float                   *pRCCh = &(nSelfFlyHndl.nRCCh[0]);

    memset((void *)(&(pThrottle[0])), ESC_MIN, 4 * sizeof(int32_t));

    pRCCh[CH_TYPE_THROTTLE] = floor(pRCCh[CH_TYPE_THROTTLE] / ROUNDING_BASE) * ROUNDING_BASE;
    nEstimatedThrottle = (float)(map(pRCCh[CH_TYPE_THROTTLE], RC_CH2_LOW, RC_CH2_HIGH, ESC_MIN, ESC_MAX));

    if((nEstimatedThrottle < ESC_MIN) || (nEstimatedThrottle > ESC_MAX))
        nEstimatedThrottle = nPrevEstimatedThrottle;

    nPrevEstimatedThrottle = nEstimatedThrottle;

    if(nEstimatedThrottle > ESC_TAKEOFF_OFFSET)
    {
        pThrottle[0] = _Clip3Int((( pPitch->nBalance + pRoll->nBalance) * 0.5 - pYaw->nTorque + nEstimatedThrottle), ESC_MIN, ESC_MAX);
        pThrottle[1] = _Clip3Int((( pPitch->nBalance - pRoll->nBalance) * 0.5 + pYaw->nTorque + nEstimatedThrottle), ESC_MIN, ESC_MAX);
        pThrottle[2] = _Clip3Int(((-pPitch->nBalance - pRoll->nBalance) * 0.5 - pYaw->nTorque + nEstimatedThrottle), ESC_MIN, ESC_MAX);
        pThrottle[3] = _Clip3Int(((-pPitch->nBalance + pRoll->nBalance) * 0.5 + pYaw->nTorque + nEstimatedThrottle), ESC_MIN, ESC_MAX);
    }
}


inline void _UpdateESCs()
{
    unsigned long           *pThrottle = &(nSelfFlyHndl.nThrottle[0]);
    int                     i = 0;

    #if !USE_DIRECTACCESS_REG
    {
        for(i=0 ; i<MAX_CH_ESC ; i++)
            nSelfFlyHndl.pESC[i]->writeMicroseconds(pThrottle[i]);
    }
    #else
    {
        unsigned long       nLoopTimer = 0;
        //while(micros() - loop_timer < 4000);                                      //We wait until 4000us are passed.
        //loop_timer = micros();                                                    //Set the timer for the next loop.
        
        nLoopTimer = micros();
        
        PORTB |= B00011110;                                                         //Set Digital Port 8, 9, 10, and 11 as high.
        
        // Set Relative Throttle Value by Adding Current Time
        for(i=0 ; i<MAX_CH_ESC ; i++)
            pThrottle[i] += nLoopTimer;

        while(PORTB & B00011110)
        {
            const unsigned long     nCurrTime = micros();

            if(pThrottle[0] <= nCurrTime)
                PORTB &= B11111101;
            
            if(pThrottle[1] <= nCurrTime)
                PORTB &= B11111011;
            
            if(pThrottle[2] <= nCurrTime)
                PORTB &= B11110111;
            
            if(pThrottle[3] <= nCurrTime)
                PORTB &= B11101111;
        }
    }
    #endif
}


#if !USE_DIRECTACCESS_REG
inline void _nRCInterrupt_CB0()
{
    const unsigned long     nCurrTime = micros();

    if(PIND & B01000000)
        nSelfFlyHndl.nRCPrevChangeTime[CH_TYPE_ROLL] = nCurrTime;
    else
        nSelfFlyHndl.nRCCh[CH_TYPE_ROLL] = nCurrTime - nSelfFlyHndl.nRCPrevChangeTime[CH_TYPE_ROLL];
}


inline void _nRCInterrupt_CB1()
{
    const unsigned long     nCurrTime = micros();

    if(PIND & B00100000)
        nSelfFlyHndl.nRCPrevChangeTime[CH_TYPE_PITCH] = nCurrTime;
    else
        nSelfFlyHndl.nRCCh[CH_TYPE_PITCH] = nCurrTime - nSelfFlyHndl.nRCPrevChangeTime[CH_TYPE_PITCH];
}


inline void _nRCInterrupt_CB2()
{
    const unsigned long     nCurrTime = micros();

    if(PIND & B00010000)
        nSelfFlyHndl.nRCPrevChangeTime[CH_TYPE_THROTTLE] = nCurrTime;
    else
        nSelfFlyHndl.nRCCh[CH_TYPE_THROTTLE] = nCurrTime - nSelfFlyHndl.nRCPrevChangeTime[CH_TYPE_THROTTLE];
}


inline void _nRCInterrupt_CB3()
{
    const unsigned long     nCurrTime = micros();

    if(PIND & B00001000)
        nSelfFlyHndl.nRCPrevChangeTime[CH_TYPE_YAW] = nCurrTime;
    else
        nSelfFlyHndl.nRCCh[CH_TYPE_YAW] = nCurrTime - nSelfFlyHndl.nRCPrevChangeTime[CH_TYPE_YAW];
}


inline void _nRCInterrupt_CB4()
{
    const unsigned long     nCurrTime = micros();

    if(PIND & B00000100)
        nSelfFlyHndl.nRCPrevChangeTime[CH_TYPE_TAKE_LAND] = nCurrTime;
    else
        nSelfFlyHndl.nRCCh[CH_TYPE_TAKE_LAND] = nCurrTime - nSelfFlyHndl.nRCPrevChangeTime[CH_TYPE_TAKE_LAND];
}
#else
ISR(PCINT2_vect)
{
    const unsigned long     nCurrTime = micros();

    if(PIND & B01000000)
    {
        if(0 == nSelfFlyHndl.nRCPrevChangeTime[CH_TYPE_ROLL])
            nSelfFlyHndl.nRCPrevChangeTime[CH_TYPE_ROLL] = nCurrTime;
    }
    else if(0 != nSelfFlyHndl.nRCPrevChangeTime[CH_TYPE_ROLL])
    {
        nSelfFlyHndl.nRCCh[CH_TYPE_ROLL] = nCurrTime - nSelfFlyHndl.nRCPrevChangeTime[CH_TYPE_ROLL];
        nSelfFlyHndl.nRCPrevChangeTime[CH_TYPE_ROLL] = 0;
    }

    if(PIND & B00100000)
    {
        if(0 == nSelfFlyHndl.nRCPrevChangeTime[CH_TYPE_PITCH])
            nSelfFlyHndl.nRCPrevChangeTime[CH_TYPE_PITCH] = nCurrTime;
    }
    else if(0 != nSelfFlyHndl.nRCPrevChangeTime[CH_TYPE_PITCH])
    {
        nSelfFlyHndl.nRCCh[CH_TYPE_PITCH] = nCurrTime - nSelfFlyHndl.nRCPrevChangeTime[CH_TYPE_PITCH];
        nSelfFlyHndl.nRCPrevChangeTime[CH_TYPE_PITCH] = 0;
    }

    if(PIND & B00010000)
    {
        if(0 == nSelfFlyHndl.nRCPrevChangeTime[CH_TYPE_THROTTLE])
            nSelfFlyHndl.nRCPrevChangeTime[CH_TYPE_THROTTLE] = nCurrTime;
    }
    else if(0 != nSelfFlyHndl.nRCPrevChangeTime[CH_TYPE_THROTTLE])
    {
        nSelfFlyHndl.nRCCh[CH_TYPE_THROTTLE] = nCurrTime - nSelfFlyHndl.nRCPrevChangeTime[CH_TYPE_THROTTLE];
        nSelfFlyHndl.nRCPrevChangeTime[CH_TYPE_THROTTLE] = 0;
    }

    if(PIND & B00001000)
    {
        if(0 == nSelfFlyHndl.nRCPrevChangeTime[CH_TYPE_YAW])
            nSelfFlyHndl.nRCPrevChangeTime[CH_TYPE_YAW] = nCurrTime;
    }
    else if(0 != nSelfFlyHndl.nRCPrevChangeTime[CH_TYPE_YAW])
    {
        nSelfFlyHndl.nRCCh[CH_TYPE_YAW] = nCurrTime - nSelfFlyHndl.nRCPrevChangeTime[CH_TYPE_YAW];
        nSelfFlyHndl.nRCPrevChangeTime[CH_TYPE_YAW] = 0;
    }

    if(PIND & B00000100)
    {
        if(0 == nSelfFlyHndl.nRCPrevChangeTime[CH_TYPE_TAKE_LAND])
            nSelfFlyHndl.nRCPrevChangeTime[CH_TYPE_TAKE_LAND] = nCurrTime;
    }
    else if(0 != nSelfFlyHndl.nRCPrevChangeTime[CH_TYPE_TAKE_LAND])
    {
        nSelfFlyHndl.nRCCh[CH_TYPE_TAKE_LAND] = nCurrTime - nSelfFlyHndl.nRCPrevChangeTime[CH_TYPE_TAKE_LAND];
        nSelfFlyHndl.nRCPrevChangeTime[CH_TYPE_TAKE_LAND] = 0;
    }
}
#endif


float _Clip3Float(const float nValue, const int MIN, const int MAX)
{
    float               nClipVal = nValue;

    if(nValue < MIN)
        nClipVal = MIN;
    else if(nValue > MAX)
        nClipVal = MAX;

    return nClipVal;
}


int _Clip3Int(const int nValue, const int MIN, const int MAX)
{
    int                 nClipVal = nValue;

    if(nValue < MIN)
        nClipVal = MIN;
    else if(nValue > MAX)
        nClipVal = MAX;

    return nClipVal;
}


/**
 * Fast inverse square root implementation
 * @see http://en.wikipedia.org/wiki/Fast_inverse_square_root
 */
float _InvSqrt(float nNumber)
{
     long           i;
     float          x, y;
     const float    f = 1.5F;

    x = nNumber * 0.5F;
    y = nNumber;
    i = * ( long * ) &y;
    i = 0x5f375a86 - ( i >> 1 );
    y = * ( float * ) &i;
    y = y * ( f - ( x * y * y ) );

    return y;
}


#if __PRINT_DEBUG__
void _print_RC_Signals()
{
     float          *pRCCh = &(nSelfFlyHndl.nRCCh[0]);

    Serialprint("   //   RC_Roll:");
    if(pRCCh[CH_TYPE_ROLL] - 1480 < 0)Serialprint("<<<");
    else if(pRCCh[CH_TYPE_ROLL] - 1520 > 0)Serialprint(">>>");
    else Serialprint("-+-");
    Serialprint(pRCCh[CH_TYPE_ROLL]);

    Serialprint("   RC_Pitch:");
    if(pRCCh[CH_TYPE_PITCH] - 1480 < 0)Serialprint("^^^");
    else if(pRCCh[CH_TYPE_PITCH] - 1520 > 0)Serialprint("vvv");
    else Serialprint("-+-");
    Serialprint(pRCCh[CH_TYPE_PITCH]);

    Serialprint("   RC_Throttle:");
    if(pRCCh[CH_TYPE_THROTTLE] - 1480 < 0)Serialprint("vvv");
    else if(pRCCh[CH_TYPE_THROTTLE] - 1520 > 0)Serialprint("^^^");
    else Serialprint("-+-");
    Serialprint(pRCCh[CH_TYPE_THROTTLE]);

    Serialprint("   RC_Yaw:");
    if(pRCCh[CH_TYPE_YAW] - 1480 < 0)Serialprint("<<<");
    else if(pRCCh[CH_TYPE_YAW] - 1520 > 0)Serialprint(">>>");
    else Serialprint("-+-");
    Serialprint(pRCCh[CH_TYPE_YAW]);

    Serialprint("   RC_Gear:");
    if(pRCCh[CH_TYPE_TAKE_LAND] - 1480 < 0)Serialprint("<<<");
    else if(pRCCh[CH_TYPE_TAKE_LAND] - 1520 > 0)Serialprint(">>>");
    else Serialprint("-+-");
    Serialprint(pRCCh[CH_TYPE_TAKE_LAND]);
}


void _print_Gyro_Signals()
{
    float                   *pFineAngle = &(nSelfFlyHndl.nAccelGyroParam.nFineAngle[0]);

    Serialprint("   Gx: ");
    Serialprint(nSelfFlyHndl.nAccelGyroParam.nRawGyro[0]);
    Serialprint("   Gy: ");
    Serialprint(nSelfFlyHndl.nAccelGyroParam.nRawGyro[1]);
    Serialprint("   Gz: ");
    Serialprint(nSelfFlyHndl.nAccelGyroParam.nRawGyro[2]);

    Serialprint("   Ax: ");
    Serialprint(nSelfFlyHndl.nAccelGyroParam.nRawAccel[0]);
    Serialprint("   Ay: ");
    Serialprint(nSelfFlyHndl.nAccelGyroParam.nRawAccel[1]);
    Serialprint("   Az: ");
    Serialprint(nSelfFlyHndl.nAccelGyroParam.nRawAccel[2]);

    Serialprint("   Temp : ");
    Serialprint(nSelfFlyHndl.nAccelGyroParam.nRawTemp/340.00 + 36.53);
}


void _print_Throttle_Signals()
{
     unsigned long  *pThrottle = &(nSelfFlyHndl.nThrottle[0]);

    Serialprint("   //    Thrt1 : ");
    Serialprint(pThrottle[0]);
    Serialprint("  Thrt2 : ");
    Serialprint(pThrottle[1]);
    Serialprint("  Thrt3 : ");
    Serialprint(pThrottle[2]);
    Serialprint("  Thrt4 : ");
    Serialprint(pThrottle[3]);
}


void _print_RPY_Signals()
{
     float          *pFineRPY = &(nSelfFlyHndl.nFineRPY[0]);

    Serialprint("   //    Roll: ");
    Serialprint(pFineRPY[0]);
    Serialprint("   Pitch: ");
    Serialprint(pFineRPY[1]);
    Serialprint("   Yaw: ");
    Serialprint(pFineRPY[2]);
}

void _print_MagData()
{
    MagneticParam_T         *pMagParam = &(nSelfFlyHndl.nMagParam);

    Serialprint("   //   Mx:"); Serialprint(pMagParam->nRawMag[0]);
    Serialprint("   pRawMag[Y_AXIS]:"); Serialprint(pMagParam->nRawMag[1]);
    Serialprint("   Mz:"); Serialprint(pMagParam->nRawMag[2]);
    Serialprint("   Magnetic HEAD:"); Serialprint(pMagParam->nMagHeadingDeg);
    Serialprint("   SmoothHEAD:"); Serialprint(pMagParam->nSmoothHeadingDegrees);
}

void _print_BarometerData()
{
    BaroParam_T             *pBaroParam = &(nSelfFlyHndl.nBaroParam);

    Serialprint("   //    Barometer AvgTemp:"); Serialprint(pBaroParam->nAvgTemp);
    Serialprint("   AvgPress:"); Serialprint(pBaroParam->nAvgPressure);
    Serialprint("   AvgAlt:"); Serialprint(pBaroParam->nAvgAbsoluteAltitude);
    Serialprint("   RelativeAlt:"); Serialprint(pBaroParam->nRelativeAltitude);
    Serialprint("   VerticalSpeed:"); Serialprint(pBaroParam->nVerticalSpeed);
    Serialprint("   ");
}

void _print_SonarData()
{
    Serialprint("   //    Sonar: ");
    Serialprint(nSelfFlyHndl.SonicParam.nDistFromGnd);
    Serialprintln("cm   ");
}
#endif


