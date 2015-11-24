/*----------------------------------------------------------------------------------------
 Constant Definitions
 ----------------------------------------------------------------------------------------*/
#define __DEBUG__                           (1)
#if (__DEBUG__)
    #define __PROFILE__                     (1)
#else
    #define __PROFILE__                     (0)
#endif

#define USE_AHRS                            (1)

// Define Axis
#define X_AXIS                              (0)
#define Y_AXIS                              (1)
#define Z_AXIS                              (2)

// Define Max Number of Each Field
#define MAX_CH_RC                           (5)
#define MAX_CH_ESC                          (4)

// Arduino Pin configuration
//#define PIN_GY86_EXT_INTERRUPT              (13)
#define PIN_SONIC_TRIG                      (13)
#define PIN_SONIC_ECHO                      (12)
#define PIN_ESC_CH0                         (11)
#define PIN_ESC_CH1                         (10)
#define PIN_ESC_CH2                         (9)
#define PIN_ESC_CH3                         (8)
#define PIN_GY86_EXT_INTERRUPT              (7)
#define PIN_RC_CH0                          (6)
#define PIN_RC_CH1                          (5)
#define PIN_RC_CH2                          (4)
#define PIN_RC_CH3                          (3)
#define PIN_RC_CH4                          (2)
#define PIN_RESERVED_01                     (1)
#define PIN_RESERVED_00                     (0)
#define PIN_CHECK_POWER_STAT                (A0)

// ESC configuration
#define ESC_MIN                             (800)
#define ESC_MAX                             (2200)
#define ESC_TAKEOFF_OFFSET                  (900)
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

#define ROUNDING_BASE                       (50)
#define SAMPLING_TIME                       (0.01)                          // Seconds

#define RAD_TO_DEG_SCALE                    (57.2958f)                      // = 180 / PI
#define DEG_TO_RAD_SCALE                    (0.0175f)                       // = PI / 180
#define SINGLE_RADIAN                       (3.141592)                      // = PI
#define DOUBLE_RADIAN                       (6.283184)                      // = 2 * PI
#define BARO_SEA_LEVEL_BASE                 (1013.25)                       // Base Sea Level

/*----------------------------------------------------------------------------------------
 File Inclusions
 ----------------------------------------------------------------------------------------*/
#include <Servo.h>
#include <I2Cdev.h>
#include <PinChangeInt.h>
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include <Wire.h>
#endif
//#include <MPU6050.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <HMC5883L.h>
#include <MS561101BA.h>
#include <math.h>


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
typedef struct _AxisInt16_T
{
    int16_t            XAxis;
    int16_t            YAxis;
    int16_t            ZAxis;
}AxisInt16_T;

typedef struct _AxisFloat_T
{
    float              XAxis;
    float              YAxis;
    float              ZAxis;
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
    float               nBaseGyro[3];
    float               nBaseAccel[3];
    float               nFineAngle[3];                          // Filtered Angles
}AccelGyroParam_T;

typedef struct _MagParam_T
{
    float               nRawMagData[3];
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
    Servo               nESC[MAX_CH_ESC];
    MPU6050             nAccelGyroHndl;                         // MPU6050 Gyroscope Interface
    AccelGyroParam_T    nAccelGyroParam;
    Quaternion          nQuater;
    VectorFloat         nGravity;
    
    HMC5883L            nMagHndl;                               // HMC5883 Magnetic Interface
    MagneticParam_T     nMagParam;
    
    MS561101BA          nBaroHndl;                              // MS5611 Barometer Interface
    BaroParam_T         nBaroParam;
    
    SonicParam_T        SonicParam;                             // HC-SR04 SuperSonic Sensor
    
    AxisErrRate_T       nPitch;
    AxisErrRate_T       nRoll;
    AxisErrRate_T       nYaw;
    float               nRCCh[MAX_CH_RC];                       // RC channel inputs
    unsigned long       nRCPrevChangeTime[MAX_CH_RC];
    
    int32_t             nThrottle[MAX_CH_ESC];
    
    float               nFineRPY[3];
    float               nFineGyro[3];
    volatile float      nQuaternion[4];                         // quaternion
    float               nEstGravity[3];                         // estimated gravity direction
    
    unsigned long       nCurrSensorCapTime;
    unsigned long       nPrevSensorCapTime;
    float               nDiffTime;
    float               nSampleFreq;                            // half the sample period expressed in seconds
    volatile float      nBeta;
    
    volatile bool       nMPUInterruptFlag;
    bool                nInterruptLockFlag;                     // Interrupt lock
}SelfFly_T;


/*----------------------------------------------------------------------------------------
 Static Function
 ----------------------------------------------------------------------------------------*/
int _AccelGyro_Initialize();
void _AccelGyro_GetData();
#if !(USE_AHRS)
void _AccelGyro_CalculateAngle();
#endif
void _AccelGyro_Calibrate();
int _Mag_Initialize();
void _Mag_GetData();
void _Mag_CalculateDirection();
int _Barometer_Initialize();
void _Barometer_GetData();
void _Barometer_CalculateData();
void _Sonic_Initialize();
void _Sonic_GetData();
inline void UpdateESCs();
inline void _CalculatePID();
inline void CalculateThrottleVal();
inline void arm();
void _ESC_Initialize();
void _RC_Initialize();
void _GetSensorRawData();
void _Normailize_SensorVal();
void _Get_RollPitchYaw();
void _Get_Quaternion();
void _Get_RollPitchYawRad_By_Q();
void _AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
//void _AHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);
float _InvSqrt(float nNumber);
inline void nRCInterrupt_CB0();
inline void nRCInterrupt_CB1();
inline void nRCInterrupt_CB2();
inline void nRCInterrupt_CB3();
inline void nRCInterrupt_CB4();
float Clip3Float(const float nValue, const int MIN, const int MAX);
int Clip3Int(const int nValue, const int MIN, const int MAX);
inline void _AcquireLock();
inline void _ReleaseLock();
#if __DEBUG__
void _print_RC_Signals();
void _print_Gyro_Signals();
void _print_Throttle_Signals();
void _print_RPY_Signals();
void _print_MagData();
void _print_BarometerData();
#endif

/*----------------------------------------------------------------------------------------
 Static Variable
 ----------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------
 Global Variable
 ----------------------------------------------------------------------------------------*/
SelfFly_T               *pSelfFlyHndl = NULL;                            // SelfFly Main Handle

/*----------------------------------------------------------------------------------------
 Function Implementation
 ----------------------------------------------------------------------------------------*/
void setup()
{
    int32_t                 i = 0;
    uint8_t                 nDevStatus;                         // return status after each device operation (0 = success, !0 = error)
    AccelGyroParam_T        *pAccelGyroParam = &(pSelfFlyHndl->nAccelGyroParam);
    MagneticParam_T         *pMagParam = &(pSelfFlyHndl->nMagParam);
    BaroParam_T             *pBaroParam = &(pSelfFlyHndl->nBaroParam);
    
    Serialprintln("");    Serialprintln("");    Serialprintln("");    Serialprintln("");
    Serialprintln("   **********************************************   ");
    Serialprintln("   **********************************************   ");
    
    pSelfFlyHndl = (SelfFly_T *) malloc(sizeof(SelfFly_T));

    memset(pSelfFlyHndl, 0, sizeof(SelfFly_T));
    
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
    #endif
    
    #if __DEBUG__
    Serial.begin(115200);
    Serial.flush();
    
    while(!Serial); // wait for Leonardo enumeration, others continue immediately
    #endif
    
    // Initialize Gyro_Accel
    _AccelGyro_Initialize();
    
    // Initialize Magnetic
    _Mag_Initialize();
    
    // Initialize Barometer
    _Barometer_Initialize();
    
    // Initialize SuperSonic Sensor
    //_Sonic_Initialize();

    // Initialize RemoteController
    _RC_Initialize();
    
    // Initialize ESCs
    _ESC_Initialize();
    
    for(i=0 ; i<MAX_CH_RC ; i++)
        pSelfFlyHndl->nRCPrevChangeTime[i] = micros();
    
    pSelfFlyHndl->nMPUInterruptFlag = false;
    pSelfFlyHndl->nInterruptLockFlag = false;
    pSelfFlyHndl->nQuaternion[0] = 1.0f;
    pSelfFlyHndl->nBeta = BETADEF;
    
    pSelfFlyHndl->nRCCh[0] = 1500;
    pSelfFlyHndl->nRCCh[1] = 1500;
    pSelfFlyHndl->nRCCh[2] = 1500;
    pSelfFlyHndl->nRCCh[3] = 1500;
    
    Serialprintln("   **********************************************   ");
    Serialprintln("   **********************************************   ");
    Serialprintln("");    Serialprintln("");    Serialprintln("");    Serialprintln("");
}


void loop()
{
    AccelGyroParam_T        *pAccelGyroParam = &(pSelfFlyHndl->nAccelGyroParam);
    MagneticParam_T         *pMagParam = &(pSelfFlyHndl->nMagParam);
    BaroParam_T             *pBaroParam = &(pSelfFlyHndl->nBaroParam);
    
    #if __PROFILE__
    float                   nStartTime0 = 0.0f, nEndTime = 0.0f;

    nStartTime0 = micros();
    #endif

    // Calculate Roll, Pitch, and Yaw by Quaternion
    _Get_RollPitchYaw();

    // Calculate Heading
    _Mag_CalculateDirection();

    // Calculate Altitude
    _Barometer_CalculateData();

    // PID Computation
    _CalculatePID();
    
    // Throttle Calculation
    CalculateThrottleVal();
    
    // Update BLDCs
    //UpdateESCs();
    
    //delay(50);
    
    #if __DEBUG__
    //_print_Gyro_Signals();
    //_print_MagData();
    //_print_BarometerData();
    //_print_RPY_Signals();
    //_print_RC_Signals();
    _print_Throttle_Signals();
    #endif

    #if __PROFILE__
    nEndTime = micros();
    Serialprint(" Loop Duration: "); Serialprintln((nEndTime - nStartTime0)/1000);
    #endif
}


int _AccelGyro_Initialize()
{
    pSelfFlyHndl->nAccelGyroHndl = MPU6050();

    Serialprintln(F(" Initializing MPU..."));
    pSelfFlyHndl->nAccelGyroHndl.initialize();
    
    // Verify Vonnection
    Serialprint(F("    Testing device connections..."));
    Serialprintln(pSelfFlyHndl->nAccelGyroHndl.testConnection() ? F("  MPU6050 connection successful") : F("  MPU6050 connection failed"));
    
    pSelfFlyHndl->nAccelGyroHndl.setI2CMasterModeEnabled(false);
    pSelfFlyHndl->nAccelGyroHndl.setI2CBypassEnabled(true);
    pSelfFlyHndl->nAccelGyroHndl.setSleepEnabled(false);
    
    // supply your own gyro offsets here, scaled for min sensitivity
    pSelfFlyHndl->nAccelGyroHndl.setXGyroOffset(220);
    pSelfFlyHndl->nAccelGyroHndl.setYGyroOffset(76);
    pSelfFlyHndl->nAccelGyroHndl.setZGyroOffset(-85);
    pSelfFlyHndl->nAccelGyroHndl.setZAccelOffset(1788);                                     // 1688 factory default for my test chip
    pSelfFlyHndl->nAccelGyroHndl.setRate(1);                                                // Sample Rate (500Hz = 1Hz Gyro SR / 1+1)
    pSelfFlyHndl->nAccelGyroHndl.setDLPFMode(MPU6050_DLPF_BW_20);                           // Low Pass filter 20hz
    pSelfFlyHndl->nAccelGyroHndl.setFullScaleGyroRange(GYRO_FS_PRECISIOM);                // 250? / s (MPU6050_GYRO_FS_250)
    pSelfFlyHndl->nAccelGyroHndl.setFullScaleAccelRange(ACCEL_FS_PRECISIOM);                // +-2g (MPU6050_ACCEL_FS_2)
    
    // Calibrate GyroAccel
    _AccelGyro_Calibrate();
    Serialprintln(F(" MPU Initialized!!!"));
    
    return 0;
}


void _AccelGyro_GetData()
{
    AccelGyroParam_T        *pAccelGyroParam = &(pSelfFlyHndl->nAccelGyroParam);
    float                   *pRawGyro = &(pAccelGyroParam->nRawGyro[X_AXIS]);
    float                   *pRawAccel = &(pAccelGyroParam->nRawAccel[X_AXIS]);
    const float             *pBaseGyro = &(pAccelGyroParam->nBaseGyro[X_AXIS]);
    
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
    
    // Remove Bias of Gyro
    pRawGyro[X_AXIS] = (pRawGyro[X_AXIS] - pBaseGyro[X_AXIS]);
    pRawGyro[Y_AXIS] = (pRawGyro[Y_AXIS] - pBaseGyro[Y_AXIS]);
    pRawGyro[Z_AXIS] = (pRawGyro[Z_AXIS] - pBaseGyro[Z_AXIS]);
}

#if USE_AHRS
void _AccelGyro_CalculateAngle()
{
    AccelGyroParam_T        *pAccelGyroParam = &(pSelfFlyHndl->nAccelGyroParam);
    const float             *pRawGyro = &(pAccelGyroParam->nRawGyro[X_AXIS]);
    const float             *pRawAccel = &(pAccelGyroParam->nRawAccel[X_AXIS]);
    const float             *pBaseGyro = &(pAccelGyroParam->nBaseGyro[X_AXIS]);
    float                   *pFineAngle = &(pAccelGyroParam->nFineAngle[X_AXIS]);
    static float            nGyroAngle[3] = {0, };
    float                   nAccelAngle[3] = {0, };
    float                   nGyroDiffAngle[3] = {0, };
    const float             nGyroWeight = pSelfFlyHndl->nDiffTime / GYRO_FS;
    
    // Convert Gyro Values to Degrees/Sec
    nGyroDiffAngle[X_AXIS] = pRawGyro[X_AXIS] * nGyroWeight;
    nGyroDiffAngle[Y_AXIS] = pRawGyro[Y_AXIS] * nGyroWeight;
    nGyroDiffAngle[Z_AXIS] = pRawGyro[Z_AXIS] * nGyroWeight;
    
    // Calculate
    nAccelAngle[X_AXIS] = atan(pRawAccel[Y_AXIS] / sqrt(pow(pRawAccel[X_AXIS], 2) + pow(pRawAccel[Z_AXIS], 2))) * RAD_TO_DEG_SCALE;
    nAccelAngle[Y_AXIS] = atan((-1) * pRawAccel[X_AXIS] / sqrt(pow(pRawAccel[Y_AXIS], 2) + pow(pRawAccel[Z_AXIS], 2))) * RAD_TO_DEG_SCALE;
    nAccelAngle[Z_AXIS] = 0;
    
    // Compute the (filtered) gyro angles
    nGyroAngle[X_AXIS] = nGyroDiffAngle[X_AXIS] + pFineAngle[X_AXIS];
    nGyroAngle[Y_AXIS] = nGyroDiffAngle[Y_AXIS] + pFineAngle[Y_AXIS];
    nGyroAngle[Z_AXIS] = nGyroDiffAngle[Z_AXIS] + pFineAngle[Z_AXIS];
    
    // Apply the complementary filter to figure out the change in angle - choice of alpha is
    // estimated now.  Alpha depends on the sampling rate...
    {
        const float     nOffset0 = 0.80;
        const float     nOffset1 = 1.0 - nOffset0;
        
        pFineAngle[X_AXIS] = nOffset0 * nGyroAngle[X_AXIS] + nOffset1 * nAccelAngle[X_AXIS];
        pFineAngle[Y_AXIS] = nOffset0 * nGyroAngle[Y_AXIS] + nOffset1 * nAccelAngle[Y_AXIS];
        pFineAngle[Z_AXIS] = nGyroAngle[Z_AXIS];  //Accelerometer doesn't give z-angle
    }
}
#endif


void _AccelGyro_Calibrate()
{
    int                     i = 0;
    int32_t                 nRawGyro[3] = {0, };
    int32_t                 nRawAccel[3] = {0, };
    const int               nLoopCnt = 50;
    AccelGyroParam_T        *pAccelGyroParam = &(pSelfFlyHndl->nAccelGyroParam);

    Serialprint(F("    Start Calibration of MPU6050 "));

    for(i=0 ; i<nLoopCnt ; i++)
    {
        _AccelGyro_GetData();
        
        nRawGyro[X_AXIS] += pAccelGyroParam->nRawGyro[X_AXIS];
        nRawGyro[Y_AXIS] += pAccelGyroParam->nRawGyro[Y_AXIS];
        nRawGyro[Z_AXIS] += pAccelGyroParam->nRawGyro[Z_AXIS];
        nRawAccel[X_AXIS] += pAccelGyroParam->nRawAccel[X_AXIS];
        nRawAccel[Y_AXIS] += pAccelGyroParam->nRawAccel[Y_AXIS];
        nRawAccel[Z_AXIS] += pAccelGyroParam->nRawAccel[Z_AXIS];
        
        delay(20);
        
        Serialprint(".");
    }
    
    // Store the raw calibration values globally
    pAccelGyroParam->nBaseGyro[X_AXIS] = (float)(nRawGyro[X_AXIS]) / nLoopCnt;
    pAccelGyroParam->nBaseGyro[Y_AXIS] = (float)(nRawGyro[Y_AXIS]) / nLoopCnt;
    pAccelGyroParam->nBaseGyro[Z_AXIS] = (float)(nRawGyro[Z_AXIS]) / nLoopCnt;
    pAccelGyroParam->nBaseAccel[X_AXIS] = (float)(nRawAccel[X_AXIS]) / nLoopCnt;
    pAccelGyroParam->nBaseAccel[Y_AXIS] = (float)(nRawAccel[Y_AXIS]) / nLoopCnt;
    pAccelGyroParam->nBaseAccel[Z_AXIS] = (float)(nRawAccel[Z_AXIS]) / nLoopCnt;

    Serialprintln(F("    Done"));
}


int _Mag_Initialize()
{
    pSelfFlyHndl->nMagHndl = HMC5883L();

    // initialize Magnetic
    Serialprintln(F(" Initializing Magnetic..."));
    pSelfFlyHndl->nMagHndl.initialize();
    
    // Verify Vonnection
    Serialprint(F("    Testing device connections..."));
    Serialprintln(pSelfFlyHndl->nMagHndl.testConnection() ? F("  HMC5883L connection successful") : F("  HMC5883L connection failed"));

    // Calibrate Magnetic
    //Serialprint(F("    Start Calibration of Magnetic Sensor (HMC5883L) "));
    //pSelfFlyHndl->nMagHndl.calibrate();
    //pSelfFlyHndl->nMagHndl.calibration_offset(1);
    //Serialprintln(F("Done"));
    
    pSelfFlyHndl->nMagHndl.setMode(HMC5883L_MODE_CONTINUOUS);
    pSelfFlyHndl->nMagHndl.setGain(HMC5883L_GAIN_1090);
    pSelfFlyHndl->nMagHndl.setDataRate(HMC5883L_RATE_75);
    pSelfFlyHndl->nMagHndl.setSampleAveraging(HMC5883L_AVERAGING_8);
    
    // Date: 2015-11-05
    // Location: Seoul, South Korea
    // Latitude: 37.0000° North
    // Longitude: 126.0000° East
    // Magnetic declination: 7° 59.76' West
    // Annual Change (minutes/year): 3.9 '/y West
    // http://www.geomag.nrcan.gc.ca/calc/mdcal-en.php
    // http://www.magnetic-declination.com/
    pSelfFlyHndl->nMagParam.nDeclinationAngle = (7.0 + (59.76 / 60.0)) * DEG_TO_RAD_SCALE;
    
    Serialprintln(F(" Done"));
    
    // Reference WebSite
    // http://www.meccanismocomplesso.org/en/arduino-magnetic-magnetic-magnetometer-hmc5883l/
}


void _Mag_GetData()
{
    int16_t                 nRawMagData[3];
    float         *pRawMagData = &(pSelfFlyHndl->nMagParam.nRawMagData[X_AXIS]);
    
    pSelfFlyHndl->nMagHndl.getScaledHeading(&(pRawMagData[X_AXIS]), &(pRawMagData[Y_AXIS]), &(pRawMagData[Z_AXIS]));
}


void _Mag_CalculateDirection()
{
    int                     i = 0;
    MagneticParam_T         *pMagParam = &(pSelfFlyHndl->nMagParam);
    
    pMagParam->nMagHeadingRad = atan2(pMagParam->nRawMagData[Y_AXIS], pMagParam->nRawMagData[X_AXIS]);
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


int _Barometer_Initialize()
{
    int                     i = 0;
    BaroParam_T             *pBaroParam = &(pSelfFlyHndl->nBaroParam);
    
    Serialprint(F(" Initializing Barometer Sensor (MS5611)..."));
    
    pSelfFlyHndl->nBaroHndl.init(MS561101BA_ADDR_CSB_LOW);
    
    for(i=0 ; i<50 ; i++)
    {
        _Barometer_GetData();
        delay(20);
    }
    
    // Get Average Pressure & Temperature
    pBaroParam->nAvgTemp = pSelfFlyHndl->nBaroHndl.getAvgTemp();
    pBaroParam->nAvgPressure = pSelfFlyHndl->nBaroHndl.getAvgPressure();
    
    // Get Reference Altitude
    pBaroParam->nRefAbsoluteAltitude = pSelfFlyHndl->nBaroHndl.getAltitude(pBaroParam->nAvgPressure, pBaroParam->nAvgTemp);
    
    Serialprintln(F(" Done"));
}


void _Barometer_GetData()
{
    BaroParam_T             *pBaroParam = &(pSelfFlyHndl->nBaroParam);
    
    pBaroParam->nRawTemp = pSelfFlyHndl->nBaroHndl.getTemperature(MS561101BA_OSR_512);
    pBaroParam->nRawPressure = pSelfFlyHndl->nBaroHndl.getPressure(MS561101BA_OSR_512);

    // Push to Array to Get Average Pressure & Temperature
    pSelfFlyHndl->nBaroHndl.pushTemp(pBaroParam->nRawTemp);
    pSelfFlyHndl->nBaroHndl.pushPressure(pBaroParam->nRawPressure);
}


void _Barometer_CalculateData()
{
    BaroParam_T             *pBaroParam = &(pSelfFlyHndl->nBaroParam);
 
    // Get Average Pressure & Temperature
    pBaroParam->nAvgTemp = pSelfFlyHndl->nBaroHndl.getAvgTemp();
    pBaroParam->nAvgPressure = pSelfFlyHndl->nBaroHndl.getAvgPressure();

    // Get Altitude
    pBaroParam->nRawAbsoluteAltitude = pSelfFlyHndl->nBaroHndl.getAltitude(pBaroParam->nAvgPressure, pBaroParam->nAvgTemp);
    
    // Push to Array to Get Average Altitude
    pSelfFlyHndl->nBaroHndl.pushAltitude(pBaroParam->nRawAbsoluteAltitude);
    
    // Get Average Pressure & Temperature
    pBaroParam->nAvgAbsoluteAltitude = pSelfFlyHndl->nBaroHndl.getAvgAltitude();
    
    // Get Vertical Speed
    pBaroParam->nVerticalSpeed = abs(pBaroParam->nAvgAbsoluteAltitude - pBaroParam->nPrevAvgAbsoluteAltitude) / (pSelfFlyHndl->nDiffTime);
    pBaroParam->nRelativeAltitude = pBaroParam->nAvgAbsoluteAltitude - pBaroParam->nRefAbsoluteAltitude;
    
    pBaroParam->nPrevAvgAbsoluteAltitude = pBaroParam->nAvgAbsoluteAltitude;
}


void _Sonic_Initialize()
{
    int                     i = 0;
    
    // Set PinMode for SuperSonic Sensor (HC-SR04)
    pinMode(PIN_SONIC_TRIG, OUTPUT);
    pinMode(PIN_SONIC_ECHO, INPUT);
    
    // Calibrate SuperSonic Sensor
    for(i=0 ; i<50 ; i++)
    {
        _Sonic_GetData();
        delay(20);
    }
}


void _Sonic_GetData()
{
    digitalWrite(PIN_SONIC_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(PIN_SONIC_TRIG, LOW);
    
    // Get Raw Distance Value
    pSelfFlyHndl->SonicParam.nRawDist = pulseIn(PIN_SONIC_ECHO, HIGH);
    
    // Calculate Distance From Ground
    pSelfFlyHndl->SonicParam.nDistFromGnd = pSelfFlyHndl->SonicParam.nRawDist * 0.017; // (340(m/s) * 1000(mm) / 1000000(microsec) / 2(oneway))
}


inline void UpdateESCs()
{
    Servo                   *pESC = &(pSelfFlyHndl->nESC[0]);
    int32_t                 *pThrottle = &(pSelfFlyHndl->nThrottle[0]);
    int                     i = 0;
    
    for(i=0 ; i<MAX_CH_ESC ; i++)
        pESC[i].writeMicroseconds(pThrottle[i]);
}


inline void _CalculatePID()
{
    static float            nRCPrevCh[5] = {0, };              // Filter variables
    static float            nPrevFineRPY[3];
    AxisErrRate_T           *pPitch = &(pSelfFlyHndl->nPitch);
    AxisErrRate_T           *pRoll = &(pSelfFlyHndl->nRoll);
    AxisErrRate_T           *pYaw = &(pSelfFlyHndl->nYaw);
    float                   *pRCCh = &(pSelfFlyHndl->nRCCh[0]);
    float                   *pFineGyro = &(pSelfFlyHndl->nFineGyro[0]);
    float                   *pFineRPY = &(pSelfFlyHndl->nFineRPY[0]);
    const float             nDiffTime = pSelfFlyHndl->nDiffTime;

    _AcquireLock();
    
    pRCCh[0] = floor(pRCCh[0] / ROUNDING_BASE) * ROUNDING_BASE;
    pRCCh[1] = floor(pRCCh[1] / ROUNDING_BASE) * ROUNDING_BASE;
    pRCCh[3] = floor(pRCCh[3] / ROUNDING_BASE) * ROUNDING_BASE;
    
    pRCCh[0] = map(pRCCh[0], RC_CH0_LOW, RC_CH0_HIGH, ROLL_ANG_MIN, ROLL_ANG_MAX) - 1;
    pRCCh[1] = map(pRCCh[1], RC_CH1_LOW, RC_CH1_HIGH, PITCH_ANG_MIN, PITCH_ANG_MAX) - 1;
    pRCCh[3] = map(pRCCh[3], RC_CH3_LOW, RC_CH3_HIGH, YAW_RATE_MIN, YAW_RATE_MAX);
    
    if((pRCCh[0] < ROLL_ANG_MIN) || (pRCCh[0] > ROLL_ANG_MAX))
        pRCCh[0] = nRCPrevCh[0];
    if((pRCCh[1] < PITCH_ANG_MIN) || (pRCCh[1] > PITCH_ANG_MAX))
        pRCCh[1] = nRCPrevCh[1];
    if((pRCCh[3] < YAW_RATE_MIN) || (pRCCh[3] > YAW_RATE_MAX))
        pRCCh[3] = nRCPrevCh[3];
    
    nRCPrevCh[0] = pRCCh[0];
    nRCPrevCh[1] = pRCCh[1];
    nRCPrevCh[3] = pRCCh[3];
    
    //pFineRPY[0] = pFineRPY[0] * RAD_TO_DEG_SCALE + ROLL_ANG_OFFSET;
    //pFineRPY[1] = pFineRPY[1] * RAD_TO_DEG_SCALE + PITCH_ANG_OFFSET;
    
    if(abs(pFineRPY[0] - nPrevFineRPY[0]) > 30)
        pFineRPY[0] = nPrevFineRPY[0];
    
    if(abs(pFineRPY[1] - nPrevFineRPY[1]) > 30)
        pFineRPY[1] = nPrevFineRPY[1];
    
    //ROLL control
    pRoll->nAngleErr = pRCCh[0] - pFineRPY[0];
    pRoll->nCurrErrRate = pRoll->nAngleErr * ROLL_OUTER_P_GAIN - pFineGyro[0];
    pRoll->nP_ErrRate = pRoll->nCurrErrRate * ROLL_INNER_P_GAIN;
    pRoll->nI_ErrRate = Clip3Float((pRoll->nI_ErrRate + (pRoll->nCurrErrRate * ROLL_INNER_I_GAIN) * nDiffTime), -100, 100);
    pRoll->nD_ErrRate = (pRoll->nCurrErrRate - pRoll->nPrevErrRate) * ROLL_INNER_D_GAIN / nDiffTime;
    pRoll->nBalance = pRoll->nP_ErrRate + pRoll->nI_ErrRate + pRoll->nD_ErrRate;
    
    //PITCH control
    pPitch->nAngleErr = pRCCh[1] - pFineRPY[1];
    pPitch->nCurrErrRate = pPitch->nAngleErr * PITCH_OUTER_P_GAIN + pFineGyro[1];
    pPitch->nP_ErrRate = pPitch->nCurrErrRate * PITCH_INNER_P_GAIN;
    pPitch->nI_ErrRate = Clip3Float((pPitch->nI_ErrRate + (pPitch->nCurrErrRate * PITCH_INNER_I_GAIN) * nDiffTime), -100, 100);
    pPitch->nD_ErrRate = (pPitch->nCurrErrRate - pPitch->nPrevErrRate) * PITCH_INNER_D_GAIN / nDiffTime;
    pPitch->nBalance = pPitch->nP_ErrRate + pPitch->nI_ErrRate + pPitch->nD_ErrRate;
    
    //YAW control
    pYaw->nCurrErrRate = pRCCh[3] - pFineRPY[2];
    pYaw->nP_ErrRate = pYaw->nCurrErrRate * YAW_P_GAIN;
    pYaw->nI_ErrRate = Clip3Float((pYaw->nI_ErrRate + pYaw->nCurrErrRate * YAW_I_GAIN * nDiffTime), -50, 50);
    pYaw->nTorque = pYaw->nP_ErrRate + pYaw->nI_ErrRate;
    
    // Backup for Next
    pRoll->nPrevErrRate = pRoll->nCurrErrRate;
    pPitch->nPrevErrRate = pPitch->nCurrErrRate;
    nPrevFineRPY[0] = pFineRPY[0];
    nPrevFineRPY[1] = pFineRPY[1];
    nPrevFineRPY[2] = pFineRPY[2];

    _ReleaseLock();
}


inline void CalculateThrottleVal()
{
    float                   nEstimatedThrottle = 0.0f;
    static float            nPrevEstimatedThrottle = 0.0f;
    AxisErrRate_T           *pPitch = &(pSelfFlyHndl->nPitch);
    AxisErrRate_T           *pRoll = &(pSelfFlyHndl->nRoll);
    AxisErrRate_T           *pYaw = &(pSelfFlyHndl->nYaw);
    int32_t                 *pThrottle = &(pSelfFlyHndl->nThrottle[0]);
    float                   *pRCCh = &(pSelfFlyHndl->nRCCh[0]);
    
    _AcquireLock();
    
    memset(&(pThrottle[0]), ESC_MIN, 4 * sizeof(int32_t));
    
    pRCCh[2] = floor(pRCCh[2] / ROUNDING_BASE) * ROUNDING_BASE;
    nEstimatedThrottle = (float)(map(pRCCh[2], RC_CH2_LOW, RC_CH2_HIGH, ESC_MIN, ESC_MAX));
    
    if((nEstimatedThrottle < ESC_MIN) || (nEstimatedThrottle > ESC_MAX))
        nEstimatedThrottle = nPrevEstimatedThrottle;
    
    nPrevEstimatedThrottle = nEstimatedThrottle;
    
    _ReleaseLock();
    
    if(nEstimatedThrottle > ESC_TAKEOFF_OFFSET)
    {
        pThrottle[0] = Clip3Int((( pPitch->nBalance - pRoll->nBalance) * 0.5 + pYaw->nTorque + nEstimatedThrottle), ESC_MIN, ESC_MAX);
        pThrottle[1] = Clip3Int(((-pPitch->nBalance - pRoll->nBalance) * 0.5 - pYaw->nTorque + nEstimatedThrottle), ESC_MIN, ESC_MAX);
        pThrottle[2] = Clip3Int(((-pPitch->nBalance + pRoll->nBalance) * 0.5 + pYaw->nTorque + nEstimatedThrottle), ESC_MIN, ESC_MAX);
        pThrottle[3] = Clip3Int((( pPitch->nBalance + pRoll->nBalance) * 0.5 - pYaw->nTorque + nEstimatedThrottle), ESC_MIN, ESC_MAX);
    }
}


inline void arm()
{
    Servo                   *pESC = &(pSelfFlyHndl->nESC[0]);
    int                     i = 0;
    
    for(i=0 ; i<MAX_CH_ESC ; i++)
        pESC[i].writeMicroseconds(ESC_MIN);
    
    delay(ESC_ARM_DELAY);
}


void _ESC_Initialize()
{
    Servo                   *pESC = &(pSelfFlyHndl->nESC[0]);
    int                     i = 0;
    
    for(i=0 ; i<MAX_CH_ESC ; i++)
        pESC[i].attach(PIN_ESC_CH0);
    
    delay(1000);
    
    arm();
}


void _RC_Initialize()
{
    pinMode(PIN_CHECK_POWER_STAT, OUTPUT);
    digitalWrite(PIN_CHECK_POWER_STAT, HIGH);
    
    PCintPort::attachInterrupt(PIN_RC_CH0, nRCInterrupt_CB0, CHANGE);
    PCintPort::attachInterrupt(PIN_RC_CH1, nRCInterrupt_CB1, CHANGE);
    PCintPort::attachInterrupt(PIN_RC_CH2, nRCInterrupt_CB2, CHANGE);
    PCintPort::attachInterrupt(PIN_RC_CH3, nRCInterrupt_CB3, CHANGE);
    PCintPort::attachInterrupt(PIN_RC_CH4, nRCInterrupt_CB4, CHANGE);
}


void _GetSensorRawData()
{
    pSelfFlyHndl->nPrevSensorCapTime = pSelfFlyHndl->nCurrSensorCapTime;
    pSelfFlyHndl->nCurrSensorCapTime = micros();

    pSelfFlyHndl->nDiffTime = (pSelfFlyHndl->nCurrSensorCapTime - pSelfFlyHndl->nPrevSensorCapTime) / 1000000.0;
    pSelfFlyHndl->nSampleFreq = 1.0 / pSelfFlyHndl->nDiffTime;//1000000.0 / ((pSelfFlyHndl->nCurrSensorCapTime - pSelfFlyHndl->nPrevSensorCapTime));

    // Get AccelGyro Raw Data
    _AccelGyro_GetData();
    
    // Get Magnetic Raw Data
    _Mag_GetData();
    
    // Get Barometer Raw Data
    _Barometer_GetData();

    // Get Supersonic Raw Data
    //_Sonic_GetData();
    
    // Normalize Sensor Values
    //_Normailize_SensorVal();
}


void _Normailize_SensorVal()
{
    float                   *pRawGyro = &(pSelfFlyHndl->nAccelGyroParam.nRawGyro[X_AXIS]);
    float                   *pRawAccel = &(pSelfFlyHndl->nAccelGyroParam.nRawAccel[X_AXIS]);
    float                   *pRawMagData = &(pSelfFlyHndl->nMagParam.nRawMagData[X_AXIS]);
}


void _Get_RollPitchYaw()
{
    float                   *pFineG = &(pSelfFlyHndl->nEstGravity[X_AXIS]);
    const volatile float    *pQ = &(pSelfFlyHndl->nQuaternion[0]);
    float                   *pFineRPY = &(pSelfFlyHndl->nFineRPY[0]);

    // Calculate Roll & Pitch & Yaw
    _Get_RollPitchYawRad_By_Q();
    
    // Convert Radian to Degree
    pFineRPY[0] *= RAD_TO_DEG_SCALE;
    pFineRPY[1] *= RAD_TO_DEG_SCALE;
    pFineRPY[2] *= RAD_TO_DEG_SCALE;
}


void _Get_Quaternion()
{
    float                   *pRawGyro = &(pSelfFlyHndl->nAccelGyroParam.nRawGyro[X_AXIS]);
    float                   *pRawAccel = &(pSelfFlyHndl->nAccelGyroParam.nRawAccel[X_AXIS]);
    float                   *pRawMagData = &(pSelfFlyHndl->nMagParam.nRawMagData[X_AXIS]);

    // Get Sensor (Gyro / Accel / Megnetic / Baro / Temp)
    _GetSensorRawData();
    
    _AHRSupdate(pRawGyro[X_AXIS]/GYRO_FS*DEG_TO_RAD_SCALE, pRawGyro[Y_AXIS]/GYRO_FS*DEG_TO_RAD_SCALE, pRawGyro[Z_AXIS]/GYRO_FS*DEG_TO_RAD_SCALE,
                pRawAccel[X_AXIS], pRawAccel[Y_AXIS], pRawAccel[Z_AXIS],
                pRawMagData[X_AXIS], pRawMagData[Y_AXIS], pRawMagData[Z_AXIS]);
}


void _Get_RollPitchYawRad_By_Q()
{
    float                   *pEstGravity = &(pSelfFlyHndl->nEstGravity[X_AXIS]);
    const volatile float    *pQ = &(pSelfFlyHndl->nQuaternion[0]);
    float                   *pFineRPY = &(pSelfFlyHndl->nFineRPY[0]);

    // Calculate Quaternion
    _Get_Quaternion();

    // Estimate Gravity
    pEstGravity[X_AXIS] = 2 * ((pQ[1] * pQ[3]) - (pQ[0] * pQ[2]));
    pEstGravity[Y_AXIS] = 2 * ((pQ[0] * pQ[1]) + (pQ[2] * pQ[3]));
    pEstGravity[Z_AXIS] = (pQ[0] * pQ[0]) - (pQ[1] * pQ[1]) - (pQ[2] * pQ[2]) + (pQ[3] * pQ[3]);
    
    // Calculate Roll, Pitch, and Yaw
    pFineRPY[0] = atan(pEstGravity[Y_AXIS] / sqrt((pEstGravity[X_AXIS] * pEstGravity[X_AXIS]) + (pEstGravity[Z_AXIS] * pEstGravity[Z_AXIS])));
    pFineRPY[1] = atan(pEstGravity[X_AXIS] / sqrt((pEstGravity[Y_AXIS] * pEstGravity[Y_AXIS]) + (pEstGravity[Z_AXIS] * pEstGravity[Z_AXIS])));
    pFineRPY[2] = atan2((2 * pQ[1] * pQ[2]) - (2 * pQ[0] * pQ[3]), (2 * pQ[0] * pQ[0]) + (2 * pQ[1] * pQ[1]) - 1);
}


// Reference Site
// http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/
void _AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
{
    float                   recipNorm;
    float                   s0, s1, s2, s3;
    float                   qDot1, qDot2, qDot3, qDot4;
    float                   hx, hy;
    float                   _2q0mx, _2q0my, _2q0mz, _2q1mx;
    float                   _2bx, _2bz, _4bx, _4bz;
    float                   _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3;
    float                   q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
    const float             nDiffTime = pSelfFlyHndl->nDiffTime;
    volatile float          *pQ = &(pSelfFlyHndl->nQuaternion[0]);
    
    // Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
    //if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
    //    _AHRSupdateIMU(gx, gy, gz, ax, ay, az);
    //    return;
    //}
    
    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-pQ[1] * gx - pQ[2] * gy - pQ[3] * gz);
    qDot2 = 0.5f * ( pQ[0] * gx + pQ[2] * gz - pQ[3] * gy);
    qDot3 = 0.5f * ( pQ[0] * gy - pQ[1] * gz + pQ[3] * gx);
    qDot4 = 0.5f * ( pQ[0] * gz + pQ[1] * gy - pQ[2] * gx);
    
    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
    {
        // Normalise accelerometer measurement
        recipNorm = _InvSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;
        
        // Normalise magnetometer measurement
        recipNorm = _InvSqrt(mx * mx + my * my + mz * mz);
        mx *= recipNorm;
        my *= recipNorm;
        mz *= recipNorm;
        
        // Auxiliary variables to avoid repeated arithmetic
        _2q0mx = 2.0f * pQ[0] * mx;
        _2q0my = 2.0f * pQ[0] * my;
        _2q0mz = 2.0f * pQ[0] * mz;
        _2q1mx = 2.0f * pQ[1] * mx;
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
        hx = mx * q0q0 - _2q0my * pQ[3] + _2q0mz * pQ[2] + mx * q1q1 + _2q1 * my * pQ[2] + _2q1 * mz * pQ[3] - mx * q2q2 - mx * q3q3;
        hy = _2q0mx * pQ[3] + my * q0q0 - _2q0mz * pQ[1] + _2q1mx * pQ[2] - my * q1q1 + my * q2q2 + _2q2 * mz * pQ[3] - my * q3q3;
        _2bx = sqrt(hx * hx + hy * hy);
        _2bz = -_2q0mx * pQ[2] + _2q0my * pQ[1] + mz * q0q0 + _2q1mx * pQ[3] - mz * q1q1 + _2q2 * my * pQ[3] - mz * q2q2 + mz * q3q3;
        _4bx = 2.0f * _2bx;
        _4bz = 2.0f * _2bz;
        
        // Gradient decent algorithm corrective step
        s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay)
                    - _2bz * pQ[2] * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx)
                    + (-_2bx * pQ[3] + _2bz * pQ[1]) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my)
                    + _2bx * pQ[2] * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        
        s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay)
                    - 4.0f * pQ[1] * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * pQ[3] * (_2bx * (0.5f - q2q2 - q3q3)
                    + _2bz * (q1q3 - q0q2) - mx) + (_2bx * pQ[2] + _2bz * pQ[0]) * (_2bx * (q1q2 - q0q3)
                    + _2bz * (q0q1 + q2q3) - my) + (_2bx * pQ[3] - _4bz * pQ[1]) * (_2bx * (q0q2 + q1q3)
                    + _2bz * (0.5f - q1q1 - q2q2) - mz);
        
        s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay)
                    - 4.0f * pQ[2] * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * pQ[2] - _2bz * pQ[0]) * (_2bx * (0.5f - q2q2 - q3q3)
                    + _2bz * (q1q3 - q0q2) - mx) + (_2bx * pQ[1] + _2bz * pQ[3]) * (_2bx * (q1q2 - q0q3)
                    + _2bz * (q0q1 + q2q3) - my) + (_2bx * pQ[0] - _4bz * pQ[2]) * (_2bx * (q0q2 + q1q3)
                    + _2bz * (0.5f - q1q1 - q2q2) - mz);
        
        s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay)
                    + (-_4bx * pQ[3] + _2bz * pQ[1]) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx)
                    + (-_2bx * pQ[0] + _2bz * pQ[2]) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my)
                    + _2bx * pQ[1] * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        
        recipNorm = _InvSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;
        
        // Apply feedback step
        qDot1 -= pSelfFlyHndl->nBeta * s0;
        qDot2 -= pSelfFlyHndl->nBeta * s1;
        qDot3 -= pSelfFlyHndl->nBeta * s2;
        qDot4 -= pSelfFlyHndl->nBeta * s3;
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

//---------------------------------------------------------------------------------------------------
// IMU algorithm update

//void _AHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az)
//{
//    float recipNorm;
//    float s0, s1, s2, s3;
//    float qDot1, qDot2, qDot3, qDot4;
//    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;
//    
//    // Rate of change of quaternion from gyroscope
//    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
//    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
//    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
//    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);
//    
//    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
//    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
//        
//        // Normalise accelerometer measurement
//        recipNorm = _InvSqrt(ax * ax + ay * ay + az * az);
//        ax *= recipNorm;
//        ay *= recipNorm;
//        az *= recipNorm;
//        
//        // Auxiliary variables to avoid repeated arithmetic
//        _2q0 = 2.0f * q0;
//        _2q1 = 2.0f * q1;
//        _2q2 = 2.0f * q2;
//        _2q3 = 2.0f * q3;
//        _4q0 = 4.0f * q0;
//        _4q1 = 4.0f * q1;
//        _4q2 = 4.0f * q2;
//        _8q1 = 8.0f * q1;
//        _8q2 = 8.0f * q2;
//        q0q0 = q0 * q0;
//        q1q1 = q1 * q1;
//        q2q2 = q2 * q2;
//        q3q3 = q3 * q3;
//        
//        // Gradient decent algorithm corrective step
//        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
//        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
//        s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
//        s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
//        recipNorm = _InvSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
//        s0 *= recipNorm;
//        s1 *= recipNorm;
//        s2 *= recipNorm;
//        s3 *= recipNorm;
//        
//        // Apply feedback step
//        qDot1 -= beta * s0;
//        qDot2 -= beta * s1;
//        qDot3 -= beta * s2;
//        qDot4 -= beta * s3;
//    }
//    
//    // Integrate rate of change of quaternion to yield quaternion
//    q0 += qDot1 * (1.0f / sampleFreq);
//    q1 += qDot2 * (1.0f / sampleFreq);
//    q2 += qDot3 * (1.0f / sampleFreq);
//    q3 += qDot4 * (1.0f / sampleFreq);
//    
//    // Normalise quaternion
//    recipNorm = _InvSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
//    q0 *= recipNorm;
//    q1 *= recipNorm;
//    q2 *= recipNorm;
//    q3 *= recipNorm;
//}


/**
 * Fast inverse square root implementation
 * @see http://en.wikipedia.org/wiki/Fast_inverse_square_root
 */
float _InvSqrt(float nNumber)
{
    volatile long           i;
    volatile float          x, y;
    volatile const float    f = 1.5F;
    
    x = nNumber * 0.5F;
    y = nNumber;
    i = * ( long * ) &y;
    i = 0x5f375a86 - ( i >> 1 );
    y = * ( float * ) &i;
    y = y * ( f - ( x * y * y ) );
    
    return y;
}


inline void nRCInterrupt_CB0()
{
    if(!(pSelfFlyHndl->nInterruptLockFlag))
        pSelfFlyHndl->nRCCh[0] = micros() - pSelfFlyHndl->nRCPrevChangeTime[0];
    
    pSelfFlyHndl->nRCPrevChangeTime[0] = micros();
}


inline void nRCInterrupt_CB1()
{
    if(!(pSelfFlyHndl->nInterruptLockFlag))
        pSelfFlyHndl->nRCCh[1] = micros() - pSelfFlyHndl->nRCPrevChangeTime[1];
    
    pSelfFlyHndl->nRCPrevChangeTime[1] = micros();
}


inline void nRCInterrupt_CB2()
{
    if(!(pSelfFlyHndl->nInterruptLockFlag))
        pSelfFlyHndl->nRCCh[2] = micros() - pSelfFlyHndl->nRCPrevChangeTime[2];
    
    pSelfFlyHndl->nRCPrevChangeTime[2] = micros();
}


inline void nRCInterrupt_CB3()
{
    if(!(pSelfFlyHndl->nInterruptLockFlag))
        pSelfFlyHndl->nRCCh[3] = micros() - pSelfFlyHndl->nRCPrevChangeTime[3];
    
    pSelfFlyHndl->nRCPrevChangeTime[3] = micros();
}


inline void nRCInterrupt_CB4()
{
    if(!(pSelfFlyHndl->nInterruptLockFlag))
        pSelfFlyHndl->nRCCh[4] = micros() - pSelfFlyHndl->nRCPrevChangeTime[4];
    
    pSelfFlyHndl->nRCPrevChangeTime[4] = micros();
}


float Clip3Float(const float nValue, const int MIN, const int MAX)
{
    float               nClipVal = nValue;
    
    if(nValue < MIN)
        nClipVal = MIN;
    else if(nValue > MAX)
        nClipVal = MAX;
    
    return nClipVal;
}


int Clip3Int(const int nValue, const int MIN, const int MAX)
{
    int                 nClipVal = nValue;
    
    if(nValue < MIN)
        nClipVal = MIN;
    else if(nValue > MAX)
        nClipVal = MAX;
    
    return nClipVal;
}


inline void _AcquireLock()
{
    pSelfFlyHndl->nInterruptLockFlag = true;
}


inline void _ReleaseLock()
{
    pSelfFlyHndl->nInterruptLockFlag = false;
}


#if __DEBUG__
void _print_RC_Signals()
{
    float                   *pRCCh = &(pSelfFlyHndl->nRCCh[0]);
    
    Serialprint("   //   RC_Roll:");
    if(pRCCh[0] - 1480 < 0)Serialprint("<<<");
    else if(pRCCh[0] - 1520 > 0)Serialprint(">>>");
    else Serialprint("-+-");
    Serialprint(pRCCh[0]);
    
    Serialprint("   RC_Pitch:");
    if(pRCCh[1] - 1480 < 0)Serialprint("^^^");
    else if(pRCCh[1] - 1520 > 0)Serialprint("vvv");
    else Serialprint("-+-");
    Serialprint(pRCCh[1]);
    
    Serialprint("   RC_Throttle:");
    if(pRCCh[2] - 1480 < 0)Serialprint("vvv");
    else if(pRCCh[2] - 1520 > 0)Serialprint("^^^");
    else Serialprint("-+-");
    Serialprint(pRCCh[2]);
    
    Serialprint("   RC_Yaw:");
    if(pRCCh[3] - 1480 < 0)Serialprint("<<<");
    else if(pRCCh[3] - 1520 > 0)Serialprint(">>>");
    else Serialprint("-+-");
    Serialprint(pRCCh[3]);
    
    Serialprint("   RC_Gear:");
    if(pRCCh[4] - 1480 < 0)Serialprint("<<<");
    else if(pRCCh[4] - 1520 > 0)Serialprint(">>>");
    else Serialprint("-+-");
    Serialprint(pRCCh[4]);
}


void _print_Gyro_Signals()
{
    float                   *pFineAngle = &(pSelfFlyHndl->nAccelGyroParam.nFineAngle[0]);
    
//    Serialprint("   //    Roll Gyro : ");
//    Serialprint(pFineAngle[0]);
//    Serialprint("   Pitch Gyro : ");
//    Serialprint(pFineAngle[1]);
//    Serialprint("   Yaw Gyro : ");
//    Serialprint(pFineAngle[2]);
//    Serialprint("   Temp : ");
//    Serialprint(pSelfFlyHndl->nAccelGyroParam.nRawTemp/340.00 + 36.53);
    
    Serialprint("   Gx: ");
    Serialprint(pSelfFlyHndl->nAccelGyroParam.nRawGyro[0]);
    Serialprint("   Gy: ");
    Serialprint(pSelfFlyHndl->nAccelGyroParam.nRawGyro[1]);
    Serialprint("   Gz: ");
    Serialprint(pSelfFlyHndl->nAccelGyroParam.nRawGyro[2]);

    Serialprint("   Ax: ");
    Serialprint(pSelfFlyHndl->nAccelGyroParam.nRawAccel[0]);
    Serialprint("   Ay: ");
    Serialprint(pSelfFlyHndl->nAccelGyroParam.nRawAccel[1]);
    Serialprint("   Az: ");
    Serialprint(pSelfFlyHndl->nAccelGyroParam.nRawAccel[2]);
}


void _print_Throttle_Signals()
{
    int32_t                 *pThrottle = &(pSelfFlyHndl->nThrottle[0]);
    
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
    float                   *pFineRPY = &(pSelfFlyHndl->nFineRPY[0]);
    
    Serialprint("   //    Roll: ");
    Serialprint(pFineRPY[0]);
    Serialprint("   Pitch: ");
    Serialprint(pFineRPY[1]);
    Serialprint("   Yaw: ");
    Serialprint(pFineRPY[2]);
}

void _print_MagData()
{
    MagneticParam_T         *pMagParam = &(pSelfFlyHndl->nMagParam);
    
//    Serialprint("   //    Magnetic HEAD:"); Serialprint(pMagParam->nMagHeadingDeg);
//    Serialprint("   SmoothHEAD:"); Serialprint(pMagParam->nSmoothHeadingDegrees);
    
    Serialprint("   //   Mx:"); Serialprint(pMagParam->nRawMagData[0]);
    Serialprint("   My:"); Serialprint(pMagParam->nRawMagData[1]);
    Serialprint("   Mz:"); Serialprint(pMagParam->nRawMagData[2]);
}

void _print_BarometerData()
{
    BaroParam_T             *pBaroParam = &(pSelfFlyHndl->nBaroParam);
    
    Serialprint("   //    Barometer AvgTemp:"); Serialprint(pBaroParam->nAvgTemp);
    Serialprint("   AvgPress:"); Serialprint(pBaroParam->nAvgPressure);
    Serialprint("   AvgAlt:"); Serialprint(pBaroParam->nAvgAbsoluteAltitude);
    Serialprint("   RelativeAlt:"); Serialprint(pBaroParam->nRelativeAltitude);
    Serialprint("   VerticalSpeed:"); Serialprint(pBaroParam->nVerticalSpeed);
    Serialprint("   ");
}
#endif


