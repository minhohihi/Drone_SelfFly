/*----------------------------------------------------------------------------------------
 Constant Definitions
 ----------------------------------------------------------------------------------------*/
#define __DEBUG__                           (1)
#if (__DEBUG__)
    #define __PROFILE__                     (1)
#else
    #define __PROFILE__                     (0)
#endif
#define __GYROACCEL_DMP_ENABLED__           (0)

// Define Axis
#define X_AXIS                              (0)
#define Y_AXIS                              (1)
#define Z_AXIS                              (2)

// Define Max Number of Each Field
#define MAX_CH_RC                           (5)
#define MAX_CH_ESC                          (4)

// Arduino Pin configuration
#define PIN_GY86_EXT_INTERRUPT              (13)
#define PIN_RESERVED_12                     (12)
#define PIN_ESC_CH0                         (11)
#define PIN_ESC_CH1                         (10)
#define PIN_ESC_CH2                         (9)
#define PIN_ESC_CH3                         (8)
#define PIN_RESERVED_07                     (7)
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
#define RC_CH_HIGH0                         (1900)
#define RC_CH_LOW0                          (1050)
#define RC_CH_HIGH1                         (1900)
#define RC_CH_LOW1                          (1050)
#define RC_CH_HIGH2                         (1900)
#define RC_CH_LOW2                          (1050)
#define RC_CH_HIGH3                         (1900)
#define RC_CH_LOW3                          (1050)
#define RC_CH_HIGH4                         (1900)
#define RC_CH_LOW4                          (1050)

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
#define GYRO_FS                             (131.0f)                         // (2^15 - 1) / (250 * (1 << GYRO_FS_PRECISIOM))
#define ACCEL_FS_PRECISIOM                  (MPU6050_ACCEL_FS_2)            //  MPU6050_ACCEL_FS_4  MPU6050_ACCEL_FS_8  MPU6050_ACCEL_FS_16
#define ACCEL_STD_DENOM                     (16384.0f / (1 << ACCEL_FS_PRECISIOM))


#define ACC_OFFSET_X                        (205)
#define ACC_OFFSET_Y                        (-39)
#define ACC_OFFSET_Z                        (1063)
#define ACC_SCALE_X                         (2*7948.565970)
#define ACC_SCALE_Y                         (2*8305.469320)
#define ACC_SCALE_Z                         (2*8486.650841)

#define MAG_OFFSET_X                        (67.0f)
#define MAG_OFFSET_Y                        (-59.0f)
#define MAG_OFFSET_Z                        (26.0f)
#define MAG_SCALE_X                         (527.652115f)
#define MAG_SCALE_Y                         (569.016790f)
#define MAG_SCALE_Z                         (514.710857f)

// Min & Max Val for Magnetic
#define MAG_X_MIN                           (-270)                          //-654  -693   -688
#define MAG_X_MAX                           (585)                           //185   209    170
#define MAG_X_RANGE                         (MAG_X_MAX - MAG_X_MIN)
#define MAG_Y_MIN                           (-600)                          //-319  -311   -310
#define MAG_Y_MAX                           (260)                           //513   563    546
#define MAG_Y_RANGE                         (MAG_Y_MAX - MAG_Y_MIN)
#define MAG_Z_MIN                           (-425)                          // -363  -374   -377
#define MAG_Z_MAX                           (285)                           // 386   429    502
#define MAG_Z_RANGE                         (MAG_Z_MAX - MAG_Z_MIN)

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

#define ROUNDING_BASE                       (50)
#define SAMPLING_TIME                       (0.01)                          // Seconds

#define TWO_KP_DEF                          (2.0f * 0.5f)                   // 2 * proportional gain
#define TWO_KI_DEF                          (2.0f * 0.1f)                   // 2 * integral gain

#define RAD_TO_DEG_SCALE                    (57.2958f)                      // = 180 / PI
#define DEG_TO_RAD_SCALE                    (0.0175f)                       // = PI / 180
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
    float               nRelativeAltitude;              // Relative Absolute Altitude Data
    float               nPrevAvgAbsoluteAltitude;               // Average Absolute Altitude Data
    float               nRefAbsoluteAltitude;                   // Reference Absolute Altitude Data
    float               nVerticalSpeed;                         // Estimated Vertical Speed
}BaroParam_T;

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
    
    AxisErrRate_T       nPitch;
    AxisErrRate_T       nRoll;
    AxisErrRate_T       nYaw;
    float               nRCCh[MAX_CH_RC];                       // RC channel inputs
    unsigned long       nRCPrevChangeTime[MAX_CH_RC];
    
    int32_t             nThrottle[MAX_CH_ESC];
    
    float               nFineRPY[3];
    float               nFineGyro[3];
    volatile float      nQuaternion[4];                              // quaternion
    float               nEstGravity[3];                              // estimated gravity direction
    
    unsigned long       nCurrSensorCapTime;
    unsigned long       nPrevSensorCapTime;
    float               nDiffTime;
    float               nSampleFreq;                            // half the sample period expressed in seconds
}SelfFly_T;


/*----------------------------------------------------------------------------------------
 Static Function
 ----------------------------------------------------------------------------------------*/
int _AccelGyro_Initialize();
void _AccelGyro_GetData();
void _AccelGyro_CalculateAngle();
void _AccelGyro_Calibrate();
#if __GYROACCEL_DMP_ENABLED__
int _AccelGyro_GetDMPData();
inline void dmpDataReady();
#endif
int _Mag_Initialize();
void _Mag_GetData();
void _Mag_CalculateDirection();
float _Mag_TiltCompensate(float *nRawMagData, float *pAccelGyro);
int _Barometer_Initialize();
void _Barometer_GetData();
void _Barometer_CalculateData();
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
volatile bool           nMPUInterruptFlag = false;
bool                    nInterruptLockFlag = false;             // Interrupt lock

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
    
    // Initialize RemoteController
    _RC_Initialize();
    
    // Initialize ESCs
    _ESC_Initialize();
    
    for(i=0 ; i<MAX_CH_RC ; i++)
        pSelfFlyHndl->nRCPrevChangeTime[i] = micros();
    
    pSelfFlyHndl->nQuaternion[0] = 1.0f;

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
    
    #if 0
    // Get AccelGyro & Magnetic & Barometer Sensor Value
    _GetSensorRawData();

    // Calculate Raw Data
    _AccelGyro_CalculateAngle();
    _Mag_CalculateDirection();
    _Barometer_CalculateData();
    #else
    _Get_RollPitchYaw();
    #endif

    // PID Computation
    _CalculatePID();
    
    // Throttle Calculation
    CalculateThrottleVal();
    
    // Update BLDCs
    UpdateESCs();
    
    delay(50);
    
    #if __DEBUG__
    //_print_Gyro_Signals();
    //_print_MagData();
    //_print_BarometerData();
    //_print_RPY_Signals();
    //_print_RC_Signals();
    //_print_Throttle_Signals();
    #endif

    #if __PROFILE__
    nEndTime = micros();
    //Serialprint(" Loop Duration: "); Serialprintln((nEndTime - nStartTime0)/1000);
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
    
    #if __GYROACCEL_DMP_ENABLED__
    {
        uint8_t                 nDevStatus;                         // return status after each device operation (0 = success, !0 = error)
        
        // load and configure the DMP
        Serialprintln(F("    Initializing DMP..."));
        nDevStatus = pSelfFlyHndl->nAccelGyroHndl.dmpInitialize();
        
        // make sure it worked (returns 0 if so)
        if(0 == nDevStatus)
        {
            // turn on the DMP, now that it's ready
            Serialprintln(F("        Enabling DMP..."));
            pSelfFlyHndl->nAccelGyroHndl.setDMPEnabled(true);
            
            // enable Arduino interrupt detection
            Serialprintln(F("            Enabling interrupt detection (Arduino external interrupt 0)..."));
            pinMode(PIN_GY86_EXT_INTERRUPT, INPUT);
            digitalWrite(PIN_GY86_EXT_INTERRUPT, HIGH);
            PCintPort::attachInterrupt(PIN_GY86_EXT_INTERRUPT, dmpDataReady, RISING);
            
            // set our DMP Ready flag so the main loop() function knows it's okay to use it
            Serialprintln(F("    Done... DMP ready! Waiting for first interrupt..."));
            nDMPReadyFlag = true;
            
            // get expected DMP packet size for later comparison
            nPacketSize = pSelfFlyHndl->nAccelGyroHndl.dmpGetFIFOPacketSize();
        }
        else
        {
            // ERROR!
            // 1 = initial memory load failed
            // 2 = DMP configuration updates failed
            // (if it's going to break, usually the code will be 1)
            Serialprint(F("     DMP Initialization failed (code "));
            Serialprint(nDevStatus);
            Serialprintln(F(")"));
        }
    }
    #endif
    
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
    float                   nRawAngle[3] = {0, };
    static float            nPrevRawAngle[3] = {0, };
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
    {
        nGyroAngle[X_AXIS] = nGyroDiffAngle[X_AXIS] + pFineAngle[X_AXIS];
        nGyroAngle[Y_AXIS] = nGyroDiffAngle[Y_AXIS] + pFineAngle[Y_AXIS];
        nGyroAngle[Z_AXIS] = nGyroDiffAngle[Z_AXIS] + pFineAngle[Z_AXIS];
        
        // Compute the drifting gyro angles
        //nRawAngle[X_AXIS] = nGyroDiffAngle[X_AXIS] + nPrevRawAngle[X_AXIS];
        //nRawAngle[Y_AXIS] = nGyroDiffAngle[Y_AXIS] + nPrevRawAngle[Y_AXIS];
        //nRawAngle[Z_AXIS] = nGyroDiffAngle[Z_AXIS] + nPrevRawAngle[Z_AXIS];
    }
    
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


#if __GYROACCEL_DMP_ENABLED__
VectorFloat nGravity;
Quaternion nQuater;

int _AccelGyro_GetDMPData()
{
    uint16_t                nFIFOCnt = 0;                                               // count of all bytes currently in FIFO
    uint8_t                 nFIFOBuf[64];                                               // FIFO storage buffer
    uint8_t                 nMPUInterruptStat;                                          // holds actual interrupt status byte from MPU
    AccelGyroParam_T        *pAccelGyroParam = &(pSelfFlyHndl->nAccelGyroParam);
    
    if(!nDMPReadyFlag)
    {
        Serialprintln("DMP Not Ready    Loop Run Fails");
        return -1;
    }
    
    //while(!nMPUInterruptFlag && nFIFOCnt < nPacketSize)
    {
        // Wait Till Interrupt Up!!
    }
    
    // reset interrupt flag and get INT_STATUS byte
    nMPUInterruptFlag = false;
    nMPUInterruptStat = pSelfFlyHndl->nAccelGyroHndl.getIntStatus();
    
    // get current FIFO count
    nFIFOCnt = pSelfFlyHndl->nAccelGyroHndl.getFIFOCount();
    
    // check for overflow (this should never happen unless our code is too inefficient)
    if((nMPUInterruptStat & 0x10) || nFIFOCnt == 1024)
    {
        // reset so we can continue cleanly
        pSelfFlyHndl->nAccelGyroHndl.resetFIFO();
        //Serialprintln(F("FIFO overflow!"));
    }
    else if(nMPUInterruptStat & 0x02)
    {
        int16_t             nGyro[3];
        
        // wait for correct available data length, should be a VERY short wait
        while (nFIFOCnt < nPacketSize)
            nFIFOCnt = pSelfFlyHndl->nAccelGyroHndl.getFIFOCount();
        
        // read a packet from FIFO
        pSelfFlyHndl->nAccelGyroHndl.getFIFOBytes(nFIFOBuf, nPacketSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        nFIFOCnt -= nPacketSize;
        
        pSelfFlyHndl->nAccelGyroHndl.dmpGetGyro(nGyro, nFIFOBuf);
        pSelfFlyHndl->nAccelGyroHndl.dmpGetQuaternion(&nQuater, nFIFOBuf);
        pSelfFlyHndl->nAccelGyroHndl.dmpGetGravity(&nGravity, &nQuater);
        pSelfFlyHndl->nAccelGyroHndl.dmpGetYawPitchRoll(pSelfFlyHndl->nFineRPY, &nQuater, &nGravity);
        
        pAccelGyroParam->nFineAngle[X_AXIS] = nGyro[X_AXIS];
        pAccelGyroParam->nFineAngle[Y_AXIS] = nGyro[Y_AXIS];
        pAccelGyroParam->nFineAngle[Z_AXIS] = nGyro[Z_AXIS];
    }
    
    return 0;
}


inline void dmpDataReady()
{
    nMPUInterruptFlag = true;
}
#endif


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
    pSelfFlyHndl->nMagParam.nDeclinationAngle = (7.0 + (59.76 / 60.0)) / RAD_TO_DEG_SCALE;
    
    Serialprintln(F(" Done"));
    
    // Reference WebSite
    // http://www.meccanismocomplesso.org/en/arduino-magnetic-magnetic-magnetometer-hmc5883l/
}


void _Mag_GetData()
{
    int16_t                 nRawMagData[3];
    float         *pRawMagData = &(pSelfFlyHndl->nMagParam.nRawMagData[X_AXIS]);
    
    //pSelfFlyHndl->nMagHndl.getHeading(&(pRawMagData[X_AXIS]), &(pRawMagData[Y_AXIS]), &(pRawMagData[Z_AXIS]));
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
    
//    if(pMagParam->nMagHeadingDeg >= 1 && pMagParam->nMagHeadingDeg < 240)
//        pMagParam->nMagHeadingDeg = map(pMagParam->nMagHeadingDeg, 0, 239, 0, 179);
//    else if(pMagParam->nMagHeadingDeg >= 240)
//        pMagParam->nMagHeadingDeg = map(pMagParam->nMagHeadingDeg, 240, 360, 180, 360);
    
    // Smooth angles rotation for +/- 3deg
    pMagParam->nSmoothHeadingDegrees = round(pMagParam->nMagHeadingDeg);
    
    if((pMagParam->nSmoothHeadingDegrees < (pMagParam->nPrevHeadingDegrees + 3)) &&
       (pMagParam->nSmoothHeadingDegrees > (pMagParam->nPrevHeadingDegrees - 3)))
        pMagParam->nSmoothHeadingDegrees = pMagParam->nPrevHeadingDegrees;
    
    pMagParam->nPrevHeadingDegrees = pMagParam->nSmoothHeadingDegrees;
}


float _Mag_TiltCompensate(float *nRawMagData, float *pAccelGyro)
{
    AccelGyroParam_T        *pAccelGyroParam = &(pSelfFlyHndl->nAccelGyroParam);
    MagneticParam_T         *pMagParam = &(pSelfFlyHndl->nMagParam);
    
    //    float roll;
    //    float pitch;
    //
    //    roll = asin(pAccelGyroParam->nRawAccel);
    //    pitch = asin(-normAccel.XAxis);
    //
    //    if (roll > 0.78 || roll < -0.78 || pitch > 0.78 || pitch < -0.78)
    //    {
    //        return -1000;
    //    }
    //
    //    // Some of these are used twice, so rather than computing them twice in the algorithem we precompute them before hand.
    //    float cosRoll = cos(roll);
    //    float sinRoll = sin(roll);
    //    float cosPitch = cos(pitch);
    //    float sinPitch = sin(pitch);
    //
    //    // Tilt compensation
    //    float Xh = mag.XAxis * cosPitch + mag.ZAxis * sinPitch;
    //    float Yh = mag.XAxis * sinRoll * sinPitch + mag.YAxis * cosRoll - mag.ZAxis * sinRoll * cosPitch;
    //
    //    float heading = atan2(Yh, Xh);
    //    
    //    return heading;
    return 0.0f;
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
    
    _AcquireLock();
    
    pRCCh[0] = floor(pRCCh[0] / ROUNDING_BASE) * ROUNDING_BASE;
    pRCCh[1] = floor(pRCCh[1] / ROUNDING_BASE) * ROUNDING_BASE;
    pRCCh[3] = floor(pRCCh[3] / ROUNDING_BASE) * ROUNDING_BASE;
    
    pRCCh[0] = map(pRCCh[0], RC_CH_LOW0, RC_CH_HIGH0, ROLL_ANG_MIN, ROLL_ANG_MAX) - 1;
    pRCCh[1] = map(pRCCh[1], RC_CH_LOW1, RC_CH_HIGH1, PITCH_ANG_MIN, PITCH_ANG_MAX) - 1;
    pRCCh[3] = map(pRCCh[3], RC_CH_LOW3, RC_CH_HIGH3, YAW_RATE_MIN, YAW_RATE_MAX);
    
    if((pRCCh[0] < ROLL_ANG_MIN) || (pRCCh[0] > ROLL_ANG_MAX))
        pRCCh[0] = nRCPrevCh[0];
    if((pRCCh[1] < PITCH_ANG_MIN) || (pRCCh[1] > PITCH_ANG_MAX))
        pRCCh[1] = nRCPrevCh[1];
    if((pRCCh[3] < YAW_RATE_MIN) || (pRCCh[3] > YAW_RATE_MAX))
        pRCCh[3] = nRCPrevCh[3];
    
    nRCPrevCh[0] = pRCCh[0];
    nRCPrevCh[1] = pRCCh[1];
    nRCPrevCh[3] = pRCCh[3];
    
    pFineRPY[1] = pFineRPY[1] * RAD_TO_DEG_SCALE + PITCH_ANG_OFFSET;
    pFineRPY[2] = pFineRPY[2] * RAD_TO_DEG_SCALE + ROLL_ANG_OFFSET;
    
    if(abs(pFineRPY[1] - nPrevFineRPY[1]) > 30)
        pFineRPY[1] = nPrevFineRPY[1];
    
    if(abs(pFineRPY[2] - nPrevFineRPY[2]) > 30)
        pFineRPY[2] = nPrevFineRPY[2];
    
    //ROLL control
    pRoll->nAngleErr = pRCCh[0] - pFineRPY[2];
    pRoll->nCurrErrRate = pRoll->nAngleErr * ROLL_OUTER_P_GAIN - pFineGyro[0];
    pRoll->nP_ErrRate = pRoll->nCurrErrRate * ROLL_INNER_P_GAIN;
    pRoll->nI_ErrRate = Clip3Float((pRoll->nI_ErrRate + (pRoll->nCurrErrRate * ROLL_INNER_I_GAIN) * SAMPLING_TIME), -100, 100);
    pRoll->nD_ErrRate = (pRoll->nCurrErrRate - pRoll->nPrevErrRate) * ROLL_INNER_D_GAIN / SAMPLING_TIME;
    pRoll->nBalance = pRoll->nP_ErrRate + pRoll->nI_ErrRate + pRoll->nD_ErrRate;
    
    //PITCH control
    pPitch->nAngleErr = pRCCh[1] - pFineRPY[1];
    pPitch->nCurrErrRate = pPitch->nAngleErr * PITCH_OUTER_P_GAIN + pFineGyro[1];
    pPitch->nP_ErrRate = pPitch->nCurrErrRate * PITCH_INNER_P_GAIN;
    pPitch->nI_ErrRate = Clip3Float((pPitch->nI_ErrRate + (pPitch->nCurrErrRate * PITCH_INNER_I_GAIN) * SAMPLING_TIME), -100, 100);
    pPitch->nD_ErrRate = (pPitch->nCurrErrRate - pPitch->nPrevErrRate) * PITCH_INNER_D_GAIN / SAMPLING_TIME;
    pPitch->nBalance = pPitch->nP_ErrRate + pPitch->nI_ErrRate + pPitch->nD_ErrRate;
    
    //YAW control
    pYaw->nCurrErrRate = pRCCh[3] - pFineGyro[2];
    pYaw->nP_ErrRate = pYaw->nCurrErrRate * YAW_P_GAIN;
    pYaw->nI_ErrRate = Clip3Float((pYaw->nI_ErrRate + pYaw->nCurrErrRate * YAW_I_GAIN * SAMPLING_TIME), -50, 50);
    pYaw->nTorque = pYaw->nP_ErrRate + pYaw->nI_ErrRate;
    
    // Backup for Next
    pRoll->nPrevErrRate = pRoll->nCurrErrRate;
    pPitch->nPrevErrRate = pPitch->nCurrErrRate;
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
    nEstimatedThrottle = (float)(map(pRCCh[2], RC_CH_LOW2, RC_CH_HIGH2, ESC_MIN, ESC_MAX));
    
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
    pSelfFlyHndl->nSampleFreq = 1.0 / ((pSelfFlyHndl->nCurrSensorCapTime - pSelfFlyHndl->nPrevSensorCapTime) / 1000000.0);

    // Get AccelGyro Raw Data
    _AccelGyro_GetData();
    
    // Get Magnetic Raw Data
    _Mag_GetData();
    
    // Get Barometer Raw Data
    _Barometer_GetData();

    // Normalize Sensor Values
    //_Normailize_SensorVal();

    if(1)
    {
        float           *pRawGyro = &(pSelfFlyHndl->nAccelGyroParam.nRawGyro[X_AXIS]);
        float           *pRawAccel = &(pSelfFlyHndl->nAccelGyroParam.nRawAccel[X_AXIS]);
        float           *pRawMagData = &(pSelfFlyHndl->nMagParam.nRawMagData[X_AXIS]);
        float           nSensorVal[9] = {0, };

        nSensorVal[0] = pRawGyro[X_AXIS];
        nSensorVal[1] = pRawGyro[Y_AXIS];
        nSensorVal[2] = pRawGyro[Z_AXIS];
        nSensorVal[3] = (pRawAccel[X_AXIS]);
        nSensorVal[4] = (pRawAccel[Y_AXIS]);
        nSensorVal[5] = (pRawAccel[Z_AXIS]);
        nSensorVal[6] = (pRawMagData[X_AXIS]);
        nSensorVal[7] = (pRawMagData[Y_AXIS]);
        nSensorVal[8] = (pRawMagData[Z_AXIS]);
        
        //Serialprint(F("DEL:"));
        //Serialprint(nSensorVal[0], DEC);
        //Serialprint(F("#GYR:"));
        Serialprint(nSensorVal[0], DEC); Serialprint(F(","));
        Serialprint(nSensorVal[1], DEC); Serialprint(F(","));
        Serialprint(nSensorVal[2], DEC); Serialprint(F(","));
        //Serialprint(F("#ACC:"));
        Serialprint(nSensorVal[3], DEC); Serialprint(F(","));
        Serialprint(nSensorVal[4], DEC); Serialprint(F(","));
        Serialprint(nSensorVal[5], DEC); Serialprint(F(","));
        //Serialprint(F("#MAG:"));
        Serialprint(nSensorVal[6], DEC); Serialprint(F(","));
        Serialprint(nSensorVal[7], DEC); Serialprint(F(","));
        Serialprintln(nSensorVal[8], DEC);
    }
}


void _Normailize_SensorVal()
{
    float                   *pRawGyro = &(pSelfFlyHndl->nAccelGyroParam.nRawGyro[X_AXIS]);
    float                   *pRawAccel = &(pSelfFlyHndl->nAccelGyroParam.nRawAccel[X_AXIS]);
    float                   *pRawMagData = &(pSelfFlyHndl->nMagParam.nRawMagData[X_AXIS]);
    
    //pRawGyro[X_AXIS] = pRawGyro[X_AXIS] / GYRO_FS * DEG_TO_RAD_SCALE;
    //pRawGyro[Y_AXIS] = pRawGyro[Y_AXIS] / GYRO_FS * DEG_TO_RAD_SCALE;
    //pRawGyro[Z_AXIS] = pRawGyro[Z_AXIS] / GYRO_FS * DEG_TO_RAD_SCALE;
    //pRawAccel[X_AXIS] = (pRawAccel[X_AXIS]) / 16383.0f * 1.1;
    //pRawAccel[Y_AXIS] = (pRawAccel[Y_AXIS]) / 16383.0f * 1.1;
    //pRawAccel[Z_AXIS] = (pRawAccel[Z_AXIS]) / 16383.0f * 1.1;
    //pRawMagData[X_AXIS] = (pRawMagData[X_AXIS]) / MAG_SCALE_X;
    //pRawMagData[Y_AXIS] = (pRawMagData[Y_AXIS]) / MAG_SCALE_Y;
    //pRawMagData[Z_AXIS] = (pRawMagData[Z_AXIS]) / MAG_SCALE_Z;
}


void _Get_RollPitchYaw()
{
    // Calculate Roll & Pitch & Yaw
    _Get_RollPitchYawRad_By_Q();
    
    float                   *pFineG = &(pSelfFlyHndl->nEstGravity[X_AXIS]);
    const volatile float    *pQ = &(pSelfFlyHndl->nQuaternion[0]);
    float                   *pFineRPY = &(pSelfFlyHndl->nFineRPY[0]);
    
    // Convert Radian to Degree
    pFineRPY[0] *= RAD_TO_DEG_SCALE;
    pFineRPY[1] *= RAD_TO_DEG_SCALE;
    pFineRPY[2] *= RAD_TO_DEG_SCALE;
    
    //Serialprint("   Y_:"); Serialprint(pFineRPY[0]);
    //Serialprint("   P_:"); Serialprint(pFineRPY[1]);
    //Serialprint("   R_:"); Serialprint(pFineRPY[2]);
}


void _Get_Quaternion()
{
    float                   *pRawGyro = &(pSelfFlyHndl->nAccelGyroParam.nRawGyro[X_AXIS]);
    float                   *pRawAccel = &(pSelfFlyHndl->nAccelGyroParam.nRawAccel[X_AXIS]);
    float                   *pRawMagData = &(pSelfFlyHndl->nMagParam.nRawMagData[X_AXIS]);

    // Get Sensor (Gyro / Accel / Megnetic / Baro / Temp)
    _GetSensorRawData();
    
    _AHRSupdate(pRawGyro[X_AXIS], pRawGyro[Y_AXIS], pRawGyro[Z_AXIS],
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

    pEstGravity[X_AXIS] = 2 * ((pQ[1] * pQ[3]) - (pQ[0] * pQ[2]));
    pEstGravity[Y_AXIS] = 2 * ((pQ[0] * pQ[1]) + (pQ[2] * pQ[3]));
    pEstGravity[Z_AXIS] = (pQ[0] * pQ[0]) - (pQ[1] * pQ[1]) - (pQ[2] * pQ[2]) + (pQ[3] * pQ[3]);
    
    pFineRPY[0] = atan2((2 * pQ[1] * pQ[2]) - (2 * pQ[0] * pQ[3]), (2 * pQ[0] * pQ[0]) + (2 * pQ[1] * pQ[1]) - 1);
    pFineRPY[1] = atan(pEstGravity[X_AXIS] / sqrt((pEstGravity[Y_AXIS] * pEstGravity[Y_AXIS]) + (pEstGravity[Z_AXIS] * pEstGravity[Z_AXIS])));
    pFineRPY[2] = atan(pEstGravity[Y_AXIS] / sqrt((pEstGravity[X_AXIS] * pEstGravity[X_AXIS]) + (pEstGravity[Z_AXIS] * pEstGravity[Z_AXIS])));
}


void _AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
{
    float                   nRecipNorm = 0.0f;
    float                   nQ00, nQ01, nQ02, nQ03, nQ11, nQ12, nQ13, nQ22, nQ23, nQ33;
    float                   halfex = 0.0f, halfey = 0.0f, halfez = 0.0f;
    float                   qa, qb, qc;
    static volatile float   nIntegralFBx = 0.0f,  nIntegralFBy = 0.0f, nIntegralFBz = 0.0f;
    const float             nSampleFreq = pSelfFlyHndl->nSampleFreq;
    volatile float          *pQ = &(pSelfFlyHndl->nQuaternion[0]);
    
    // Auxiliary variables to avoid repeated arithmetic
    nQ00 = pQ[0] * pQ[0];
    nQ01 = pQ[0] * pQ[1];
    nQ02 = pQ[0] * pQ[2];
    nQ03 = pQ[0] * pQ[3];
    nQ11 = pQ[1] * pQ[1];
    nQ12 = pQ[1] * pQ[2];
    nQ13 = pQ[1] * pQ[3];
    nQ22 = pQ[2] * pQ[2];
    nQ23 = pQ[2] * pQ[3];
    nQ33 = pQ[3] * pQ[3];
    
    if((mx != 0.0f) && (my != 0.0f) && (mz != 0.0f))
    {
        float hx, hy, bx, bz;
        float halfwx, halfwy, halfwz;
        
        // Normalise magnetometer measurement
        nRecipNorm = 1.0 / sqrt(mx * mx + my * my + mz * mz);//_InvSqrt(mx * mx + my * my + mz * mz);
        mx *= nRecipNorm;
        my *= nRecipNorm;
        mz *= nRecipNorm;
        
        // Reference direction of Earth's magnetic field
        hx = 2.0f * (mx * (0.5f - nQ22 - nQ33) + my * (nQ12 - nQ03) + mz * (nQ13 + nQ02));
        hy = 2.0f * (mx * (nQ12 + nQ03) + my * (0.5f - nQ11 - nQ33) + mz * (nQ23 - nQ01));
        bx = sqrt(hx * hx + hy * hy);
        bz = 2.0f * (mx * (nQ13 - nQ02) + my * (nQ23 + nQ01) + mz * (0.5f - nQ11 - nQ22));
        
        // Estimated direction of magnetic field
        halfwx = bx * (0.5f - nQ22 - nQ33) + bz * (nQ13 - nQ02);
        halfwy = bx * (nQ12 - nQ03) + bz * (nQ01 + nQ23);
        halfwz = bx * (nQ02 + nQ13) + bz * (0.5f - nQ11 - nQ22);
        
        // Error is sum of cross product between estimated direction and measured direction of field vectors
        halfex = (my * halfwz - mz * halfwy);
        halfey = (mz * halfwx - mx * halfwz);
        halfez = (mx * halfwy - my * halfwx);
    }
    
    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if((ax != 0.0f) && (ay != 0.0f) && (az != 0.0f))
    {
        float halfvx, halfvy, halfvz;
        
        // Normalise accelerometer measurement
        nRecipNorm = 1.0f / sqrt(ax * ax + ay * ay + az * az);//_InvSqrt(ax * ax + ay * ay + az * az);
        ax *= nRecipNorm;
        ay *= nRecipNorm;
        az *= nRecipNorm;
        
        // Estimated direction of gravity
        halfvx = nQ13 - nQ02;
        halfvy = nQ01 + nQ23;
        halfvz = nQ00 + nQ33 - 0.5f;
        
        // Error is sum of cross product between estimated direction and measured direction of field vectors
        halfex += (ay * halfvz - az * halfvy);
        halfey += (az * halfvx - ax * halfvz);
        halfez += (ax * halfvy - ay * halfvx);
    }
    
    // Apply feedback only when valid data has been gathered from the accelerometer or magnetometer
    if(halfex != 0.0f && halfey != 0.0f && halfez != 0.0f)
    {
        // Compute and apply integral feedback if enabled
        if(TWO_KI_DEF > 0.0f)
        {
            nIntegralFBx += TWO_KI_DEF * halfex * (1.0f / nSampleFreq);  // integral error scaled by Ki
            nIntegralFBy += TWO_KI_DEF * halfey * (1.0f / nSampleFreq);
            nIntegralFBz += TWO_KI_DEF * halfez * (1.0f / nSampleFreq);
            gx += nIntegralFBx;  // apply integral feedback
            gy += nIntegralFBy;
            gz += nIntegralFBz;
        }
        else
        {
            nIntegralFBx = 0.0f; // prevent integral windup
            nIntegralFBy = 0.0f;
            nIntegralFBz = 0.0f;
        }
        
        // Apply proportional feedback
        gx += TWO_KP_DEF * halfex;
        gy += TWO_KP_DEF * halfey;
        gz += TWO_KP_DEF * halfez;
    }
    
    // Integrate rate of change of quaternion
    gx *= (0.5f * (1.0f / nSampleFreq));   // pre-multiply common factors
    gy *= (0.5f * (1.0f / nSampleFreq));
    gz *= (0.5f * (1.0f / nSampleFreq));
    
    qa = pQ[0];
    qb = pQ[1];
    qc = pQ[2];
    
    pQ[0] += ((-qb * gx) - (qc * gy) - (pQ[3] * gz));
    pQ[1] += (( qa * gx) + (qc * gz) - (pQ[3] * gy));
    pQ[2] += (( qa * gy) - (qb * gz) + (pQ[3] * gx));
    pQ[3] += (( qa * gz) + (qb * gy) - (qc * gx));
    
    // Normalise quaternion
    nRecipNorm = 1.0f / sqrt((pQ[0] * pQ[0]) + (pQ[1] * pQ[1]) + (pQ[2] * pQ[2]) + (pQ[3] * pQ[3]));//_InvSqrt((pQ[0] * pQ[0]) + (pQ[1] * pQ[1]) + (pQ[2] * pQ[2]) + (pQ[3] * pQ[3]));
    pQ[0] *= nRecipNorm;
    pQ[1] *= nRecipNorm;
    pQ[2] *= nRecipNorm;
    pQ[3] *= nRecipNorm;
}


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
    if(!nInterruptLockFlag)
        pSelfFlyHndl->nRCCh[0] = micros() - pSelfFlyHndl->nRCPrevChangeTime[0];
    
    pSelfFlyHndl->nRCPrevChangeTime[0] = micros();
}


inline void nRCInterrupt_CB1()
{
    if(!nInterruptLockFlag)
        pSelfFlyHndl->nRCCh[1] = micros() - pSelfFlyHndl->nRCPrevChangeTime[1];
    
    pSelfFlyHndl->nRCPrevChangeTime[1] = micros();
}


inline void nRCInterrupt_CB2()
{
    if(!nInterruptLockFlag)
        pSelfFlyHndl->nRCCh[2] = micros() - pSelfFlyHndl->nRCPrevChangeTime[2];
    
    pSelfFlyHndl->nRCPrevChangeTime[2] = micros();
}


inline void nRCInterrupt_CB3()
{
    if(!nInterruptLockFlag)
        pSelfFlyHndl->nRCCh[3] = micros() - pSelfFlyHndl->nRCPrevChangeTime[3];
    
    pSelfFlyHndl->nRCPrevChangeTime[3] = micros();
}


inline void nRCInterrupt_CB4()
{
    if(!nInterruptLockFlag)
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
    nInterruptLockFlag = true;
}


inline void _ReleaseLock()
{
    nInterruptLockFlag = false;
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
    Serialprint(pFineRPY[2]);
    Serialprint("   Pitch: ");
    Serialprint(pFineRPY[1]);
    Serialprint("   Yaw: ");
    Serialprint(pFineRPY[0]);
}

void _print_MagData()
{
    MagneticParam_T         *pMagParam = &(pSelfFlyHndl->nMagParam);
    
//    Serialprint("   //    Magnetic HEAD:"); Serialprint(pMagParam->nMagHeadingDeg);
//    Serialprint("   SmoothHEAD:"); Serialprint(pMagParam->nSmoothHeadingDegrees);
    
    Serialprint("   Mx:"); Serialprint(pMagParam->nRawMagData[0]);
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

