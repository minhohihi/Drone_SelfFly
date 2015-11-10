/*----------------------------------------------------------------------------------------
 Constant Definitions
 ----------------------------------------------------------------------------------------*/
#define __DEBUG__                           (1)
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
#define PITCH_OUTER_P_GAIN                  (4.750)             // angle control
#define PITCH_INNER_P_GAIN                  (2.933)             // rate control
#define PITCH_INNER_I_GAIN                  (0.440)
#define PITCH_INNER_D_GAIN                  (0.335)
#define ROLL_OUTER_P_GAIN                   (4.750)             // angle control
#define ROLL_INNER_P_GAIN                   (2.833)             // rate control
#define ROLL_INNER_I_GAIN                   (0.440)
#define ROLL_INNER_D_GAIN                   (0.335)
#define YAW_P_GAIN                          (2.325)             // yaw -> rate control
#define YAW_I_GAIN                          (0.650)

#define GYRO_FS_PRECISIOM                   (MPU6050_GYRO_FS_250)
#define ACCEL_FS_PRECISIOM                  (MPU6050_ACCEL_FS_2)    //  MPU6050_ACCEL_FS_4  MPU6050_ACCEL_FS_8  MPU6050_ACCEL_FS_16
#define ACCEL_STD_DENOM                     (16384.0f / (1 << ACCEL_FS_PRECISIOM))

// Min & Max Val for Magnetic
#define MAG_X_MIN                           (-270)    //-654  -693   -688
#define MAG_X_MAX                           (585)     //185   209    170
#define MAG_X_RANGE                         (MAG_X_MAX - MAG_X_MIN)
#define MAG_Y_MIN                           (-600)    //-319  -311   -310
#define MAG_Y_MAX                           (260)     //513   563    546
#define MAG_Y_RANGE                         (MAG_Y_MAX - MAG_Y_MIN)
#define MAG_Z_MIN                           (-425)    //-363  -374   -377
#define MAG_Z_MAX                           (285)     //386   429    502
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
#define SAMPLING_TIME                       (0.01)              // seconds

#define TWO_KP_DEF                          (2.0f * 0.5f) // 2 * proportional gain
#define TWO_KI_DEF                          (2.0f * 0.1f) // 2 * integral gain

#define RAD_TO_DEG_SCALE                    (57.2958f)          // = 180 / PI
#define DEG_TO_RAD_SCALE                    (0.0175f)           // = PI / 180
#define DOUBLE_RADIAN                       (6.283184)          // = 2 * PI

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
    int16_t             nRawGyro[3];
    int16_t             nRawAccel[3];
    float               nBaseGyro[3];
    float               nBaseAccel[3];
    float               nCurrReadTime;
    float               nPrevReadTime;
    float               nFineAngle[3];                          // Filtered Angles
    float               nFineRPY[3];                            // yaw pitch roll values
}AccelGyroParam_T;

typedef struct _MagParam_T
{
    byte                nRawBits[6];
    //Vector              nRawMagData;
    //Vector              nNormMagData;
    float               nRawMagData[3];
    float               nNormMagData[3];
    float               nFineMag[3];
    float               nMagHeadingRad;
    float               nMagHeadingDeg;
}MagneticParam_T;

typedef struct _BaroParam_T
{
    uint32_t            nRawTemp;                               // Raw Temperature Data
    uint32_t            nRawPressure;                           // Raw Pressure Data
    double              nRealTemperature;                       // Real Temperature Data
    float               nRealPressure;                          // Real Pressure Data
    float               nAvgpressure;                           // Average Pressure Data
    float               nAbsoluteAltitude;                      // Estimated Absolute Altitude
    float               nRelativeAltitude;                      // Estimated Relative Altitude
    double              nRefTemperature;                        // Reference Temperature Value
    int32_t             nRefPressure;                           // Reference Pressure Value
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
    float               nFineQ[4];                              // quaternion
    float               nFineG[3];                              // estimated gravity direction
    
    unsigned long       nLatestSensorCapTime;
    unsigned long       nPrevSensorCapTime;
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
#endif
int _Mag_Initialize();
void _Mag_Start();
void _Mag_GetData();
void _Mag_CalculateDirection();
void _Mag_Calibrate();
int _Barometer_Initialize();
void _Barometer_GetData();
inline void UpdateESCs();
inline void nRCInterrupt_CB0();
inline void nRCInterrupt_CB1();
inline void nRCInterrupt_CB2();
inline void nRCInterrupt_CB3();
inline void nRCInterrupt_CB4();
inline void dmpDataReady();
float Clip3Float(const float nValue, const int MIN, const int MAX);
int Clip3Int(const int nValue, const int MIN, const int MAX);
inline void _AcquireLock();
inline void _ReleaseLock();
inline void arm();
void _ESC_Initialize();
void _RC_Initialize();
void _AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void _GetYawPitchRoll();
void _GetSensorRawData();
void _GetQuaternion();
void _GetYawPitchRollRad();
void _Convert_Rad_to_Deg(float *pArr);
float _InvSqrt(float nNumber);
inline void _CalculatePID();
inline void CalculateThrottleVal();
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
bool                    nDMPReadyFlag = false;                  // set true if DMP init was successful
uint16_t                nPacketSize = 0;                            // expected DMP packet size (default is 42 bytes)
float                   nDeclinationAngle = 0.0f;

SelfFly_T               nSelfFlyHndl;                            // SelfFly Main Handle

/*----------------------------------------------------------------------------------------
 Function Implementation
 ----------------------------------------------------------------------------------------*/
void setup()
{
    int32_t                 i = 0;
    uint8_t                 nDevStatus;                         // return status after each device operation (0 = success, !0 = error)
    AccelGyroParam_T        *pAccelGyroParam = &(nSelfFlyHndl.nAccelGyroParam);
    MagneticParam_T         *pMagParam = &(nSelfFlyHndl.nMagParam);
    BaroParam_T             *pBaroParam = &(nSelfFlyHndl.nBaroParam);
    
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
    
    memset(&(nSelfFlyHndl.nPitch), 0, sizeof(AxisErrRate_T));
    memset(&(nSelfFlyHndl.nRoll), 0, sizeof(AxisErrRate_T));
    memset(&(nSelfFlyHndl.nYaw), 0, sizeof(AxisErrRate_T));
    memset(&(nSelfFlyHndl.nRCCh[0]), 0, MAX_CH_RC * sizeof(float));
    memset(&(nSelfFlyHndl.nThrottle[0]), ESC_MIN, MAX_CH_ESC * sizeof(int32_t));
    
    for(i=0 ; i<MAX_CH_RC ; i++)
        nSelfFlyHndl.nRCPrevChangeTime[i] = micros();
    
    memset(&(nSelfFlyHndl.nFineGyro), 0, 3 * sizeof(float));
    memset(&(nSelfFlyHndl.nFineRPY), 0, 3 * sizeof(float));
    memset(&(nSelfFlyHndl.nFineG), 0, 3 * sizeof(float));
    memset(&(nSelfFlyHndl.nFineQ), 0, 3 * sizeof(float));
}


void loop()
{
    float                   nStartTime0 = 0.0f, nStartTime1 = 0.0f, nStartTime2 = 0.0f, nStartTime3 = 0.0f, nEndTime = 0.0f;
    AccelGyroParam_T        *pAccelGyroParam = &(nSelfFlyHndl.nAccelGyroParam);
    MagneticParam_T         *pMagParam = &(nSelfFlyHndl.nMagParam);
    BaroParam_T             *pBaroParam = &(nSelfFlyHndl.nBaroParam);
    
    nStartTime0 = micros();
    
    ////////////////////////////////// Get angle & gyro ///////////////////////////////////////////////////////
    
    nStartTime1 = micros();
    // Get AccelGyro Sensor Value
    #if !__GYROACCEL_DMP_ENABLED__
    _AccelGyro_GetData();
    _AccelGyro_CalculateAngle();
    #else
    if(0 != _AccelGyro_GetDMPData())
        return;
    #endif
    
    nStartTime2 = micros();
    // Get Magnetic Sensor Value
    _Mag_GetData();
    _Mag_CalculateDirection();
    
    // Get Barometer Sensor Value
    _Barometer_GetData();
    
    nStartTime3 = micros();
    memset(&(nSelfFlyHndl.nThrottle[0]), ESC_MIN, 4 * sizeof(int32_t));
    
    // PID Computation
    _CalculatePID();
    
    //Throttle Calculation
    CalculateThrottleVal();
    
    //Update BLDCs
    UpdateESCs();
    
    #if __DEBUG__
    //_print_Gyro_Signals(pAccelGyroParam->nFineAngle);
    //_print_RPY_Signals(pAccelGyroParam->nFineRPY);
    //_print_MagData(pMagParam);
    //_print_BarometerData(pBaroParam);
    //_print_Throttle_Signals(nThrottle);
    //_print_RC_Signals();
    nEndTime = micros();
    Serialprint(" ");Serialprint((nEndTime - nStartTime0)/1000);
    Serialprintln("");
    #endif
}


int _AccelGyro_Initialize()
{
    Serialprintln(F("Initializing MPU..."));
    nSelfFlyHndl.nAccelGyroHndl.initialize();
    
    // verify connection
    Serialprint(F("    Testing device connections..."));
    Serialprintln(nSelfFlyHndl.nAccelGyroHndl.testConnection() ? F("  MPU6050 connection successful") : F("  MPU6050 connection failed"));
    
    nSelfFlyHndl.nAccelGyroHndl.setI2CMasterModeEnabled(false);
    nSelfFlyHndl.nAccelGyroHndl.setI2CBypassEnabled(true);
    nSelfFlyHndl.nAccelGyroHndl.setSleepEnabled(false);
    
    #if __GYROACCEL_DMP_ENABLED__
    {
        uint8_t                 nDevStatus;                         // return status after each device operation (0 = success, !0 = error)
        
        // load and configure the DMP
        Serialprintln(F("    Initializing DMP..."));
        nDevStatus = nSelfFlyHndl.nAccelGyroHndl.dmpInitialize();
        
        // make sure it worked (returns 0 if so)
        if(0 == nDevStatus)
        {
            // turn on the DMP, now that it's ready
            Serialprintln(F("        Enabling DMP..."));
            nSelfFlyHndl.nAccelGyroHndl.setDMPEnabled(true);
            
            // enable Arduino interrupt detection
            Serialprintln(F("            Enabling interrupt detection (Arduino external interrupt 0)..."));
            pinMode(PIN_GY86_EXT_INTERRUPT, INPUT);
            digitalWrite(PIN_GY86_EXT_INTERRUPT, HIGH);
            PCintPort::attachInterrupt(PIN_GY86_EXT_INTERRUPT, dmpDataReady, RISING);
            
            // set our DMP Ready flag so the main loop() function knows it's okay to use it
            Serialprintln(F("    Done... DMP ready! Waiting for first interrupt..."));
            nDMPReadyFlag = true;
            
            // get expected DMP packet size for later comparison
            nPacketSize = nSelfFlyHndl.nAccelGyroHndl.dmpGetFIFOPacketSize();
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
    nSelfFlyHndl.nAccelGyroHndl.setXGyroOffset(220);
    nSelfFlyHndl.nAccelGyroHndl.setYGyroOffset(76);
    nSelfFlyHndl.nAccelGyroHndl.setZGyroOffset(-85);
    nSelfFlyHndl.nAccelGyroHndl.setZAccelOffset(1788);                                     // 1688 factory default for my test chip
    nSelfFlyHndl.nAccelGyroHndl.setRate(1);                                                // Sample Rate (500Hz = 1Hz Gyro SR / 1+1)
    nSelfFlyHndl.nAccelGyroHndl.setDLPFMode(MPU6050_DLPF_BW_20);                           // Low Pass filter 20hz
    nSelfFlyHndl.nAccelGyroHndl.setFullScaleGyroRange(GYRO_FS_PRECISIOM);                // 250? / s (MPU6050_GYRO_FS_250)
    nSelfFlyHndl.nAccelGyroHndl.setFullScaleAccelRange(ACCEL_FS_PRECISIOM);                // +-2g (MPU6050_ACCEL_FS_2)

    // Calibrate GyroAccel
    Serialprint(F("    Calibrating Gyro & Accel   "));
    _AccelGyro_Calibrate();
    Serialprintln(F("    Done"));
    Serialprintln(F("Done"));
    
    return 0;
}


void _AccelGyro_GetData()
{
    AccelGyroParam_T        *pAccelGyroParam = &(nSelfFlyHndl.nAccelGyroParam);
    int16_t                 *pRawGyro = &(pAccelGyroParam->nRawGyro[X_AXIS]);
    int16_t                 *pRawAccel = &(pAccelGyroParam->nRawAccel[X_AXIS]);
    const float             *pBaseGyro = &(pAccelGyroParam->nBaseGyro[X_AXIS]);

    pAccelGyroParam->nCurrReadTime = micros();

    // Read Gyro and Accelerate Data
    //nSelfFlyHndl.nAccelGyroHndl.getRotation(&pRawGyro[X_AXIS], &pRawGyro[Y_AXIS], &pRawGyro[Z_AXIS]);
    //nSelfFlyHndl.nAccelGyroHndl.getAcceleration(&pRawAccel[X_AXIS], &pRawAccel[Y_AXIS], &pRawAccel[Z_AXIS]);
    nSelfFlyHndl.nAccelGyroHndl.getMotion6(&pRawAccel[X_AXIS], &pRawAccel[Y_AXIS], &pRawAccel[Z_AXIS],
                                            &pRawGyro[X_AXIS], &pRawGyro[Y_AXIS], &pRawGyro[Z_AXIS]);

    pRawGyro[X_AXIS] = (pRawGyro[X_AXIS] - pBaseGyro[X_AXIS]);
    pRawGyro[Y_AXIS] = (pRawGyro[Y_AXIS] - pBaseGyro[Y_AXIS]);
    pRawGyro[Z_AXIS] = (pRawGyro[Z_AXIS] - pBaseGyro[Z_AXIS]);
}


void _AccelGyro_CalculateAngle()
{
    AccelGyroParam_T        *pAccelGyroParam = &(nSelfFlyHndl.nAccelGyroParam);
    const float             nFS = 131.0f;
    const int16_t           *pRawGyro = &(pAccelGyroParam->nRawGyro[X_AXIS]);
    const int16_t           *pRawAccel = &(pAccelGyroParam->nRawAccel[X_AXIS]);
    const float             *pBaseGyro = &(pAccelGyroParam->nBaseGyro[X_AXIS]);
    float                   *pFineAngle = &(pAccelGyroParam->nFineAngle[X_AXIS]);
    static float            nGyroAngle[3] = {0, };
    float                   nAccelAngle[3] = {0, };
    float                   nGyroDiffAngle[3] = {0, };
    float                   nRawAngle[3] = {0, };
    static float            nPrevRawAngle[3] = {0, };
    
    // Convert Gyro Values to Degrees/Sec
    nGyroDiffAngle[X_AXIS] = pRawGyro[X_AXIS] / nFS;
    nGyroDiffAngle[Y_AXIS] = pRawGyro[Y_AXIS] / nFS;
    nGyroDiffAngle[Z_AXIS] = pRawGyro[Z_AXIS] / nFS;
    
    // float accel_vector_length = sqrt(pow(accel_x,2) + pow(accel_y,2) + pow(accel_z,2));
    nAccelAngle[X_AXIS] = atan(pRawAccel[Y_AXIS] / sqrt(pow(pRawAccel[X_AXIS], 2) + pow(pRawAccel[Z_AXIS], 2))) * RAD_TO_DEG_SCALE;
    nAccelAngle[Y_AXIS] = atan((-1) * pRawAccel[X_AXIS] / sqrt(pow(pRawAccel[Y_AXIS], 2) + pow(pRawAccel[Z_AXIS], 2))) * RAD_TO_DEG_SCALE;
    nAccelAngle[Z_AXIS] = 0;
    
    // Compute the (filtered) gyro angles
    {
        float dt = (pAccelGyroParam->nCurrReadTime - pAccelGyroParam->nPrevReadTime) / 1000000.0;
        
        nGyroAngle[X_AXIS] = nGyroDiffAngle[X_AXIS] * dt + pFineAngle[X_AXIS];
        nGyroAngle[Y_AXIS] = nGyroDiffAngle[Y_AXIS] * dt + pFineAngle[Y_AXIS];
        nGyroAngle[Z_AXIS] = nGyroDiffAngle[Z_AXIS] * dt + pFineAngle[Z_AXIS];
        
        // Compute the drifting gyro angles
        //nRawAngle[X_AXIS] = nGyroDiffAngle[X_AXIS] * dt + nPrevRawAngle[X_AXIS];
        //nRawAngle[Y_AXIS] = nGyroDiffAngle[Y_AXIS] * dt + nPrevRawAngle[Y_AXIS];
        //nRawAngle[Z_AXIS] = nGyroDiffAngle[Z_AXIS] * dt + nPrevRawAngle[Z_AXIS];
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
    
    // Update the saved data with the latest values
    pAccelGyroParam->nPrevReadTime = pAccelGyroParam->nCurrReadTime;
}


void _AccelGyro_Calibrate()
{
    int                     i = 0;
    int32_t                 nRawGyro[3] = {0, };
    int32_t                 nRawAccel[3] = {0, };
    const int               nLoopCnt = 20;
    AccelGyroParam_T        *pAccelGyroParam = &(nSelfFlyHndl.nAccelGyroParam);
    
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
    pAccelGyroParam->nBaseGyro[X_AXIS] = nRawGyro[X_AXIS] / nLoopCnt;
    pAccelGyroParam->nBaseGyro[Y_AXIS] = nRawGyro[Y_AXIS] / nLoopCnt;
    pAccelGyroParam->nBaseGyro[Z_AXIS] = nRawGyro[Z_AXIS] / nLoopCnt;
    pAccelGyroParam->nBaseAccel[X_AXIS] = nRawAccel[X_AXIS] / nLoopCnt;
    pAccelGyroParam->nBaseAccel[Y_AXIS] = nRawAccel[Y_AXIS] / nLoopCnt;
    pAccelGyroParam->nBaseAccel[Z_AXIS] = nRawAccel[Z_AXIS] / nLoopCnt;
}


#if __GYROACCEL_DMP_ENABLED__
int _AccelGyro_GetDMPData()
{
    uint16_t                nFIFOCnt = 0;                                               // count of all bytes currently in FIFO
    uint8_t                 nFIFOBuf[64];                                               // FIFO storage buffer
    uint8_t                 nMPUInterruptStat;                                          // holds actual interrupt status byte from MPU
    AccelGyroParam_T        *pAccelGyroParam = &(nSelfFlyHndl.nAccelGyroParam);

    if(!nDMPReadyFlag)
    {
        Serialprintln("DMP Not Ready    Loop Run Fails");
        return -1;
    }
    
    while(!nMPUInterruptFlag && nFIFOCnt < nPacketSize)
    {
        // Wait Till Interrupt Up!!
    }
    
    // reset interrupt flag and get INT_STATUS byte
    nMPUInterruptFlag = false;
    nMPUInterruptStat = nSelfFlyHndl.nAccelGyroHndl.getIntStatus();
    
    // get current FIFO count
    nFIFOCnt = nSelfFlyHndl.nAccelGyroHndl.getFIFOCount();
    
    // check for overflow (this should never happen unless our code is too inefficient)
    if((nMPUInterruptStat & 0x10) || nFIFOCnt == 1024)
    {
        // reset so we can continue cleanly
        nSelfFlyHndl.nAccelGyroHndl.resetFIFO();
        //Serialprintln(F("FIFO overflow!"));
    }
    else if(nMPUInterruptStat & 0x02)
    {
        int16_t             nGyro[3];
        
        // wait for correct available data length, should be a VERY short wait
        while (nFIFOCnt < nPacketSize)
            nFIFOCnt = nSelfFlyHndl.nAccelGyroHndl.getFIFOCount();
        
        // read a packet from FIFO
        nSelfFlyHndl.nAccelGyroHndl.getFIFOBytes(nFIFOBuf, nPacketSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        nFIFOCnt -= nPacketSize;
        
        nSelfFlyHndl.nAccelGyroHndl.dmpGetGyro(nGyro, nFIFOBuf);
        nSelfFlyHndl.nAccelGyroHndl.dmpGetQuaternion(&nQuater, nFIFOBuf);
        nSelfFlyHndl.nAccelGyroHndl.dmpGetGravity(&nGravity, &nQuater);
        nSelfFlyHndl.nAccelGyroHndl.dmpGetYawPitchRoll(pAccelGyroParam->nFineRPY, &nQuater, &nGravity);
        
        pAccelGyroParam->nFineAngle[X_AXIS] = nGyro[X_AXIS];
        pAccelGyroParam->nFineAngle[Y_AXIS] = nGyro[Y_AXIS];
        pAccelGyroParam->nFineAngle[Z_AXIS] = nGyro[Z_AXIS];
    }
    
    return 0;
}
#endif


int _Mag_Initialize()
{
    // initialize Magnetic
    Serialprintln(F("Initializing Magnetic..."));
    _Mag_Start();
    
    // Calibrate Magnetic
    Serialprint(F("    Calibrating Magnetic...."));
    _Mag_Calibrate();
    
    // Date: 2015-11-05
    // Location: Seoul, South Korea
    // Latitude: 37.0000° North
    // Longitude: 126.0000° East
    // Magnetic declination: 7° 59.76' West
    // Annual Change (minutes/year): 3.9 '/y West
    // http://www.geomag.nrcan.gc.ca/calc/mdcal-en.php
    nDeclinationAngle = (7.0 + (59.76 / 60.0)) / (180 / M_PI);
    Serialprintln(F("Done"));
}


void _Mag_Start()
{
    // Open communication with HMC5883L
    Wire.beginTransmission(HMC5883L_ADDRESS);
    Wire.write(0x00);                                           // Configuration Register A
    Wire.write(0x70);                                           // Num samples: 8 ; output rate: 15Hz ; normal measurement mode
    Wire.endTransmission();
    delay(1);
    
    // Open communication with HMC5883L
    Wire.beginTransmission(HMC5883L_ADDRESS);
    Wire.write(0x01);                                           // Configuration Register B
    Wire.write(0x20);                                           // Configuration gain 1.3Ga
    Wire.endTransmission();
    delay(1);
    
    // Put the HMC5883 IC into the correct operating mode
    // Open communication with HMC5883L
    Wire.beginTransmission(HMC5883L_ADDRESS);
    Wire.write(0x02);                                           // Select mode register
    Wire.write(0x00);                                           // Continuous measurement mode
    Wire.endTransmission();
    delay(1);
}


void _Mag_GetData()
{
    int                     i = 0;
    MagneticParam_T         *pMagParam = &(nSelfFlyHndl.nMagParam);
    
    //Tell the HMC5883 where to begin reading data
    Wire.beginTransmission(HMC5883L_ADDRESS);
    Wire.write(0x03);                                               // Select register 3, X MSB register
    Wire.endTransmission();
    
    //Read data from each axis, 2 registers per axis
    Wire.requestFrom(HMC5883L_ADDRESS, 6);
    while(Wire.available())
    {
        pMagParam->nRawBits[i] = Wire.read();
        i++;
    }
    Wire.endTransmission();
    
    pMagParam->nRawMagData[X_AXIS] = ((pMagParam->nRawBits[0] << 8) | pMagParam->nRawBits[1]);                  // Offset + 1.05
    pMagParam->nRawMagData[Z_AXIS] = ((pMagParam->nRawBits[2] << 8) | pMagParam->nRawBits[3]) * (-1);           // + 0.05
    pMagParam->nRawMagData[Y_AXIS] = ((pMagParam->nRawBits[4] << 8) | pMagParam->nRawBits[5]) * (-1);           // - 0.55
    
    //pMagParam->nRawMagData = nMagHndl.readRaw();
    //pMagParam->nNormMagData = nMagHndl.readNormalize();
}


void _Mag_CalculateDirection()
{
    int                     i = 0;
    int                     MagX,MagY,MagZ;
    static float            MagXf = 0.0f ,MagYf = 0.0f ,MagZf = 0.0f;
    MagneticParam_T         *pMagParam = &(nSelfFlyHndl.nMagParam);
    
    pMagParam->nNormMagData[X_AXIS] = pMagParam->nNormMagData[X_AXIS] + (pMagParam->nRawMagData[X_AXIS] - pMagParam->nNormMagData[X_AXIS]) * 0.55;
    pMagParam->nNormMagData[Y_AXIS] = pMagParam->nNormMagData[Y_AXIS] + (pMagParam->nRawMagData[Y_AXIS] - pMagParam->nNormMagData[Y_AXIS]) * 0.55;
    pMagParam->nNormMagData[Z_AXIS] = pMagParam->nNormMagData[Z_AXIS] + (pMagParam->nRawMagData[Z_AXIS] - pMagParam->nNormMagData[Z_AXIS]) * 0.55;

    // adjust for  Magnetic axis offsets/sensitivity differences by scaling to +/-5 range
    pMagParam->nFineMag[X_AXIS] = ((pMagParam->nNormMagData[X_AXIS] - MAG_X_MIN) / (MAG_X_RANGE)) * 10.0 - 5.0;
    pMagParam->nFineMag[Y_AXIS] = ((pMagParam->nNormMagData[Y_AXIS] - MAG_Y_MIN) / (MAG_Y_RANGE)) * 10.0 - 5.0;
    pMagParam->nFineMag[Z_AXIS] = ((pMagParam->nNormMagData[Z_AXIS] - MAG_Z_MIN) / (MAG_Z_RANGE)) * 10.0 - 5.0;

    pMagParam->nMagHeadingRad = atan2(pMagParam->nRawMagData[Y_AXIS], pMagParam->nRawMagData[X_AXIS]) + nDeclinationAngle;
    
    if(pMagParam->nMagHeadingRad < 0)
        pMagParam->nMagHeadingRad += DOUBLE_RADIAN;
    
    if(pMagParam->nMagHeadingRad > DOUBLE_RADIAN)
        pMagParam->nMagHeadingRad -= DOUBLE_RADIAN;
    
    pMagParam->nMagHeadingDeg = pMagParam->nMagHeadingRad * RAD_TO_DEG_SCALE;
    
    //if(pMagParam->nMagHeadingDeg >= 1 && pMagParam->nMagHeadingDeg < 240)
    //    pMagParam->nMagHeadingDeg = map(pMagParam->nMagHeadingDeg, 0, 239, 0, 179);
    //else if(pMagParam->nMagHeadingDeg >= 240)
    //    pMagParam->nMagHeadingDeg = map(pMagParam->nMagHeadingDeg, 240, 360, 180, 360);
    
    // Smooth angles rotation for +/- 3deg
    //int smoothHeadingDegrees = round(pMagParam->nMagHeadingDeg);
    //int previousDegree = 0;
    //if(smoothHeadingDegrees < (previousDegree + 3) && smoothHeadingDegrees > (previousDegree - 3))
    //    smoothHeadingDegrees = previousDegree;
}


void _Mag_Calibrate()
{
    int                     i = 0;
    float                   nSumMinX = 0.0f;
    float                   nSumMaxX = 0.0f;
    float                   nSumMinY = 0.0f;
    float                   nSumMaxY = 0.0f;
    const int               nLoopCnt = 50;
    MagneticParam_T         *pMagParam = &(nSelfFlyHndl.nMagParam);
    
    for(i=0 ; i<nLoopCnt ; i++)
    {
        float           nMinX = 0.0f;
        float           nMaxX = 0.0f;
        float           nMinY = 0.0f;
        float           nMaxY = 0.0f;
        
        _Mag_GetData();
        
        // Determine Min / Max values
        if(pMagParam->nRawMagData[X_AXIS] < nMinX)
            nMinX = pMagParam->nRawMagData[X_AXIS];
        if(pMagParam->nRawMagData[X_AXIS] > nMaxX)
            nMaxX = pMagParam->nRawMagData[X_AXIS];
        if(pMagParam->nRawMagData[Y_AXIS] < nMinY)
            nMinY = pMagParam->nRawMagData[Y_AXIS];
        if(pMagParam->nRawMagData[Y_AXIS] > nMaxY)
            nMaxY = pMagParam->nRawMagData[Y_AXIS];
        
        nSumMinX += nMinX;
        nSumMaxX += nMaxX;
        nSumMinY += nMinY;
        nSumMaxY += nMaxY;
        
        delay(20);
        
        Serialprint(".");
    }
    
    // Calculate offsets
    nSelfFlyHndl.nMagHndl.setOffset(((nSumMaxX + nSumMinX) / 2 / nLoopCnt), ((nSumMaxY + nSumMinY) / 2 / nLoopCnt));
}


int _Barometer_Initialize()
{
    Serialprint(F("Initializing Barometer..."));
    nSelfFlyHndl.nBaroHndl = MS561101BA();
    nSelfFlyHndl.nBaroHndl.init(MS561101BA_ADDR_CSB_LOW);
    Serialprintln(F("Done"));
}


void _Barometer_GetData()
{
    BaroParam_T             *pBaroParam = &(nSelfFlyHndl.nBaroParam);

    pBaroParam->nRealTemperature = nSelfFlyHndl.nBaroHndl.getTemperature(MS561101BA_OSR_256);
    pBaroParam->nRealPressure = nSelfFlyHndl.nBaroHndl.getPressure(MS561101BA_OSR_256);
    nSelfFlyHndl.nBaroHndl.pushPressure(pBaroParam->nRealPressure);
    pBaroParam->nAvgpressure = nSelfFlyHndl.nBaroHndl.getAvgPressure();
    pBaroParam->nAbsoluteAltitude = nSelfFlyHndl.nBaroHndl.getAltitude(pBaroParam->nAvgpressure, pBaroParam->nRealTemperature);
}


inline void UpdateESCs()
{
    Servo                   *pESC = &(nSelfFlyHndl.nESC[0]);
    int32_t                 *pThrottle = &(nSelfFlyHndl.nThrottle[0]);
    int                     i = 0;
    
    for(i=0 ; i<MAX_CH_ESC ; i++)
        pESC[i].writeMicroseconds(pThrottle[i]);
}


inline void nRCInterrupt_CB0()
{
    if(!nInterruptLockFlag)
        nSelfFlyHndl.nRCCh[0] = micros() - nSelfFlyHndl.nRCPrevChangeTime[0];
    
    nSelfFlyHndl.nRCPrevChangeTime[0] = micros();
}


inline void nRCInterrupt_CB1()
{
    if(!nInterruptLockFlag)
        nSelfFlyHndl.nRCCh[1] = micros() - nSelfFlyHndl.nRCPrevChangeTime[1];
    
    nSelfFlyHndl.nRCPrevChangeTime[1] = micros();
}


inline void nRCInterrupt_CB2()
{
    if(!nInterruptLockFlag)
        nSelfFlyHndl.nRCCh[2] = micros() - nSelfFlyHndl.nRCPrevChangeTime[2];
    
    nSelfFlyHndl.nRCPrevChangeTime[2] = micros();
}


inline void nRCInterrupt_CB3()
{
    if(!nInterruptLockFlag)
        nSelfFlyHndl.nRCCh[3] = micros() - nSelfFlyHndl.nRCPrevChangeTime[3];
    
    nSelfFlyHndl.nRCPrevChangeTime[3] = micros();
}


inline void nRCInterrupt_CB4()
{
    if(!nInterruptLockFlag)
        nSelfFlyHndl.nRCCh[4] = micros() - nSelfFlyHndl.nRCPrevChangeTime[4];
    
    nSelfFlyHndl.nRCPrevChangeTime[4] = micros();
}


inline void dmpDataReady()
{
    nMPUInterruptFlag = true;
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


inline void arm()
{
    Servo                   *pESC = &(nSelfFlyHndl.nESC[0]);
    int                     i = 0;
    
    for(i=0 ; i<MAX_CH_ESC ; i++)
        pESC[i].writeMicroseconds(ESC_MIN);
    
    delay(ESC_ARM_DELAY);
}


void _ESC_Initialize()
{
    Servo                   *pESC = &(nSelfFlyHndl.nESC[0]);
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


void _AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
{
    float                   nRecipNorm = 0.0f;
    float                   nFineQ00, nFineQ01, nFineQ02, nFineQ03, nFineQ11, nFineQ12, nFineQ13, nFineQ22, nFineQ23, nFineQ33;
    float                   halfex = 0.0f, halfey = 0.0f, halfez = 0.0f;
    float                   qa, qb, qc;
    static volatile float   nIntegralFBx = 0.0f,  nIntegralFBy = 0.0f, nIntegralFBz = 0.0f;
    const float             nSampleFreq = nSelfFlyHndl.nSampleFreq;
    float                   *pFineQ = &(nSelfFlyHndl.nFineQ[0]);
    
    
    // Auxiliary variables to avoid repeated arithmetic
    nFineQ00 = pFineQ[0] * pFineQ[0];
    nFineQ01 = pFineQ[0] * pFineQ[1];
    nFineQ02 = pFineQ[0] * pFineQ[2];
    nFineQ03 = pFineQ[0] * pFineQ[3];
    nFineQ11 = pFineQ[1] * pFineQ[1];
    nFineQ12 = pFineQ[1] * pFineQ[2];
    nFineQ13 = pFineQ[1] * pFineQ[3];
    nFineQ22 = pFineQ[2] * pFineQ[2];
    nFineQ23 = pFineQ[2] * pFineQ[3];
    nFineQ33 = pFineQ[3] * pFineQ[3];
    
    if((mx != 0.0f) && (my != 0.0f) && (mz != 0.0f))
    {
        float hx, hy, bx, bz;
        float halfwx, halfwy, halfwz;
        
        // Normalise magnetometer measurement
        nRecipNorm = _InvSqrt(mx * mx + my * my + mz * mz);
        mx *= nRecipNorm;
        my *= nRecipNorm;
        mz *= nRecipNorm;
        
        // Reference direction of Earth's magnetic field
        hx = 2.0f * (mx * (0.5f - nFineQ22 - nFineQ33) + my * (nFineQ12 - nFineQ03) + mz * (nFineQ13 + nFineQ02));
        hy = 2.0f * (mx * (nFineQ12 + nFineQ03) + my * (0.5f - nFineQ11 - nFineQ33) + mz * (nFineQ23 - nFineQ01));
        bx = sqrt(hx * hx + hy * hy);
        bz = 2.0f * (mx * (nFineQ13 - nFineQ02) + my * (nFineQ23 + nFineQ01) + mz * (0.5f - nFineQ11 - nFineQ22));
        
        // Estimated direction of magnetic field
        halfwx = bx * (0.5f - nFineQ22 - nFineQ33) + bz * (nFineQ13 - nFineQ02);
        halfwy = bx * (nFineQ12 - nFineQ03) + bz * (nFineQ01 + nFineQ23);
        halfwz = bx * (nFineQ02 + nFineQ13) + bz * (0.5f - nFineQ11 - nFineQ22);
        
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
        nRecipNorm = _InvSqrt(ax * ax + ay * ay + az * az);
        ax *= nRecipNorm;
        ay *= nRecipNorm;
        az *= nRecipNorm;
        
        // Estimated direction of gravity
        halfvx = nFineQ13 - nFineQ02;
        halfvy = nFineQ01 + nFineQ23;
        halfvz = nFineQ00 + nFineQ33 - 0.5f;
        
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
    qa = pFineQ[0];
    qb = pFineQ[1];
    qc = pFineQ[2];
    pFineQ[0] += ((-qb * gx) - (qc * gy) - (pFineQ[3] * gz));
    pFineQ[1] += (( qa * gx) + (qc * gz) - (pFineQ[3] * gy));
    pFineQ[2] += (( qa * gy) - (qb * gz) + (pFineQ[3] * gx));
    pFineQ[3] += (( qa * gz) + (qb * gy) - (qc * gx));
    
    // Normalise quaternion
    nRecipNorm = _InvSqrt((pFineQ[0] * pFineQ[0]) + (pFineQ[1] * pFineQ[1]) + (pFineQ[2] * pFineQ[2]) + (pFineQ[3] * pFineQ[3]));
    pFineQ[0] *= nRecipNorm;
    pFineQ[1] *= nRecipNorm;
    pFineQ[2] *= nRecipNorm;
    pFineQ[3] *= nRecipNorm;
}


void _GetYawPitchRoll()
{
    _GetYawPitchRollRad();
    
    // Convert Radian to Degree
    _Convert_Rad_to_Deg(&(nSelfFlyHndl.nFineRPY[0]));
}


void _GetSensorRawData()
{
    // Get AccelGyro Raw Data
    _AccelGyro_GetData();
    
    // Get Magnetic Raw Data
    _Mag_GetData();
    
    nSelfFlyHndl.nLatestSensorCapTime = micros();
}


void _GetQuaternion()
{
    float                   nVal[9];
    float                   *pFineQ = &(nSelfFlyHndl.nFineQ[0]);
    
    //getValues(val);
    _GetSensorRawData();
    
    nSelfFlyHndl.nSampleFreq = 1.0 / ((nSelfFlyHndl.nLatestSensorCapTime - nSelfFlyHndl.nPrevSensorCapTime) / 1000000.0);
    nSelfFlyHndl.nPrevSensorCapTime = nSelfFlyHndl.nLatestSensorCapTime;
    
    // gyro values are expressed in deg/sec, the * M_PI/180 will convert it to radians/sec
    //AHRSupdate(val[3] * DEG_TO_RAD_SCALE, val[4] * DEG_TO_RAD_SCALE, val[5] * DEG_TO_RAD_SCALE,
    //           val[0], val[1], val[2], val[6], val[7], val[8]);
}


void _GetYawPitchRollRad()
{
    float                   *pFineG = &(nSelfFlyHndl.nFineG[0]);
    float                   *pFineQ = &(nSelfFlyHndl.nFineQ[0]);
    float                   *pFineRPY = &(nSelfFlyHndl.nFineRPY[0]);
    
    _GetQuaternion();
    
    pFineG[X_AXIS] = 2 * ((pFineQ[1] * pFineQ[3]) - (pFineQ[0] * pFineQ[2]));
    pFineG[Y_AXIS] = 2 * ((pFineQ[0] * pFineQ[1]) + (pFineQ[2] * pFineQ[3]));
    pFineG[Z_AXIS] = (pFineQ[0] * pFineQ[0]) - (pFineQ[1] * pFineQ[1]) - (pFineQ[2] * pFineQ[2]) + (pFineQ[3] * pFineQ[3]);
    
    pFineRPY[0] = atan2((2 * pFineQ[1] * pFineQ[2]) - (2 * pFineQ[0] * pFineQ[3]), (2 * pFineQ[0] * pFineQ[0]) + (2 * pFineQ[1] * pFineQ[1]) - 1);
    pFineRPY[1] = atan(pFineG[X_AXIS] / sqrt((pFineG[Y_AXIS] * pFineG[Y_AXIS]) + (pFineG[Z_AXIS] * pFineG[Z_AXIS])));
    pFineRPY[2] = atan(pFineG[Y_AXIS] / sqrt((pFineG[X_AXIS] * pFineG[X_AXIS]) + (pFineG[Z_AXIS] * pFineG[Z_AXIS])));
}


void _Convert_Rad_to_Deg(float *pArr)
{
    pArr[0] *= RAD_TO_DEG_SCALE;
    pArr[1] *= RAD_TO_DEG_SCALE;
    pArr[2] *= RAD_TO_DEG_SCALE;
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
    AxisErrRate_T           *pPitch = &(nSelfFlyHndl.nPitch);
    AxisErrRate_T           *pRoll = &(nSelfFlyHndl.nRoll);
    AxisErrRate_T           *pYaw = &(nSelfFlyHndl.nYaw);
    int32_t                 *pThrottle = &(nSelfFlyHndl.nThrottle[0]);
    float                   *pRCCh = &(nSelfFlyHndl.nRCCh[0]);
    
    _AcquireLock();
    
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


#if __DEBUG__
void _print_RC_Signals()
{
    float                   *pRCCh = &(nSelfFlyHndl.nRCCh[0]);
    
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
    float                   *pFineGyro = &(nSelfFlyHndl.nFineGyro[0]);
    
    Serialprint("   //    Roll Gyro : ");
    Serialprint(pFineGyro[0]);
    Serialprint("   Pitch Gyro : ");
    Serialprint(pFineGyro[1]);
    Serialprint("   Yaw Gyro : ");
    Serialprint(pFineGyro[2]);
}


void _print_Throttle_Signals()
{
    int32_t                 *pThrottle = &(nSelfFlyHndl.nThrottle[0]);

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
    float                   *pFineRPY = &(nSelfFlyHndl.nFineRPY[0]);
    
    Serialprint("   //    Roll: ");
    Serialprint(pFineRPY[2]);
    Serialprint("   Pitch: ");
    Serialprint(pFineRPY[1]);
    Serialprint("   Yaw: ");
    Serialprint(pFineRPY[0]);
}

void _print_MagData()
{
    MagneticParam_T         *pMagParam = &(nSelfFlyHndl.nMagParam);

    Serialprint("   //    Magnetic -> X:"); Serialprint(pMagParam->nNormMagData[X_AXIS]);
    Serialprint("   Y:"); Serialprint(pMagParam->nNormMagData[Y_AXIS]);
    Serialprint("   Z:"); Serialprint(pMagParam->nNormMagData[Z_AXIS]);
    Serialprint("   Head:"); Serialprint(pMagParam->nMagHeadingDeg);
}

void _print_BarometerData()
{
    BaroParam_T             *pBaroParam = &(nSelfFlyHndl.nBaroParam);

    Serialprint("   //    Barometer -> Temp:"); Serialprint(pBaroParam->nRealTemperature);
    Serialprint("   Press:"); Serialprint(pBaroParam->nRealPressure);
    Serialprint("   AvgPress:"); Serialprint(pBaroParam->nAvgpressure);
    Serialprint("   AbsAlt:"); Serialprint(pBaroParam->nAbsoluteAltitude);
}
#endif














