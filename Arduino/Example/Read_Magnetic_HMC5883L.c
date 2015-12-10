
/*----------------------------------------------------------------------------------------
 File Inclusions
 ----------------------------------------------------------------------------------------*/
#include <Servo.h>
#include <I2Cdev.h>
#include <PinChangeInt.h>
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include <Wire.h>
#endif
#include <MPU6050_6Axis_MotionApps20.h>
#include <HMC5883L.h>

/*----------------------------------------------------------------------------------------
 Constant Definitions
 ----------------------------------------------------------------------------------------*/
// Define Axis
#define X_AXIS                              (0)
#define Y_AXIS                              (1)
#define Z_AXIS                              (2)

#define GYRO_FS_PRECISIOM                   (MPU6050_GYRO_FS_250)
#define GYRO_FS                             (131.0f)                        // (2^15 - 1) / (250 * (1 << GYRO_FS_PRECISIOM))
#define ACCEL_FS_PRECISIOM                  (MPU6050_ACCEL_FS_2)            //  MPU6050_ACCEL_FS_4  MPU6050_ACCEL_FS_8  MPU6050_ACCEL_FS_16
#define ACCEL_STD_DENOM                     (16384.0f / (1 << ACCEL_FS_PRECISIOM))

#define ROUNDING_BASE                       (50)
#define SAMPLING_TIME                       (0.01)                          // Unit: Seconds

#define RAD_TO_DEG_SCALE                    (57.2958f)                      // = 180 / PI
#define DEG_TO_RAD_SCALE                    (0.0175f)                       // = PI / 180
#define SINGLE_RADIAN                       (3.141592)                      // = PI
#define DOUBLE_RADIAN                       (6.283184)                      // = 2 * PI
#define BARO_SEA_LEVEL_BASE                 (1013.25)                       // Base Sea Level


/*----------------------------------------------------------------------------------------
 Macro Definitions
 ----------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------
 Type Definitions
 ----------------------------------------------------------------------------------------*/
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
        
    unsigned long       nCurrSensorCapTime;
    unsigned long       nPrevSensorCapTime;
    float               nDiffTime;
    float               nSampleFreq;                            // half the sample period expressed in seconds
}SelfFly_T;


/*----------------------------------------------------------------------------------------
 Static Function
 ----------------------------------------------------------------------------------------*/
void _AccelGyro_Initialize();
void _Mag_Initialize();
void _Mag_GetData();
void _Mag_CalculateDirection();
void _print_MagData();


/*----------------------------------------------------------------------------------------
 Static Variable
 ----------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------
 Global Variable
 ----------------------------------------------------------------------------------------*/
SelfFly_T               *pSelfFlyHndl = NULL;                            // SelfFly Main Handle
float                   _gRawMagData[3];


/*----------------------------------------------------------------------------------------
 Function Implementation
 ----------------------------------------------------------------------------------------*/
void setup()
{
    int32_t                 i = 0;
    uint8_t                 nDevStatus;                         // return status after each device operation (0 = success, !0 = error)
    
    Serial.println("");    Serial.println("");    Serial.println("");    Serial.println("");
    Serial.println("   **********************************************   ");
    Serial.println("   **********************************************   ");
    
    pSelfFlyHndl = (SelfFly_T *) malloc(sizeof(SelfFly_T));
    
    memset(pSelfFlyHndl, 0, sizeof(SelfFly_T));
    
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
    #endif
    
    Serial.begin(115200);
    Serial.flush();
    
    while(!Serial); // wait for Leonardo enumeration, others continue immediately
    
    // Initialize Gyro_Accel
    _AccelGyro_Initialize();

    // Initialize Magnetic
    _Mag_Initialize();
    
    Serial.println("   **********************************************   ");
    Serial.println("   **********************************************   ");
    Serial.println("");    Serial.println("");    Serial.println("");    Serial.println("");
}


void loop()
{
    pSelfFlyHndl->nPrevSensorCapTime = pSelfFlyHndl->nCurrSensorCapTime;
    pSelfFlyHndl->nCurrSensorCapTime = micros();
    
    pSelfFlyHndl->nDiffTime = (pSelfFlyHndl->nCurrSensorCapTime - pSelfFlyHndl->nPrevSensorCapTime) / 1000000.0;
    pSelfFlyHndl->nSampleFreq = 1000000.0 / ((pSelfFlyHndl->nCurrSensorCapTime - pSelfFlyHndl->nPrevSensorCapTime));
    
    _Mag_GetData();
    
    _Mag_CalculateDirection();
    
    _print_MagData();
    
    delay(50);
}


void _AccelGyro_Initialize()
{
    pSelfFlyHndl->nAccelGyroHndl = MPU6050();
    
    Serial.println(F(" Initializing MPU..."));
    pSelfFlyHndl->nAccelGyroHndl.initialize();
    
    // Verify Vonnection
    Serial.print(F("    Testing device connections..."));
    Serial.println(pSelfFlyHndl->nAccelGyroHndl.testConnection() ? F("  MPU6050 connection successful") : F("  MPU6050 connection failed"));
    
    pSelfFlyHndl->nAccelGyroHndl.setI2CMasterModeEnabled(false);
    pSelfFlyHndl->nAccelGyroHndl.setI2CBypassEnabled(true);
    pSelfFlyHndl->nAccelGyroHndl.setSleepEnabled(false);
    
    // supply your own gyro offsets here, scaled for min sensitivity
    pSelfFlyHndl->nAccelGyroHndl.setRate(1);                                            // Sample Rate (500Hz = 1Hz Gyro SR / 1+1)
    pSelfFlyHndl->nAccelGyroHndl.setDLPFMode(MPU6050_DLPF_BW_20);                       // Low Pass filter 20hz
    pSelfFlyHndl->nAccelGyroHndl.setFullScaleGyroRange(GYRO_FS_PRECISIOM);              // 250? / s (MPU6050_GYRO_FS_250)
    pSelfFlyHndl->nAccelGyroHndl.setFullScaleAccelRange(ACCEL_FS_PRECISIOM);            // +-2g (MPU6050_ACCEL_FS_2)
    
    Serial.println(F(" MPU Initialized!!!"));
    
    return;
}


void _Mag_Initialize()
{
    pSelfFlyHndl->nMagHndl = HMC5883L();
    
    // initialize Magnetic
    Serial.println(F(" Initializing Magnetic..."));
    pSelfFlyHndl->nMagHndl.initialize();
    
    // Verify Vonnection
    Serial.print(F("    Testing device connections..."));
    Serial.println(pSelfFlyHndl->nMagHndl.testConnection() ? F("  HMC5883L connection successful") : F("  HMC5883L connection failed"));
    
    // Calibrate Magnetic
    Serial.print(F("    Start Calibration of Magnetic Sensor (HMC5883L) "));
    pSelfFlyHndl->nMagHndl.calibration_offset(3);
    Serial.println(F("Done"));
    
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
    
    Serial.println(F(" Done"));
    
    // Reference WebSite
    // http://www.meccanismocomplesso.org/en/arduino-magnetic-magnetic-magnetometer-hmc5883l/
}


void _Mag_GetData()
{
    float                   *pRawMagData = &(pSelfFlyHndl->nMagParam.nRawMagData[X_AXIS]);
    
    pSelfFlyHndl->nMagHndl.getScaledHeading(&(pRawMagData[X_AXIS]), &(pRawMagData[Y_AXIS]), &(pRawMagData[Z_AXIS]));
    pSelfFlyHndl->nMagHndl.getRawHeading(&(_gRawMagData[X_AXIS]), &(_gRawMagData[Y_AXIS]), &(_gRawMagData[Z_AXIS]));
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


void _print_MagData()
{
    MagneticParam_T         *pMagParam = &(pSelfFlyHndl->nMagParam);
    
    Serial.print("          "); Serial.print(_gRawMagData[0]);                       // Mx
    Serial.print("          "); Serial.print(_gRawMagData[1]);                       // My
    Serial.print("          "); Serial.print(_gRawMagData[2]);                       // Mz

    Serial.print("          "); Serial.print(pMagParam->nRawMagData[0]);                       // Mx
    Serial.print("          "); Serial.print(pMagParam->nRawMagData[1]);                       // My
    Serial.print("          "); Serial.print(pMagParam->nRawMagData[2]);                       // Mz
    
    Serial.print("          "); Serial.print(pMagParam->nMagHeadingDeg);                       // Magnetic Head
    Serial.print("          "); Serial.print(pMagParam->nSmoothHeadingDegrees);       // Smoothed Magnetic Head
    Serial.println("          ");
}
