
/*----------------------------------------------------------------------------------------
 File Inclusions
 ----------------------------------------------------------------------------------------*/
#include <Servo.h>
#include <I2Cdev.h>
#include <PinChangeInt.h>
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include <Wire.h>
#endif
#include <MS561101BA.h>

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

typedef struct _SelfFly_T
{
    // For Barometer Sensor
    MS561101BA          nBaroHndl;                              // MS5611 Barometer Interface
    BaroParam_T         nBaroParam;
        
    unsigned long       nCurrSensorCapTime;
    unsigned long       nPrevSensorCapTime;
    float               nDiffTime;
    float               nSampleFreq;                            // half the sample period expressed in seconds
}SelfFly_T;


/*----------------------------------------------------------------------------------------
 Static Function
 ----------------------------------------------------------------------------------------*/
void _Barometer_Initialize();
void _Barometer_GetData();
void _Barometer_CalculateData();
void _print_BarometerData();


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
    
    // Initialize Barometer
    _Barometer_Initialize();
    
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
    
    _Barometer_GetData();
    
    // Calculate Altitude
    _Barometer_CalculateData();
    
    _print_BarometerData();
    
    delay(50);
}


void _Barometer_Initialize()
{
    int                     i = 0;
    BaroParam_T             *pBaroParam = &(pSelfFlyHndl->nBaroParam);
    
    Serial.print(F(" Initializing Barometer Sensor (MS5611)..."));
    
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
    
    Serial.println(F(" Done"));
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


void _print_BarometerData()
{
    BaroParam_T             *pBaroParam = &(pSelfFlyHndl->nBaroParam);
    
    Serial.print("           "); Serial.print(pBaroParam->nAvgTemp);                  // Barometer AvgTemp
    Serial.print("           "); Serial.print(pBaroParam->nAvgPressure);              // AvgPress
    Serial.print("           "); Serial.print(pBaroParam->nAvgAbsoluteAltitude);      // AvgAlt
    Serial.print("           "); Serial.print(pBaroParam->nRelativeAltitude);         // RelativeAlt
    Serial.print("           "); Serial.print(pBaroParam->nVerticalSpeed);            // VerticalSpeed
    Serial.println("   ");
}

