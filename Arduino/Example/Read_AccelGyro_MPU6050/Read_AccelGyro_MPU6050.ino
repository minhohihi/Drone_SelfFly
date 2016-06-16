
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
#define ACCEL_FS                            (16384.0f / (1 << ACCEL_FS_PRECISIOM))

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
    float               _gRawGyro[3];
    float               _gRawAccel[3];
    float               _gRawTemp;
    float               nBaseGyro[3];
    float               nBaseAccel[3];
    float               nFineAngle[3];                          // Filtered Angles
}AccelGyroParam_T;

typedef struct _SelfFly_T
{
    // For Accelerator & Gyroscope Sensor
    MPU6050             nAccelGyroHndl;                         // MPU6050 Gyroscope Interface
    AccelGyroParam_T    nAccelGyroParam;
    int                 nCalibMean_AX, nCalibMean_AY, nCalibMean_AZ;
    int                 nCalibMean_GX, nCalibMean_GY, nCalibMean_GZ;
    
    unsigned long       _gCurrSensorCapTime;
    unsigned long       _gPrevSensorCapTime;
    float               _gDiffTime;
    float               nSampleFreq;                            // half the sample period expressed in seconds
}SelfFly_T;


/*----------------------------------------------------------------------------------------
 Static Function
 ----------------------------------------------------------------------------------------*/
void _AccelGyro_Initialize();
void _AccelGyro_GetData();
void _AccelGyro_CalculateAngle();
void _AccelGyro_GetMeanSensor();
void _AccelGyro_Calibration();
void _print_Gyro_Signals();


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
    AccelGyroParam_T        *pAccelGyroParam = &(nAccelGyroParam);
    
    Serial.println("");    Serial.println("");    Serial.println("");    Serial.println("");
    Serial.println("****************************************************");
    Serial.println("****************************************************");
    
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
    
    Serial.println("****************************************************");
    Serial.println("****************************************************");
    Serial.println("");    Serial.println("");    Serial.println("");    Serial.println("");
}


void loop()
{
    AccelGyroParam_T        *pAccelGyroParam = &(nAccelGyroParam);

    _gPrevSensorCapTime = _gCurrSensorCapTime;
    _gCurrSensorCapTime = micros();
    
    _gDiffTime = (_gCurrSensorCapTime - _gPrevSensorCapTime) / 1000000.0;
    nSampleFreq = 1000000.0 / ((_gCurrSensorCapTime - _gPrevSensorCapTime));
    
    _AccelGyro_GetData();
    
    delay(50);
    
    _print_Gyro_Signals();
}


void _AccelGyro_Initialize()
{
    nAccelGyroHndl = MPU6050();
    
    Serial.println(F(" Initializing MPU..."));
    nAccelGyroHndl.initialize();
    
    // Verify Vonnection
    Serial.print(F("    Testing device connections..."));
    Serial.println(nAccelGyroHndl.testConnection() ? F("  MPU6050 connection successful") : F("  MPU6050 connection failed"));
    
    nAccelGyroHndl.setI2CMasterModeEnabled(false);
    nAccelGyroHndl.setI2CBypassEnabled(true);
    nAccelGyroHndl.setSleepEnabled(false);
    
    // Calibrate GyroAccel
    #if 0
    _AccelGyro_Calibration();
    #else
    nAccelGyroHndl.setXGyroOffset(65);
    nAccelGyroHndl.setYGyroOffset(-42);
    nAccelGyroHndl.setZGyroOffset(-3);
    nAccelGyroHndl.setXAccelOffset(-73);
    nAccelGyroHndl.setYAccelOffset(-737);
    nAccelGyroHndl.setZAccelOffset(0);
    #endif
    
    // supply your own gyro offsets here, scaled for min sensitivity
    nAccelGyroHndl.setRate(1);                                            // Sample Rate (500Hz = 1Hz Gyro SR / 1+1)
    nAccelGyroHndl.setDLPFMode(MPU6050_DLPF_BW_20);                       // Low Pass filter 20hz
    nAccelGyroHndl.setFullScaleGyroRange(GYRO_FS_PRECISIOM);              // 250? / s (MPU6050_GYRO_FS_250)
    nAccelGyroHndl.setFullScaleAccelRange(ACCEL_FS_PRECISIOM);            // +-2g (MPU6050_ACCEL_FS_2)
    
    Serial.println(F(" MPU Initialized!!!"));
    
    return;
}


void _AccelGyro_GetData()
{
    AccelGyroParam_T        *pAccelGyroParam = &(nAccelGyroParam);
    float                   *pRawGyro = &(pAccelGyroParam->_gRawGyro[X_AXIS]);
    float                   *pRawAccel = &(pAccelGyroParam->_gRawAccel[X_AXIS]);
    const float             *pBaseGyro = &(pAccelGyroParam->nBaseGyro[X_AXIS]);
    
    // Read Gyro and Accelerate Data
    Wire.beginTransmission(MPU6050_ADDRESS_AD0_LOW);
    Wire.write(MPU6050_RA_ACCEL_XOUT_H);  // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDRESS_AD0_LOW, 14, true);  // request a total of 14 registers
    
    pRawAccel[X_AXIS] = (float)(Wire.read()<<8 | Wire.read());               // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
    pRawAccel[Y_AXIS] = (float)(Wire.read()<<8 | Wire.read());               // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    pRawAccel[Z_AXIS] = (float)(Wire.read()<<8 | Wire.read());               // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    pAccelGyroParam->_gRawTemp = (float)(Wire.read()<<8 | Wire.read());       // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)        340 per degrees Celsius, -512 at 35 degrees
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
    AccelGyroParam_T        *pAccelGyroParam = &(nAccelGyroParam);
    const float             *pRawGyro = &(pAccelGyroParam->_gRawGyro[X_AXIS]);
    const float             *pRawAccel = &(pAccelGyroParam->_gRawAccel[X_AXIS]);
    const float             *pBaseGyro = &(pAccelGyroParam->nBaseGyro[X_AXIS]);
    float                   *pFineAngle = &(pAccelGyroParam->nFineAngle[X_AXIS]);
    static float            nGyroAngle[3] = {0, };
    float                   nAccelAngle[3] = {0, };
    float                   nGyroDiffAngle[3] = {0, };
    const float             nGyroWeight = _gDiffTime / GYRO_FS;
    
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


void _AccelGyro_GetMeanSensor()
{
    long                i=0;
    long                nAccelBufX=0, nAccelBufY=0, nAccelBufZ=0;
    long                nGyroBufX=0, nGyroBufY=0, nGyroBufZ=0;
    const int           nLoopCnt = 1000;
    
    while (i<(nLoopCnt + 101))
    {
        // read raw accel/gyro measurements from device
        // Read Gyro and Accelerate Data
        Wire.beginTransmission(MPU6050_ADDRESS_AD0_LOW);
        Wire.write(MPU6050_RA_ACCEL_XOUT_H);  // starting with register 0x3B (ACCEL_XOUT_H)
        Wire.endTransmission(false);
        Wire.requestFrom(MPU6050_ADDRESS_AD0_LOW, 14, true);  // request a total of 14 registers
        
        if (i>100 && i<=(nLoopCnt + 100))
        {
            //First 100 measures are discarded
            nAccelBufX += (Wire.read()<<8 | Wire.read());
            nAccelBufY += (Wire.read()<<8 | Wire.read());
            nAccelBufZ += (Wire.read()<<8 | Wire.read());
            Wire.read(); Wire.read();
            nGyroBufX += (Wire.read()<<8 | Wire.read());
            nGyroBufY += (Wire.read()<<8 | Wire.read());
            nGyroBufZ += (Wire.read()<<8 | Wire.read());
        }
        
        if (i==(nLoopCnt + 100))
        {
            nCalibMean_AX = nAccelBufX / nLoopCnt;
            nCalibMean_AY = nAccelBufY / nLoopCnt;
            nCalibMean_AZ = nAccelBufZ / nLoopCnt;
            nCalibMean_GX = nGyroBufX / nLoopCnt;
            nCalibMean_GY = nGyroBufY / nLoopCnt;
            nCalibMean_GZ = nGyroBufZ / nLoopCnt;
        }
        
        i++;
        delay(2); //Needed so we don't get repeated measures
    }
}


// Reference
// http://wired.chillibasket.com/2015/01/calibrating-mpu6050/
void _AccelGyro_Calibration()
{
    int             nOffset_AX = 0, nOffset_AY = 0, nOffset_AZ;
    int             nOffset_GX = 0, nOffset_GY = 0, nOffset_GZ = 0;
    const int       nAccelDeadZone = 8;     //Acelerometer error allowed, make it lower to get more precision, but sketch may not converge  (default:8)
    const int       nGyroDeadZone = 1;     //Giro error allowed, make it lower to get more precision, but sketch may not converge  (default:1)
    
    _AccelGyro_GetMeanSensor();
    
    nOffset_AX = -nCalibMean_AX / 8;
    nOffset_AY = -nCalibMean_AY / 8;
    nOffset_AZ = (16384 - nCalibMean_AZ) / 8;
    nOffset_GX = -nCalibMean_GX / 4;
    nOffset_GY = -nCalibMean_GY / 4;
    nOffset_GZ = -nCalibMean_GZ / 4;
    
    while (1)
    {
        int         ready = 0;
        
        nAccelGyroHndl.setXAccelOffset(nOffset_AX);
        nAccelGyroHndl.setYAccelOffset(nOffset_AY);
        nAccelGyroHndl.setZAccelOffset(nOffset_AZ);
        nAccelGyroHndl.setXGyroOffset(nOffset_GX);
        nAccelGyroHndl.setYGyroOffset(nOffset_GY);
        nAccelGyroHndl.setZGyroOffset(nOffset_GZ);
        
        _AccelGyro_GetMeanSensor();
        Serial.print("...");
        
        if (abs(nCalibMean_AX) <= nAccelDeadZone) ready++;
        else nOffset_AX -= (nCalibMean_AX / nAccelDeadZone);
        
        if (abs(nCalibMean_AY)<=nAccelDeadZone) ready++;
        else nOffset_AY -= (nCalibMean_AY / nAccelDeadZone);
        
        if (abs(16384-nCalibMean_AZ)<=nAccelDeadZone) ready++;
        else nOffset_AZ += ((16384 - nCalibMean_AZ) / nAccelDeadZone);
        
        if (abs(nCalibMean_GX)<=nGyroDeadZone) ready++;
        else nOffset_GX -= (nCalibMean_GX / (nGyroDeadZone + 1));
        
        if (abs(nCalibMean_GY)<=nGyroDeadZone) ready++;
        else nOffset_GY -= (nCalibMean_GY / (nGyroDeadZone + 1));
        
        if (abs(nCalibMean_GZ)<=nGyroDeadZone) ready++;
        else nOffset_GZ -= (nCalibMean_GZ / (nGyroDeadZone + 1));
        
        Serial.print("   Gx: ");
        Serial.print(nOffset_GX);
        Serial.print("   Gy: ");
        Serial.print(nOffset_GY);
        Serial.print("   Gz: ");
        Serial.print(nOffset_GZ);
        
        Serial.print("   Ax: ");
        Serial.print(nOffset_AX);
        Serial.print("   Ay: ");
        Serial.print(nOffset_AY);
        Serial.print("   Az: ");
        Serial.println(nOffset_AZ);
        
        if(6 == ready)
            break;
    }
}


void _print_Gyro_Signals()
{
    Serial.print("            ");
    Serial.print(nAccelGyroParam._gRawGyro[0]);
    Serial.print("            ");
    Serial.print(nAccelGyroParam._gRawGyro[1]);
    Serial.print("            ");
    Serial.print(nAccelGyroParam._gRawGyro[2]);
    
    Serial.print("            ");
    Serial.print(nAccelGyroParam._gRawAccel[0]);
    Serial.print("          ");
    Serial.print(nAccelGyroParam._gRawAccel[1]);
    Serial.print("           ");
    Serial.println(nAccelGyroParam._gRawAccel[2]);
}

