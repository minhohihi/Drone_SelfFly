/*----------------------------------------------------------------------------------------
 Constant Definitions
 ----------------------------------------------------------------------------------------*/
#define __DEBUG__                           (0)
#define __GYROSCOPE_ENABLED__               (1)
#define __COMPASS_ENABLED__                 (0)
#define __BAROMETER_ENABLED__               (0)

// Arduino Pin configuration
#define PIN_GY86_EXT_INTERRUPT              (13)
#define PIN_RESERVED_12                     (12)
#define PIN_ESC_1                           (11)
#define PIN_ESC_2                           (10)
#define PIN_ESC_3                           (9)
#define PIN_ESC_4                           (8)
#define PIN_RESERVED_07                     (7)
#define PIN_RC_CH1                          (6)
#define PIN_RC_CH2                          (5)
#define PIN_RC_CH3                          (4)
#define PIN_RC_CH4                          (3)
#define PIN_RC_CH5                          (2)
#define PIN_RESERVED_01                     (1)
#define PIN_RESERVED_00                     (0)
#define PIN_CHECK_POWER_STAT                (A0)

// ESC configuration
#define ESC_MIN                             (800)
#define ESC_MAX                             (2200)
#define ESC_TAKEOFF_OFFSET                  (900)
#define ESC_ARM_DELAY                       (1000)

// RC configuration
#define RC_HIGH_CH1                         (1900)
#define RC_LOW_CH1                          (1050)
#define RC_HIGH_CH2                         (1900)
#define RC_LOW_CH2                          (1050)
#define RC_HIGH_CH3                         (1900)
#define RC_LOW_CH3                          (1050)
#define RC_HIGH_CH4                         (1900)
#define RC_LOW_CH4                          (1050)
#define RC_HIGH_CH5                         (1900)
#define RC_LOW_CH5                          (1050)

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


/*----------------------------------------------------------------------------------------
 File Inclusions
 ----------------------------------------------------------------------------------------*/
#include <Servo.h>
#include <I2Cdev.h>
#include <PinChangeInt.h>
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include <Wire.h>
#endif

#if __GYROSCOPE_ENABLED__
    //#include <MPU6050.h>
    #include <MPU6050_6Axis_MotionApps20.h>
#endif

#if __COMPASS_ENABLED__
    #include <HMC5883L.h>
#endif

#if __BAROMETER_ENABLED__
    #include <MS5611.h>
#endif


/*----------------------------------------------------------------------------------------
 Macro Definitions
 ----------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------
 Type Definitions
 ----------------------------------------------------------------------------------------*/
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

#if __COMPASS_ENABLED__
typedef struct _CompassParam_T
{
    Vector              nRawData;
    Vector              nNormData;
}CompassParam_T;
#endif

#if __BAROMETER_ENABLED__
typedef struct _BaroParam_T
{
    uint32_t            nRawTemp;                               // Raw Temperature Data
    uint32_t            nRawPressure;                           // Raw Pressure Data
    double              nRealTemperature;                       // Real Temperature Data
    long                nRealPressure;                          // Real Pressure Data
    float               nAbsoluteAltitude;                      // Estimated Absolute Altitude
    float               nRelativeAltitude;                      // Estimated Relative Altitude
    double              nRefBarometerVal;                       // Reference Barometer Value
}BaroParam_T;
#endif

/*----------------------------------------------------------------------------------------
 Static Function
 ----------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------
 Static Variable
 ----------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------
 Global Variable
 ----------------------------------------------------------------------------------------*/
volatile bool           nMPUInterruptFlag = false;
bool                    nInterruptLockFlag = false;             // Interrupt lock
bool                    nDMPReadyFlag = false;                  // set true if DMP init was successful
uint16_t                nPacketSize;                            // expected DMP packet size (default is 42 bytes)


float                   nRC_Ch[5] = {0, };                      // RC channel inputs
unsigned long           nRCPrevChangeTime1 = micros();
unsigned long           nRCPrevChangeTime2 = micros();
unsigned long           nRCPrevChangeTime3 = micros();
unsigned long           nRCPrevChangeTime4 = micros();
unsigned long           nRCPrevChangeTime5 = micros();

Servo                   nESC[4];
#if __GYROSCOPE_ENABLED__
    MPU6050             nMPU;                                   // MPU6050 Gyroscope Interface
    Quaternion          nQuater;
    VectorFloat         nGravity;
#endif

#if __COMPASS_ENABLED__
    HMC5883L            nCompass;                               // HMC58x3 Compass Interface
    CompassParam_T      nCompassParam;
#endif

#if __BAROMETER_ENABLED__
    MS5611              nBarometer;                             // MS5611 Barometer Interface
    BaroParam_T         nBaroParam;
#endif


/*----------------------------------------------------------------------------------------
 Function Implementation
 ----------------------------------------------------------------------------------------*/
void setup()
{
    uint8_t                 nDevStatus;                         // return status after each device operation (0 = success, !0 = error)
    
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
    
    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
    // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.
    
    #if __GYROSCOPE_ENABLED__
    {
        // initialize MPU
        Serial.println(F("Initializing MPU..."));
        nMPU.setI2CMasterModeEnabled(false);
        nMPU.setI2CBypassEnabled(true);
        nMPU.setSleepEnabled(false);
        nMPU.initialize();
        
        // verify connection
        Serial.println(F("Testing device connections..."));
        Serial.println(nMPU.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
        
        // wait for ready
        //Serial.println(F("\nSend any character to begin DMP programming and demo: "));
        //while (Serial.available() && Serial.read()); // empty buffer
        //while (!Serial.available());                                 // wait for data
        //while (Serial.available() && Serial.read()); // empty buffer again
        
        // load and configure the DMP
        Serial.println(F("Initializing DMP..."));
        nDevStatus = nMPU.dmpInitialize();
        
        // supply your own gyro offsets here, scaled for min sensitivity
        nMPU.setXGyroOffset(220);
        nMPU.setYGyroOffset(76);
        nMPU.setZGyroOffset(-85);
        nMPU.setZAccelOffset(1788); // 1688 factory default for my test chip
        
        // make sure it worked (returns 0 if so)
        if(0 == nDevStatus)
        {
            // turn on the DMP, now that it's ready
            Serial.println(F("Enabling DMP..."));
            nMPU.setDMPEnabled(true);
            
            // enable Arduino interrupt detection
            Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
            pinMode(PIN_GY86_EXT_INTERRUPT, INPUT);
            digitalWrite(PIN_GY86_EXT_INTERRUPT, HIGH);
            //attachInterrupt(digitalPinToInterrupt(2), dmpDataReady, RISING);
            PCintPort::attachInterrupt(PIN_GY86_EXT_INTERRUPT, dmpDataReady, RISING);
            
            // set our DMP Ready flag so the main loop() function knows it's okay to use it
            Serial.println(F("DMP ready! Waiting for first interrupt..."));
            nDMPReadyFlag = true;
            
            // get expected DMP packet size for later comparison
            nPacketSize = nMPU.dmpGetFIFOPacketSize();
        }
        else
        {
            // ERROR!
            // 1 = initial memory load failed
            // 2 = DMP configuration updates failed
            // (if it's going to break, usually the code will be 1)
            Serial.print(F("DMP Initialization failed (code "));
            Serial.print(nDevStatus);
            Serial.println(F(")"));
        }
    }
    #endif

    #if __COMPASS_ENABLED__
    {
        // initialize Compass
        Serial.println(F("Initializing Compass..."));
        
        while(!nCompass.begin())
        {
            Serial.println("Can Not Find a Valid HMC5883L (Compass), Check H/W");
            delay(500);
        }
        
        // Set Measurement Range
        nCompass.setRange(HMC5883L_RANGE_1_3GA);
        
        // Set Measurement Mode
        nCompass.setMeasurementMode(HMC5883L_CONTINOUS);
        
        // Set Data Rate
        nCompass.setDataRate(HMC5883L_DATARATE_75HZ);
        
        // Set Number of Samples Averaged
        nCompass.setSamples(HMC5883L_SAMPLES_8);
        
        // Set Calibration Offset (Ref. HMC5883L_calibraton.ion)
        nCompass.setOffset(0, 0);
    }
    #endif

    #if __BAROMETER_ENABLED__
    {
        // initialize Barometer
        Serial.println(F("Initializing Barometer..."));
        
        while(!nBarometer.begin())
        {
            Serial.println("Can Not Find a Valid MS5611 (Barometer), Check H/W");
            delay(500);
        }
        
        // Set Measurement Range
        nBaroParam.nRefBarometerVal = nBarometer.readPressure();
        
        Serial.print("Barometer Oversampling: ");
        Serial.println(nBarometer.getOversampling());
    }
    #endif
    
    initRC();
    initESCs();
}

void loop()
{
    bool                    bSkipFlag = false;
    uint8_t                 nMPUInterruptStat;                      // holds actual interrupt status byte from MPU
    uint16_t                nFIFOCnt;                               // count of all bytes currently in FIFO
    uint8_t                 nFIFOBuf[64];                           // FIFO storage buffer
    int                     nThrottle[4];                           // throttles
    int16_t                 nGyro[3];                               // Gyroscope Value
    float                   nRPY[3];                                // yaw pitch roll values
    
    ////////////////////////////////// Get angle & gyro ///////////////////////////////////////////////////////
    // if programming failed, don't try to do anything
    if(!nDMPReadyFlag)
    {
        Serial.println("DMP Not Ready    Loop Run Fails");
        return;
    }
    
    // wait for MPU interrupt or extra packet(s) available
    while(!nMPUInterruptFlag && nFIFOCnt < nPacketSize)
    {
        // other program behavior stuff here
        // .
        // .
        // .
        // if you are really paranoid you can frequently test in between other
        // stuff to see if nMPUInterruptFlag is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data
        // .
        // .
        // .
    }
    
    #if __GYROSCOPE_ENABLED__
    // reset interrupt flag and get INT_STATUS byte
    nMPUInterruptFlag = false;
    nMPUInterruptStat = nMPU.getIntStatus();
    
    // get current FIFO count
    nFIFOCnt = nMPU.getFIFOCount();
    
    // check for overflow (this should never happen unless our code is too inefficient)
    if((nMPUInterruptStat & 0x10) || nFIFOCnt == 1024)
    {
        // reset so we can continue cleanly
        nMPU.resetFIFO();
        //Serial.println(F("FIFO overflow!"));
        
        // otherwise, check for DMP data ready interrupt (this should happen frequently)
        
        bSkipFlag = true;
    }
    else if (nMPUInterruptStat & 0x02)
    {
        // wait for correct available data length, should be a VERY short wait
        while (nFIFOCnt < nPacketSize)
            nFIFOCnt = nMPU.getFIFOCount();
        
        // read a packet from FIFO
        nMPU.getFIFOBytes(nFIFOBuf, nPacketSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        nFIFOCnt -= nPacketSize;
        
        nMPU.dmpGetGyro(nGyro, nFIFOBuf);
        nMPU.dmpGetQuaternion(&nQuater, nFIFOBuf);
        nMPU.dmpGetGravity(&nGravity, &nQuater);
        nMPU.dmpGetYawPitchRoll(nRPY, &nQuater, &nGravity);
    }
    #endif

    #if __COMPASS_ENABLED__
    nCompassParam.nRawData = nCompass.readRaw();
    nCompassParam.nNormData = nCompass.readNormalize();
    #endif

    #if __BAROMETER_ENABLED__
    // Read raw values
    nBaroParam.nRawTemp = nBarometer.readRawTemperature();
    nBaroParam.nRawPressure = nBarometer.readRawPressure();
    
    // Read true temperature & Pressure
    nBaroParam.nRealTemperature = nBarometer.readTemperature();
    nBaroParam.nRealPressure = nBarometer.readPressure();
    
    // Calculate altitude
    nBaroParam.nAbsoluteAltitude = nBarometer.getAltitude(nBaroParam.nRealPressure);
    nBaroParam.nRelativeAltitude = nBarometer.getAltitude(nBaroParam.nRealPressure, nBaroParam.nRefBarometerVal);
    #endif
    
    if(false == bSkipFlag)
    {
        AxisErrRate_T           nPitch = {0, };
        AxisErrRate_T           nRoll = {0, };
        AxisErrRate_T           nYaw = {0, };
        
        // PID Computation
        CalculatePID(&nRoll, &nPitch, &nYaw, nGyro, nRPY);
        
        //Throttle Calculation
        CalculateThrottleVal(&nRoll, &nPitch, &nYaw, nThrottle);
        
        //Update BLDCs
        UpdateESCs(nThrottle);
    }
    
    #if __DEBUG__
    _print_RPY_Signals(nRPY);
    _print_Throttle_Signals(nThrottle);
    _print_Gyro_Signals(nGyro);
    _print_RC_Signals();
    Serial.println(" ");
    #endif
}


inline void arm()
{
    nESC[1].writeMicroseconds(ESC_MIN);
    nESC[2].writeMicroseconds(ESC_MIN);
    nESC[3].writeMicroseconds(ESC_MIN);
    nESC[4].writeMicroseconds(ESC_MIN);
    
    delay(ESC_ARM_DELAY);
}


inline void initESCs()
{
    nESC[1].attach(PIN_ESC_1);
    nESC[2].attach(PIN_ESC_2);
    nESC[3].attach(PIN_ESC_3);
    nESC[4].attach(PIN_ESC_4);
    
    delay(1000);
    
    arm();
}


inline void initRC()
{
    pinMode(PIN_CHECK_POWER_STAT, OUTPUT);
    digitalWrite(PIN_CHECK_POWER_STAT, HIGH);
    
    PCintPort::attachInterrupt(PIN_RC_CH1, nRCInterrupt_CB1, CHANGE);
    PCintPort::attachInterrupt(PIN_RC_CH2, nRCInterrupt_CB2, CHANGE);
    PCintPort::attachInterrupt(PIN_RC_CH3, nRCInterrupt_CB3, CHANGE);
    PCintPort::attachInterrupt(PIN_RC_CH4, nRCInterrupt_CB4, CHANGE);
    PCintPort::attachInterrupt(PIN_RC_CH5, nRCInterrupt_CB5, CHANGE);
}


inline void CalculatePID(struct _AxisErrRate_T *pRoll, struct _AxisErrRate_T *pPitch, struct _AxisErrRate_T *pYaw, int16_t nGyro[3], float nRPY[3])
{
    static float            nRC_PrevCh[5] = {0, };              // Filter variables
    static float            nPrevRPY[3];

    acquireLock();
    
    nRC_Ch[1] = floor(nRC_Ch[1] / ROUNDING_BASE) * ROUNDING_BASE;
    nRC_Ch[2] = floor(nRC_Ch[2] / ROUNDING_BASE) * ROUNDING_BASE;
    nRC_Ch[4] = floor(nRC_Ch[4] / ROUNDING_BASE) * ROUNDING_BASE;
    
    nRC_Ch[1] = map(nRC_Ch[1], RC_LOW_CH1, RC_HIGH_CH1, ROLL_ANG_MIN, ROLL_ANG_MAX) - 1;
    nRC_Ch[2] = map(nRC_Ch[2], RC_LOW_CH2, RC_HIGH_CH2, PITCH_ANG_MIN, PITCH_ANG_MAX) - 1;
    nRC_Ch[4] = map(nRC_Ch[4], RC_LOW_CH4, RC_HIGH_CH4, YAW_RATE_MIN, YAW_RATE_MAX);
    
    if((nRC_Ch[1] < ROLL_ANG_MIN) || (nRC_Ch[1] > ROLL_ANG_MAX))
        nRC_Ch[1] = nRC_PrevCh[1];
    if((nRC_Ch[2] < PITCH_ANG_MIN) || (nRC_Ch[2] > PITCH_ANG_MAX))
        nRC_Ch[2] = nRC_PrevCh[2];
    if((nRC_Ch[4] < YAW_RATE_MIN) || (nRC_Ch[4] > YAW_RATE_MAX))
        nRC_Ch[4] = nRC_PrevCh[4];
    
    nRC_PrevCh[1] = nRC_Ch[1];
    nRC_PrevCh[2] = nRC_Ch[2];
    nRC_PrevCh[4] = nRC_Ch[4];
    
    nRPY[1] = nRPY[1] * 180 / M_PI + PITCH_ANG_OFFSET;
    nRPY[2] = nRPY[2] * 180 / M_PI + ROLL_ANG_OFFSET;
    
    if(abs(nRPY[1] - nPrevRPY[1]) > 30)
        nRPY[1] = nPrevRPY[1];
    
    if(abs(nRPY[2] - nPrevRPY[2]) > 30)
        nRPY[2] = nPrevRPY[2];
    
    nPrevRPY[1] = nRPY[1];
    nPrevRPY[2] = nRPY[2];
    
    //ROLL control
    pRoll->nAngleErr = nRC_Ch[1] - nRPY[2];
    pRoll->nCurrErrRate = pRoll->nAngleErr * ROLL_OUTER_P_GAIN - nGyro[0];
    pRoll->nP_ErrRate = pRoll->nCurrErrRate * ROLL_INNER_P_GAIN;
    pRoll->nI_ErrRate = pRoll->nI_ErrRate + (pRoll->nCurrErrRate * ROLL_INNER_I_GAIN) * SAMPLING_TIME;
    Clip3Float(&pRoll->nI_ErrRate, -100, 100);
    pRoll->nD_ErrRate = (pRoll->nCurrErrRate - pRoll->nPrevErrRate) / SAMPLING_TIME * ROLL_INNER_D_GAIN;
    pRoll->nPrevErrRate = pRoll->nCurrErrRate;
    pRoll->nBalance = pRoll->nP_ErrRate + pRoll->nI_ErrRate + pRoll->nD_ErrRate;

    //PITCH control
    pPitch->nAngleErr = nRC_Ch[2] - nRPY[1];
    pPitch->nCurrErrRate = pPitch->nAngleErr * PITCH_OUTER_P_GAIN + nGyro[1];
    pPitch->nP_ErrRate = pPitch->nCurrErrRate * PITCH_INNER_P_GAIN;
    pPitch->nI_ErrRate = pPitch->nI_ErrRate + (pPitch->nCurrErrRate * PITCH_INNER_I_GAIN) * SAMPLING_TIME;
    Clip3Float(&pPitch->nI_ErrRate, -100, 100);
    pPitch->nD_ErrRate = (pPitch->nCurrErrRate - pPitch->nPrevErrRate) / SAMPLING_TIME * PITCH_INNER_D_GAIN;
    pPitch->nPrevErrRate = pPitch->nCurrErrRate;
    pPitch->nBalance = pPitch->nP_ErrRate + pPitch->nI_ErrRate + pPitch->nD_ErrRate;

    //YAW control
    pYaw->nCurrErrRate = nRC_Ch[4] - nGyro[2];
    pYaw->nP_ErrRate = pYaw->nCurrErrRate * YAW_P_GAIN;
    pYaw->nI_ErrRate = pYaw->nI_ErrRate + pYaw->nCurrErrRate * YAW_I_GAIN * SAMPLING_TIME;
    Clip3Float(&pYaw->nI_ErrRate, -50, 50);
    pYaw->nTorque = pYaw->nP_ErrRate + pYaw->nI_ErrRate;
    
    releaseLock();
}


inline void CalculateThrottleVal(struct _AxisErrRate_T *pRoll, struct _AxisErrRate_T *pPitch, struct _AxisErrRate_T *pYaw, int nThrottle[4])
{
    float                   nEstimatedThrottle = 0.0f;
    static float            nPrevEstimatedThrottle = 0.0f;
    
    acquireLock();
    
    nRC_Ch[3] = floor(nRC_Ch[3] / ROUNDING_BASE) * ROUNDING_BASE;
    nEstimatedThrottle = (float)(map(nRC_Ch[3], RC_LOW_CH3, RC_HIGH_CH3, ESC_MIN, ESC_MAX));
    if((nEstimatedThrottle < ESC_MIN) || (nEstimatedThrottle > ESC_MAX))
        nEstimatedThrottle = nPrevEstimatedThrottle;
    
    nPrevEstimatedThrottle = nEstimatedThrottle;
    
    releaseLock();
    
    if(nEstimatedThrottle > ESC_TAKEOFF_OFFSET)
    {
        nThrottle[0] = ( pPitch->nBalance - pRoll->nBalance) * 0.5 + pYaw->nTorque + nEstimatedThrottle;
        nThrottle[1] = (-pPitch->nBalance - pRoll->nBalance) * 0.5 - pYaw->nTorque + nEstimatedThrottle;
        nThrottle[2] = (-pPitch->nBalance + pRoll->nBalance) * 0.5 + pYaw->nTorque + nEstimatedThrottle;
        nThrottle[3] = ( pPitch->nBalance + pRoll->nBalance) * 0.5 - pYaw->nTorque + nEstimatedThrottle;
        
        Clip3Int(&nThrottle[0], ESC_MIN, ESC_MAX);
        Clip3Int(&nThrottle[1], ESC_MIN, ESC_MAX);
        Clip3Int(&nThrottle[2], ESC_MIN, ESC_MAX);
        Clip3Int(&nThrottle[3], ESC_MIN, ESC_MAX);
    }
    else
    {
        nThrottle[0] = ESC_MIN;
        nThrottle[1] = ESC_MIN;
        nThrottle[2] = ESC_MIN;
        nThrottle[3] = ESC_MIN;
    }
}


inline void UpdateESCs(int nThrottle[4])
{
    //nESC[1].writeMicroseconds(nThrottle[0]);
    //nESC[3].writeMicroseconds(nThrottle[1]);
    //nESC[2].writeMicroseconds(nThrottle[2]);
    //nESC[4].writeMicroseconds(nThrottle[3]);
}

inline void nRCInterrupt_CB1()
{
    if(!nInterruptLockFlag)
        nRC_Ch[1] = micros() - nRCPrevChangeTime1;
    
    nRCPrevChangeTime1 = micros();
}


inline void nRCInterrupt_CB2()
{
    if(!nInterruptLockFlag)
        nRC_Ch[2] = micros() - nRCPrevChangeTime2;
    
    nRCPrevChangeTime2 = micros();
}


inline void nRCInterrupt_CB3()
{
    if(!nInterruptLockFlag)
        nRC_Ch[3] = micros() - nRCPrevChangeTime3;
    
    nRCPrevChangeTime3 = micros();
}


inline void nRCInterrupt_CB4()
{
    if(!nInterruptLockFlag)
        nRC_Ch[4] = micros() - nRCPrevChangeTime4;
    
    nRCPrevChangeTime4 = micros();
}


inline void nRCInterrupt_CB5()
{
    if(!nInterruptLockFlag)
        nRC_Ch[5] = micros() - nRCPrevChangeTime5;
    
    nRCPrevChangeTime5 = micros();
}


inline void acquireLock()
{
    nInterruptLockFlag = true;
}


inline void releaseLock()
{
    nInterruptLockFlag = false;
}


void dmpDataReady()
{
    nMPUInterruptFlag = true;
}


//ERR_I limit to prevent divergence
void Clip3Float(float *value, int MIN, int MAX)
{
    if(*value < MIN)
    {
        *value = MIN;
    }
    else if(*value > MAX)
    {
        *value = MAX;
    }
}


void Clip3Int(int *value, int MIN, int MAX)
{
    if(*value < MIN)
    {
        *value = MIN;
    }
    else if(*value > MAX)
    {
        *value = MAX;
    }
}

#if __DEBUG__
void _print_RC_Signals()
{
    Serial.print("   //   RC_Roll:");
    if(nRC_Ch[1] - 1480 < 0)Serial.print("<<<");
    else if(nRC_Ch[1] - 1520 > 0)Serial.print(">>>");
    else Serial.print("-+-");
    Serial.print(nRC_Ch[1]);
    
    Serial.print("   RC_Pitch:");
    if(nRC_Ch[2] - 1480 < 0)Serial.print("^^^");
    else if(nRC_Ch[2] - 1520 > 0)Serial.print("vvv");
    else Serial.print("-+-");
    Serial.print(nRC_Ch[2]);
    
    Serial.print("   RC_Throttle:");
    if(nRC_Ch[3] - 1480 < 0)Serial.print("vvv");
    else if(nRC_Ch[3] - 1520 > 0)Serial.print("^^^");
    else Serial.print("-+-");
    Serial.print(nRC_Ch[3]);
    
    Serial.print("   RC_Yaw:");
    if(nRC_Ch[4] - 1480 < 0)Serial.print("<<<");
    else if(nRC_Ch[4] - 1520 > 0)Serial.print(">>>");
    else Serial.print("-+-");
    Serial.print(nRC_Ch[4]);
    
    Serial.print("   RC_Gear:");
    if(nRC_Ch[5] - 1480 < 0)Serial.print("<<<");
    else if(nRC_Ch[5] - 1520 > 0)Serial.print(">>>");
    else Serial.print("-+-");
    Serial.print(nRC_Ch[5]);
}

void _print_Gyro_Signals(int16_t nGyro[3])
{
    Serial.print("   //    Roll Gyro : ");
    Serial.print(nGyro[0]);
    Serial.print("   Pitch Gyro : ");
    Serial.print(nGyro[1]);
    Serial.print("   Yaw Gyro : ");
    Serial.print(nGyro[2]);
}

void _print_Throttle_Signals(int nThrottle[4])
{
    Serial.print("   //    Thrt1 : ");
    Serial.print(nThrottle[1]);
    Serial.print("  Thrt2 : ");
    Serial.print(nThrottle[2]);
    Serial.print("  Thrt3 : ");
    Serial.print(nThrottle[2]);
    Serial.print("  Thrt4 : ");
    Serial.print(nThrottle[3]);
}


void _print_RPY_Signals(float nRPY[3])
{
    Serial.print("   //    Roll : ");
    Serial.print(nRPY[2]);
    Serial.print("   Pitch : ");
    Serial.print(nRPY[1]);
    Serial.print("   Yaw : ");
    Serial.print(nRPY[0]);
}
#endif
