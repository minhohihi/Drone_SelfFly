/*----------------------------------------------------------------------------------------
 Constant Definitions
 ----------------------------------------------------------------------------------------*/
#define __DEBUG__                           (1)
#define __GYROSCOPE_ENABLED__               (1)
#define __COMPASS_ENABLED__                 (1)
#define __BAROMETER_ENABLED__               (1)

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
    #include <MS561101BA.h>
#endif


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
    float               nRealPressure;                          // Real Pressure Data
    float               nAvgpressure;                           // Average Pressure Data
    float               nAbsoluteAltitude;                      // Estimated Absolute Altitude
    float               nRelativeAltitude;                      // Estimated Relative Altitude
    double              nRefTemperature;                        // Reference Temperature Value
    int32_t             nRefPressure;                           // Reference Pressure Value
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
    HMC5883L            nCompass;                               // HMC5883 Compass Interface
    CompassParam_T      nCompassParam;
#endif

#if __BAROMETER_ENABLED__
    MS5611              nBarometer;                             // MS5611 Barometer Interface
    BaroParam_T         nBaroParam;
    MS561101BA          nBarometerBA;
#endif


/*----------------------------------------------------------------------------------------
 Function Implementation
 ----------------------------------------------------------------------------------------*/
void MagHMC5883Int()
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

int MagX,MagY,MagZ;
float MagXf,MagYf,MagZf;
float c_magnetom_x;
float c_magnetom_y;
float c_magnetom_z;
int M_X_MIN = -270;    //-654  -693   -688
int M_X_MAX = 585;     //185   209    170
int M_Y_MIN = -600;    //-319  -311   -310
int M_Y_MAX = 260;     //513   563    546
int M_Z_MIN = -425;    //-363  -374   -377
int M_Z_MAX = 285;     //386   429    502

void Mag5883Read()
{
    int i = 0;
    byte result[6];

    //Tell the HMC5883 where to begin reading data
    Wire.beginTransmission(HMC5883L_ADDRESS);
    Wire.write(0x03);                                           // Select register 3, X MSB register
    Wire.endTransmission();

    //Read data from each axis, 2 registers per axis
    Wire.requestFrom(HMC5883L_ADDRESS, 6);
    while(Wire.available())
    { 
        result[i] = Wire.read();
        i++;
    }
    Wire.endTransmission();

    MagX = ((result[0] << 8) | result[1]);                      //offset + 1.05
    MagZ = ((result[2] << 8) | result[3])*-1;                   // + 0.05
    MagY = ((result[4] << 8) | result[5])*-1;                   // - 0.55
    MagXf = MagXf + (MagX - MagXf)*0.55;
    MagYf = MagYf + (MagY - MagYf)*0.55;
    MagZf = MagZf + (MagZ - MagZf)*0.55;

    // adjust for  compass axis offsets/sensitivity differences by scaling to +/-5 range
    c_magnetom_x = ((float)(MagXf - M_X_MIN) / (M_X_MAX - M_X_MIN))*10.0 - 5.0;
    c_magnetom_y = ((float)(MagYf - M_Y_MIN) / (M_Y_MAX - M_Y_MIN))*10.0 - 5.0;
    c_magnetom_z = ((float)(MagZf - M_Z_MIN) / (M_Z_MAX - M_Z_MIN))*10.0 - 5.0;

    //Serialprint("Compass: "); Serialprint(c_magnetom_x);
    //Serialprint(" "); Serialprint(c_magnetom_y);
    //Serialprint(" "); Serialprint(c_magnetom_z);
}

 
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
    
    #if __DEBUG__
    Serial.begin(115200);
    Serial.flush();
    
    while(!Serial); // wait for Leonardo enumeration, others continue immediately
    #endif
    
    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
    // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.
    
    #if __GYROSCOPE_ENABLED__
    {
        // initialize MPU
        Serialprintln(F("Initializing MPU..."));
        nMPU.initialize();
        
        // verify connection
        Serialprintln(F("Testing device connections..."));
        Serialprintln(nMPU.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
        
        // wait for ready
        //Serialprintln(F("\nSend any character to begin DMP programming and demo: "));
        //while (Serialavailable() && Serialread()); // empty buffer
        //while (!Serialavailable());                                 // wait for data
        //while (Serialavailable() && Serialread()); // empty buffer again
        
        // load and configure the DMP
        Serialprintln(F("Initializing DMP..."));
        nDevStatus = nMPU.dmpInitialize();

        nMPU.setI2CMasterModeEnabled(false);
        nMPU.setI2CBypassEnabled(true);
        nMPU.setSleepEnabled(false);
        
        // supply your own gyro offsets here, scaled for min sensitivity
        nMPU.setXGyroOffset(220);
        nMPU.setYGyroOffset(76);
        nMPU.setZGyroOffset(-85);
        nMPU.setZAccelOffset(1788); // 1688 factory default for my test chip
        
        // make sure it worked (returns 0 if so)
        if(0 == nDevStatus)
        {
            // turn on the DMP, now that it's ready
            Serialprintln(F("Enabling DMP..."));
            nMPU.setDMPEnabled(true);
            
            // enable Arduino interrupt detection
            Serialprintln(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
            pinMode(PIN_GY86_EXT_INTERRUPT, INPUT);
            digitalWrite(PIN_GY86_EXT_INTERRUPT, HIGH);
            PCintPort::attachInterrupt(PIN_GY86_EXT_INTERRUPT, dmpDataReady, RISING);
            
            // set our DMP Ready flag so the main loop() function knows it's okay to use it
            Serialprintln(F("DMP ready! Waiting for first interrupt..."));
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
            Serialprint(F("DMP Initialization failed (code "));
            Serialprint(nDevStatus);
            Serialprintln(F(")"));
        }
    }
    #endif

    #if __COMPASS_ENABLED__
    {
        // initialize Compass
        Serialprintln(F("Initializing Compass..."));

        #if 0
            while(!nCompass.begin())
            {
                Serialprintln("Can Not Find a Valid HMC5883L (Compass), Check H/W");
                delay(500);
            }
            
            // Set Number of Samples Averaged
            nCompass.setSamples(HMC5883L_SAMPLES_8);

            // Set Data Rate
            nCompass.setDataRate(HMC5883L_DATARATE_15HZ);
            
            // Set Measurement Range
            nCompass.setRange(HMC5883L_RANGE_1_3GA);

            // Set Measurement Mode
            nCompass.setMeasurementMode(HMC5883L_CONTINOUS);
            
            // Set Calibration Offset (Ref. HMC5883L_calibraton.ion)
            nCompass.setOffset(0, 0);
        #else
            MagHMC5883Int();
        #endif
    }
    #endif

    #if __BAROMETER_ENABLED__
    {
        // initialize Barometer
        Serialprintln(F("Initializing Barometer..."));

        #if 0
            while(!nBarometer.begin())
            {
                Serialprintln("Can Not Find a Valid MS5611 (Barometer), Check H/W");
                delay(500);
            }
    
            nBarometer.setOversampling(MS5611_ULTRA_LOW_POWER);
            
            // Set Measurement Range
            nBaroParam.nRefTemperature = nBarometer.readTemperature();
            nBaroParam.nRefPressure = nBarometer.readPressure();
        #else
            nBarometerBA = MS561101BA();
            nBarometerBA.init(MS561101BA_ADDR_CSB_LOW);
        #endif
        
        Serialprint("Barometer Oversampling: ");
        Serialprintln(nBarometer.getOversampling());
    }
    #endif
    
    initRC();
    initESCs();
}


void loop()
{
    bool                    bSkipFlag = false;
    bool                    bGetCompassSkipFlag = false;
    bool                    bGetBarometerSkipFlag = false;
    uint8_t                 nMPUInterruptStat;                                          // holds actual interrupt status byte from MPU
    uint16_t                nFIFOCnt;                                                   // count of all bytes currently in FIFO
    uint8_t                 nFIFOBuf[64];                                               // FIFO storage buffer
    int                     nThrottle[4] = {ESC_MIN, ESC_MIN, ESC_MIN, ESC_MIN};        // throttles
    int16_t                 nGyro[3];                                                   // Gyroscope Value
    float                   nRPY[3];                                                    // yaw pitch roll values
    float                   nStartTime = 0.0f, nEndTime = 0.0f;

    nStartTime = micros();
    
    ////////////////////////////////// Get angle & gyro ///////////////////////////////////////////////////////
    // if programming failed, don't try to do anything
    if(!nDMPReadyFlag)
    {
        Serialprintln("DMP Not Ready    Loop Run Fails");
        return;
    }
    
    // wait for MPU interrupt or extra packet(s) available
    while(!nMPUInterruptFlag && nFIFOCnt < nPacketSize)
    {
        // Get Compass Sensor Value
        #if __COMPASS_ENABLED__
        {
            if(false == bGetCompassSkipFlag)
            {
                #if 0
                nCompassParam.nRawData = nCompass.readRaw();
                nCompassParam.nNormData = nCompass.readNormalize();
                #else
                Mag5883Read();
                #endif
                bGetCompassSkipFlag = true;
    
                //Serialprint("Compass: "); Serialprint(nCompassParam.nNormData.XAxis);
                //Serialprint(" "); Serialprint(nCompassParam.nNormData.YAxis);
                //Serialprint(" "); Serialprint(nCompassParam.nNormData.ZAxis);
            }
        }
        #endif

        // Get Barometer Sensor Value
        #if __BAROMETER_ENABLED__
        {
            #if 0
            {
                // Read raw values
                nBaroParam.nRawTemp = nBarometer.readRawTemperature();
                nBaroParam.nRawPressure = nBarometer.readRawPressure();
        
                // Read true temperature & Pressure
                nBaroParam.nRealTemperature = nBarometer.readTemperature();
                nBaroParam.nRealPressure = nBarometer.readPressure();
        
                // Calculate altitude
                nBaroParam.nAbsoluteAltitude = nBarometer.getAltitude(nBaroParam.nRealPressure);
                nBaroParam.nRelativeAltitude = nBarometer.getAltitude(nBaroParam.nRealPressure, nBaroParam.nRefPressure);
                //Serialprint("  Barometer: "); Serialprint(nBaroParam.nAbsoluteAltitude);
                //Serialprint(" "); Serialprint(nBaroParam.nRelativeAltitude);
            }
            #else
            {
                if(false == bGetBarometerSkipFlag)
                {
                    nBaroParam.nRealTemperature = nBarometerBA.getTemperature(MS561101BA_OSR_256);
                    nBaroParam.nRealPressure = nBarometerBA.getPressure(MS561101BA_OSR_256);
                    nBarometerBA.pushPressure(nBaroParam.nRealPressure);
                    nBaroParam.nAvgpressure = nBarometerBA.getAvgPressure();
                    nBaroParam.nAbsoluteAltitude = nBarometerBA.getAltitude(nBaroParam.nAvgpressure, nBaroParam.nRealTemperature);
                    bGetBarometerSkipFlag = true;
                    
                    //Serialprint("  Barometer: "); Serialprint(nBaroParam.nRealTemperature);
                    //Serialprint(" "); Serialprint(nBaroParam.nRealPressure);
                    //Serialprint(" "); Serialprint(nBaroParam.nAvgpressure);
                    //Serialprint(" "); Serialprint(nBaroParam.nAbsoluteAltitude);
                }
            }
            #endif
        }
        #endif
    }

    #if __GYROSCOPE_ENABLED__
    {
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
            //Serialprintln(F("FIFO overflow!"));
            
            // otherwise, check for DMP data ready interrupt (this should happen frequently)
            
            bSkipFlag = true;
        }
        else if(nMPUInterruptStat & 0x02)
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
    }
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
    //_print_RPY_Signals(nRPY);
    //_print_Throttle_Signals(nThrottle);
    //_print_Gyro_Signals(nGyro);
    //_print_RC_Signals();
    nEndTime = micros();
    Serialprint(" ");Serialprint((nEndTime - nStartTime)/1000);
    Serialprintln("");
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
    
    nRC_Ch[0] = floor(nRC_Ch[0] / ROUNDING_BASE) * ROUNDING_BASE;
    nRC_Ch[1] = floor(nRC_Ch[1] / ROUNDING_BASE) * ROUNDING_BASE;
    nRC_Ch[3] = floor(nRC_Ch[3] / ROUNDING_BASE) * ROUNDING_BASE;
    
    nRC_Ch[0] = map(nRC_Ch[0], RC_LOW_CH1, RC_HIGH_CH1, ROLL_ANG_MIN, ROLL_ANG_MAX) - 1;
    nRC_Ch[1] = map(nRC_Ch[1], RC_LOW_CH2, RC_HIGH_CH2, PITCH_ANG_MIN, PITCH_ANG_MAX) - 1;
    nRC_Ch[3] = map(nRC_Ch[3], RC_LOW_CH4, RC_HIGH_CH4, YAW_RATE_MIN, YAW_RATE_MAX);
    
    if((nRC_Ch[0] < ROLL_ANG_MIN) || (nRC_Ch[0] > ROLL_ANG_MAX))
        nRC_Ch[0] = nRC_PrevCh[0];
    if((nRC_Ch[1] < PITCH_ANG_MIN) || (nRC_Ch[1] > PITCH_ANG_MAX))
        nRC_Ch[1] = nRC_PrevCh[1];
    if((nRC_Ch[3] < YAW_RATE_MIN) || (nRC_Ch[3] > YAW_RATE_MAX))
        nRC_Ch[3] = nRC_PrevCh[3];
    
    nRC_PrevCh[0] = nRC_Ch[0];
    nRC_PrevCh[1] = nRC_Ch[1];
    nRC_PrevCh[3] = nRC_Ch[3];
    
    nRPY[1] = nRPY[1] * 180 / M_PI + PITCH_ANG_OFFSET;
    nRPY[2] = nRPY[2] * 180 / M_PI + ROLL_ANG_OFFSET;
    
    if(abs(nRPY[1] - nPrevRPY[1]) > 30)
        nRPY[1] = nPrevRPY[1];
    
    if(abs(nRPY[2] - nPrevRPY[2]) > 30)
        nRPY[2] = nPrevRPY[2];
    
    //ROLL control
    pRoll->nAngleErr = nRC_Ch[0] - nRPY[2];
    pRoll->nCurrErrRate = pRoll->nAngleErr * ROLL_OUTER_P_GAIN - nGyro[0];
    pRoll->nP_ErrRate = pRoll->nCurrErrRate * ROLL_INNER_P_GAIN;
    pRoll->nI_ErrRate = Clip3Float((pRoll->nI_ErrRate + (pRoll->nCurrErrRate * ROLL_INNER_I_GAIN) * SAMPLING_TIME), -100, 100);
    pRoll->nD_ErrRate = (pRoll->nCurrErrRate - pRoll->nPrevErrRate) * ROLL_INNER_D_GAIN / SAMPLING_TIME;
    pRoll->nBalance = pRoll->nP_ErrRate + pRoll->nI_ErrRate + pRoll->nD_ErrRate;
    
    //PITCH control
    pPitch->nAngleErr = nRC_Ch[1] - nRPY[1];
    pPitch->nCurrErrRate = pPitch->nAngleErr * PITCH_OUTER_P_GAIN + nGyro[1];
    pPitch->nP_ErrRate = pPitch->nCurrErrRate * PITCH_INNER_P_GAIN;
    pPitch->nI_ErrRate = Clip3Float((pPitch->nI_ErrRate + (pPitch->nCurrErrRate * PITCH_INNER_I_GAIN) * SAMPLING_TIME), -100, 100);
    pPitch->nD_ErrRate = (pPitch->nCurrErrRate - pPitch->nPrevErrRate) * PITCH_INNER_D_GAIN / SAMPLING_TIME;
    pPitch->nBalance = pPitch->nP_ErrRate + pPitch->nI_ErrRate + pPitch->nD_ErrRate;
    
    //YAW control
    pYaw->nCurrErrRate = nRC_Ch[3] - nGyro[2];
    pYaw->nP_ErrRate = pYaw->nCurrErrRate * YAW_P_GAIN;
    pYaw->nI_ErrRate = Clip3Float((pYaw->nI_ErrRate + pYaw->nCurrErrRate * YAW_I_GAIN * SAMPLING_TIME), -50, 50);
    pYaw->nTorque = pYaw->nP_ErrRate + pYaw->nI_ErrRate;

    // Backup for Next
    pRoll->nPrevErrRate = pRoll->nCurrErrRate;
    pPitch->nPrevErrRate = pPitch->nCurrErrRate;
    nPrevRPY[1] = nRPY[1];
    nPrevRPY[2] = nRPY[2];
    
    releaseLock();
}


inline void CalculateThrottleVal(struct _AxisErrRate_T *pRoll, struct _AxisErrRate_T *pPitch, struct _AxisErrRate_T *pYaw, int nThrottle[4])
{
    float                   nEstimatedThrottle = 0.0f;
    static float            nPrevEstimatedThrottle = 0.0f;
    
    acquireLock();
    
    nRC_Ch[2] = floor(nRC_Ch[2] / ROUNDING_BASE) * ROUNDING_BASE;
    nEstimatedThrottle = (float)(map(nRC_Ch[2], RC_LOW_CH3, RC_HIGH_CH3, ESC_MIN, ESC_MAX));
    
    if((nEstimatedThrottle < ESC_MIN) || (nEstimatedThrottle > ESC_MAX))
        nEstimatedThrottle = nPrevEstimatedThrottle;
    
    nPrevEstimatedThrottle = nEstimatedThrottle;
    
    releaseLock();
    
    if(nEstimatedThrottle > ESC_TAKEOFF_OFFSET)
    {
        nThrottle[0] = Clip3Int((( pPitch->nBalance - pRoll->nBalance) * 0.5 + pYaw->nTorque + nEstimatedThrottle), ESC_MIN, ESC_MAX);
        nThrottle[1] = Clip3Int(((-pPitch->nBalance - pRoll->nBalance) * 0.5 - pYaw->nTorque + nEstimatedThrottle), ESC_MIN, ESC_MAX);
        nThrottle[2] = Clip3Int(((-pPitch->nBalance + pRoll->nBalance) * 0.5 + pYaw->nTorque + nEstimatedThrottle), ESC_MIN, ESC_MAX);
        nThrottle[3] = Clip3Int((( pPitch->nBalance + pRoll->nBalance) * 0.5 - pYaw->nTorque + nEstimatedThrottle), ESC_MIN, ESC_MAX);
    }
}


inline void UpdateESCs(int nThrottle[4])
{
    nESC[1].writeMicroseconds(nThrottle[0]);
    nESC[3].writeMicroseconds(nThrottle[1]);
    nESC[2].writeMicroseconds(nThrottle[2]);
    nESC[4].writeMicroseconds(nThrottle[3]);
}



inline void nRCInterrupt_CB1()
{
    if(!nInterruptLockFlag)
        nRC_Ch[0] = micros() - nRCPrevChangeTime1;
    
    nRCPrevChangeTime1 = micros();
}


inline void nRCInterrupt_CB2()
{
    if(!nInterruptLockFlag)
        nRC_Ch[1] = micros() - nRCPrevChangeTime2;
    
    nRCPrevChangeTime2 = micros();
}


inline void nRCInterrupt_CB3()
{
    if(!nInterruptLockFlag)
        nRC_Ch[2] = micros() - nRCPrevChangeTime3;
    
    nRCPrevChangeTime3 = micros();
}


inline void nRCInterrupt_CB4()
{
    if(!nInterruptLockFlag)
        nRC_Ch[3] = micros() - nRCPrevChangeTime4;
    
    nRCPrevChangeTime4 = micros();
}


inline void nRCInterrupt_CB5()
{
    if(!nInterruptLockFlag)
        nRC_Ch[4] = micros() - nRCPrevChangeTime5;
    
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
float Clip3Float(const float nValue, const int MIN, const int MAX)
{
    float           nClipVal = nValue;
    
    if(nValue < MIN)
        nClipVal = MIN;
    else if(nValue > MAX)
        nClipVal = MAX;

    return nClipVal;
}


int Clip3Int(const int nValue, const int MIN, const int MAX)
{
    int             nClipVal = nValue;
    
    if(nValue < MIN)
        nClipVal = MIN;
    else if(nValue > MAX)
        nClipVal = MAX;
    
    return nClipVal;
}


#if __DEBUG__
void _print_RC_Signals()
{
    Serialprint("   //   RC_Roll:");
    if(nRC_Ch[0] - 1480 < 0)Serialprint("<<<");
    else if(nRC_Ch[0] - 1520 > 0)Serialprint(">>>");
    else Serialprint("-+-");
    Serialprint(nRC_Ch[0]);
    
    Serialprint("   RC_Pitch:");
    if(nRC_Ch[1] - 1480 < 0)Serialprint("^^^");
    else if(nRC_Ch[1] - 1520 > 0)Serialprint("vvv");
    else Serialprint("-+-");
    Serialprint(nRC_Ch[1]);
    
    Serialprint("   RC_Throttle:");
    if(nRC_Ch[2] - 1480 < 0)Serialprint("vvv");
    else if(nRC_Ch[2] - 1520 > 0)Serialprint("^^^");
    else Serialprint("-+-");
    Serialprint(nRC_Ch[2]);
    
    Serialprint("   RC_Yaw:");
    if(nRC_Ch[3] - 1480 < 0)Serialprint("<<<");
    else if(nRC_Ch[3] - 1520 > 0)Serialprint(">>>");
    else Serialprint("-+-");
    Serialprint(nRC_Ch[3]);
    
    Serialprint("   RC_Gear:");
    if(nRC_Ch[4] - 1480 < 0)Serialprint("<<<");
    else if(nRC_Ch[4] - 1520 > 0)Serialprint(">>>");
    else Serialprint("-+-");
    Serialprint(nRC_Ch[4]);
}


void _print_Gyro_Signals(int16_t nGyro[3])
{
    Serialprint("   //    Roll Gyro : ");
    Serialprint(nGyro[0]);
    Serialprint("   Pitch Gyro : ");
    Serialprint(nGyro[1]);
    Serialprint("   Yaw Gyro : ");
    Serialprint(nGyro[2]);
}


void _print_Throttle_Signals(int nThrottle[4])
{
    Serialprint("   //    Thrt1 : ");
    Serialprint(nThrottle[0]);
    Serialprint("  Thrt2 : ");
    Serialprint(nThrottle[1]);
    Serialprint("  Thrt3 : ");
    Serialprint(nThrottle[2]);
    Serialprint("  Thrt4 : ");
    Serialprint(nThrottle[3]);
}


void _print_RPY_Signals(float nRPY[3])
{
    Serialprint("   //    Roll : ");
    Serialprint(nRPY[2]);
    Serialprint("   Pitch : ");
    Serialprint(nRPY[1]);
    Serialprint("   Yaw : ");
    Serialprint(nRPY[0]);
}
#endif
