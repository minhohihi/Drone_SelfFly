#ifndef DronixCopter(RC)
#define DronixCopter(RC)

#define __DEBUG__                           (0)

#define __GYROSCOPE_ENABLED__               (1)
#define __COMPASS_ENABLED__                 (0)
#define __BAROMETER_ENABLED__               (0)

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

// Arduino Pin configuration
#define ESC_1                               (11)
#define ESC_2                               (10)
#define ESC_3                               (9)
#define ESC_4                               (8)
#define REMOTECTRL_CH1                      (6)
#define REMOTECTRL_CH2                      (5)
#define REMOTECTRL_CH3                      (4)
#define REMOTECTRL_CH4                      (3)
#define REMOTECTRL_CH5                      (7)
#define REMOTECTRL_CHPWR                    (A0)

// ESC configuration
#define ESC_MIN                             (800)
#define ESC_MAX (2200)
#define ESC_TAKEOFF_OFFSET                  (900)
#define ESC_ARM_DELAY                       (1000)

// RC configuration
#define REMOTECTRL_HIGH_CH1                 (1900)
#define REMOTECTRL_LOW_CH1                  (1050)
#define REMOTECTRL_HIGH_CH2                 (1900)
#define REMOTECTRL_LOW_CH2                  (1050)
#define REMOTECTRL_HIGH_CH3                 (1900)
#define REMOTECTRL_LOW_CH3                  (1050)
#define REMOTECTRL_HIGH_CH4                 (1900)
#define REMOTECTRL_LOW_CH4                  (1050)
#define REMOTECTRL_HIGH_CH5                 (1900)
#define REMOTECTRL_LOW_CH5                  (1050)

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

bool                    nDMPReadyFlag = false;                  // set true if DMP init was successful
uint8_t                 nMPUInterruptStat;                      // holds actual interrupt status byte from MPU
uint8_t                 nDevStatus;                             // return status after each device operation (0 = success, !0 = error)
uint16_t                nPacketSize;                            // expected DMP packet size (default is 42 bytes)
uint16_t                nFIFOCnt;                               // count of all bytes currently in FIFO
uint8_t                 nFIFOBuf[64];                           // FIFO storage buffer
float                   nRPY[3];                                 // yaw pitch roll values
float                   nPrevRPY[3];
int16_t                 nGyro[3];                               // Gyroscope Value
double                  nRefBarometerVal;                       // Reference Barometer Value

volatile bool           nMPUInterruptFlag = false;
boolean                 nInterruptLockFlag = false;             // Interrupt lock

float                   nRC_Ch1, nRC_Ch2, nRC_Ch3, nRC_Ch4, nRC_Ch5;        // RC channel inputs
int                     nThrottle[4];                                       //throttles
float                   nRC_Ch1Prev, nRC_Ch2Prev, nRC_Ch4Prev;              // Filter variables

unsigned long           nRCPrevChangeTime1 = micros();
unsigned long           nRCPrevChangeTime2 = micros();
unsigned long           nRCPrevChangeTime3 = micros();
unsigned long           nRCPrevChangeTime4 = micros();
unsigned long           nRCPrevChangeTime5 = micros();

// Motor controll variables
int                     nPrevEstimatedThrottle;                 // global throttle

Servo                   nESC[4];
#if __GYROSCOPE_ENABLED__
    MPU6050               nMPU;                                 // MPU6050 Gyroscope Interface
    Quaternion            nQuater;
    VectorFloat           nGravity;
#endif
#if __COMPASS_ENABLED__
    HMC5883L              nCompass;                             // HMC58x3 Compass Interface
#endif
#if __BAROMETER_ENABLED__
    MS5611                nBarometer;                           // MS5611 Barometer Interface
    uint32_t              nRawTemp;                             // Raw Temperature Data
    uint32_t              nRawPressure;                         // Raw Pressure Data
    double                nRealTemperature;                     // Real Temperature Data
    long                  nRealPressure;                        // Real Pressure Data
    float                 nAbsoluteAltitude;                    // Estimated Absolute Altitude
    float                 nRelativeAltitude;                    // Estimated Relative Altitude
#endif

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

AxisErrRate_T           nPitch = {0, };
AxisErrRate_T           nRoll = {0, };
AxisErrRate_T           nYaw = {0, };


// Setup function
void setup()
{
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
    
    Serial.begin(115200);
    Serial.flush();
    
    while (!Serial); // wait for Leonardo enumeration, others continue immediately
    
    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
    // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    #if __GYROSCOPE_ENABLED__
    {
        #if 0
        while(!nMPU.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
        {
            Serial.println("Can Not Find a Valid MPU6050, Check H/W");
            delay(500);
        }
        nMPU.setI2CMasterModeEnabled(false);
        nMPU.setI2CBypassEnabled(true);
        nMPU.setSleepEnabled(false);
        #else
        // initialize MPU
        Serial.println(F("Initializing MPU..."));
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
            attachInterrupt(digitalPinToInterrupt(2), dmpDataReady, RISING);
            nMPUInterruptStat = nMPU.getIntStatus();
        
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
        #endif
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
        nCompass.setDataRate(HMC5883L_DATARATE_30HZ);
        
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
        nRefBarometerVal = nBarometer.readPressure();
        
        Serial.print("Barometer Oversampling: ");
        Serial.println(nBarometer.getOversampling());
    }
    #endif
    
    initRC();
    initESCs();
}

void loop()
{
    bool bSkipFlag = false;
  
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
    #endif

    #if __BAROMETER_ENABLED__
        // Read raw values
        nRawTemp = nBarometer.readRawTemperature();
        nRawPressure = nBarometer.readRawPressure();

        // Read true temperature & Pressure
        nRealTemperature = nBarometer.readTemperature();
        nRealPressure = nBarometer.readPressure();

        // Calculate altitude
        nAbsoluteAltitude = nBarometer.getAltitude(nRealPressure);
        nRelativeAltitude = nBarometer.getAltitude(nRealPressure, nRefBarometerVal);
    #endif

    if(false == bSkipFlag)
    {
        // PID Computation
        CalculatePID();
    
        //Throttle Calculation
        CalculateThrottleVal();
    
        //Update BLDCs
        UpdateESCs();
    }

    #if __DEBUG__
        _print_RPY_Signals();
        _print_Throttle_Signals();
        _print_Gyro_Signals();
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
    nESC[1].attach(ESC_1);
    nESC[2].attach(ESC_2);
    nESC[3].attach(ESC_3);
    nESC[4].attach(ESC_4);
    
    delay(1000);
    
    arm();
}


inline void initRC()
{
    pinMode(REMOTECTRL_CHPWR, OUTPUT);
    digitalWrite(REMOTECTRL_CHPWR, HIGH);
    
    PCintPort::attachInterrupt(REMOTECTRL_CH1, nRCInterrupt_CB1, CHANGE);
    PCintPort::attachInterrupt(REMOTECTRL_CH2, nRCInterrupt_CB2, CHANGE);
    PCintPort::attachInterrupt(REMOTECTRL_CH3, nRCInterrupt_CB3, CHANGE);
    PCintPort::attachInterrupt(REMOTECTRL_CH4, nRCInterrupt_CB4, CHANGE);
    PCintPort::attachInterrupt(REMOTECTRL_CH5, nRCInterrupt_CB5, CHANGE);
}


inline void CalculatePID()
{
    acquireLock();
    
    nRC_Ch1 = floor(nRC_Ch1 / ROUNDING_BASE) * ROUNDING_BASE;
    nRC_Ch2 = floor(nRC_Ch2 / ROUNDING_BASE) * ROUNDING_BASE;
    nRC_Ch4 = floor(nRC_Ch4 / ROUNDING_BASE) * ROUNDING_BASE;
    
    nRC_Ch1 = map(nRC_Ch1, REMOTECTRL_LOW_CH1, REMOTECTRL_HIGH_CH1, ROLL_ANG_MIN, ROLL_ANG_MAX) - 1;
    nRC_Ch2 = map(nRC_Ch2, REMOTECTRL_LOW_CH2, REMOTECTRL_HIGH_CH2, PITCH_ANG_MIN, PITCH_ANG_MAX) - 1;
    nRC_Ch4 = map(nRC_Ch4, REMOTECTRL_LOW_CH4, REMOTECTRL_HIGH_CH4, YAW_RATE_MIN, YAW_RATE_MAX);
    
    if((nRC_Ch1 < ROLL_ANG_MIN) || (nRC_Ch1 > ROLL_ANG_MAX))
        nRC_Ch1 = nRC_Ch1Prev;
    if((nRC_Ch2 < PITCH_ANG_MIN) || (nRC_Ch2 > PITCH_ANG_MAX))
        nRC_Ch2 = nRC_Ch2Prev;
    if((nRC_Ch4 < YAW_RATE_MIN) || (nRC_Ch4 > YAW_RATE_MAX))
        nRC_Ch4 = nRC_Ch4Prev;
    
    nRC_Ch1Prev = nRC_Ch1;
    nRC_Ch2Prev = nRC_Ch2;
    nRC_Ch4Prev = nRC_Ch4;
    
    nRPY[1] = nRPY[1] * 180/M_PI + PITCH_ANG_OFFSET;
    nRPY[2] = nRPY[2] * 180/M_PI + ROLL_ANG_OFFSET;
    
    if(abs(nRPY[1]-nPrevRPY[1])>30)
        nRPY[1] = nPrevRPY[1];
    
    if(abs(nRPY[2]-nPrevRPY[2])>30)
        nRPY[2] = nPrevRPY[2];
    
    nPrevRPY[1] = nRPY[1];
    nPrevRPY[2] = nRPY[2];
    
    //ROLL control
    nRoll.nAngleErr = nRC_Ch1 - nRPY[2];
    nRoll.nCurrErrRate = nRoll.nAngleErr * ROLL_OUTER_P_GAIN - nGyro[0];
    nRoll.nP_ErrRate = nRoll.nCurrErrRate * ROLL_INNER_P_GAIN;
    nRoll.nI_ErrRate = nRoll.nI_ErrRate + (nRoll.nCurrErrRate * ROLL_INNER_I_GAIN) * SAMPLING_TIME;
    Clip3Float(&nRoll.nI_ErrRate, -100, 100);
    nRoll.nD_ErrRate = (nRoll.nCurrErrRate - nRoll.nPrevErrRate) / SAMPLING_TIME * ROLL_INNER_D_GAIN;
    nRoll.nPrevErrRate = nRoll.nCurrErrRate;
    nRoll.nBalance = nRoll.nP_ErrRate + nRoll.nI_ErrRate + nRoll.nD_ErrRate;
    
    //PITCH control
    nPitch.nAngleErr = nRC_Ch2 - nRPY[1];
    nPitch.nCurrErrRate = nPitch.nAngleErr * PITCH_OUTER_P_GAIN + nGyro[1];
    nPitch.nP_ErrRate = nPitch.nCurrErrRate * PITCH_INNER_P_GAIN;
    nPitch.nI_ErrRate = nPitch.nI_ErrRate + (nPitch.nCurrErrRate * PITCH_INNER_I_GAIN) * SAMPLING_TIME;
    Clip3Float(&nPitch.nI_ErrRate, -100, 100);
    nPitch.nD_ErrRate = (nPitch.nCurrErrRate - nPitch.nPrevErrRate) / SAMPLING_TIME * PITCH_INNER_D_GAIN;
    nPitch.nPrevErrRate = nPitch.nCurrErrRate;
    nPitch.nBalance = nPitch.nP_ErrRate + nPitch.nI_ErrRate + nPitch.nD_ErrRate;
    
    //YAW control
    nYaw.nCurrErrRate = nRC_Ch4 - nGyro[2];
    nYaw.nP_ErrRate = nYaw.nCurrErrRate * YAW_P_GAIN;
    nYaw.nI_ErrRate = nYaw.nI_ErrRate + nYaw.nCurrErrRate * YAW_I_GAIN * SAMPLING_TIME;
    Clip3Float(&nYaw.nI_ErrRate, -50, 50);
    nYaw.nTorque = nYaw.nP_ErrRate + nYaw.nI_ErrRate;
    
    releaseLock();
}


inline void CalculateThrottleVal()
{
    float nEstimatedThrottle = 0.0f;
    
    acquireLock();
    
    nRC_Ch3 = floor(nRC_Ch3 / ROUNDING_BASE) * ROUNDING_BASE;
    nEstimatedThrottle = (float)(map(nRC_Ch3, REMOTECTRL_LOW_CH3, REMOTECTRL_HIGH_CH3, ESC_MIN, ESC_MAX));
    if((nEstimatedThrottle < ESC_MIN) || (nEstimatedThrottle > ESC_MAX))
        nEstimatedThrottle = nPrevEstimatedThrottle;
    
    nPrevEstimatedThrottle = nEstimatedThrottle;
    
    releaseLock();
    
    if(nEstimatedThrottle > ESC_TAKEOFF_OFFSET)
    {
        nThrottle[0] = ( nPitch.nBalance - nRoll.nBalance) * 0.5 + nYaw.nTorque + nEstimatedThrottle;
        nThrottle[1] = (-nPitch.nBalance - nRoll.nBalance) * 0.5 - nYaw.nTorque + nEstimatedThrottle;
        nThrottle[2] = (-nPitch.nBalance + nRoll.nBalance) * 0.5 + nYaw.nTorque + nEstimatedThrottle;
        nThrottle[3] = ( nPitch.nBalance + nRoll.nBalance) * 0.5 - nYaw.nTorque + nEstimatedThrottle;
        
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


inline void UpdateESCs()
{
    //nESC[1].writeMicroseconds(nThrottle[0]);
    //nESC[3].writeMicroseconds(nThrottle[1]);
    //nESC[2].writeMicroseconds(nThrottle[2]);
    //nESC[4].writeMicroseconds(nThrottle[3]);
}

inline void nRCInterrupt_CB1()
{
    if(!nInterruptLockFlag)
        nRC_Ch1 = micros() - nRCPrevChangeTime1;
    
    nRCPrevChangeTime1 = micros();
}


inline void nRCInterrupt_CB2()
{
    if(!nInterruptLockFlag)
        nRC_Ch2 = micros() - nRCPrevChangeTime2;
    
    nRCPrevChangeTime2 = micros();
}


inline void nRCInterrupt_CB3()
{
    if(!nInterruptLockFlag)
        nRC_Ch3 = micros() - nRCPrevChangeTime3;
    
    nRCPrevChangeTime3 = micros();
}


inline void nRCInterrupt_CB4()
{
    if(!nInterruptLockFlag)
        nRC_Ch4 = micros() - nRCPrevChangeTime4;
    
    nRCPrevChangeTime4 = micros();
}


inline void nRCInterrupt_CB5()
{
    if(!nInterruptLockFlag)
        nRC_Ch5 = micros() - nRCPrevChangeTime5;
    
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
    if(nRC_Ch1 - 1480 < 0)Serial.print("<<<");
    else if(nRC_Ch1 - 1520 > 0)Serial.print(">>>");
    else Serial.print("-+-");
    Serial.print(nRC_Ch1);
    
    Serial.print("   RC_Pitch:");
    if(nRC_Ch2 - 1480 < 0)Serial.print("^^^");
    else if(nRC_Ch2 - 1520 > 0)Serial.print("vvv");
    else Serial.print("-+-");
    Serial.print(nRC_Ch2);
    
    Serial.print("   RC_Throttle:");
    if(nRC_Ch3 - 1480 < 0)Serial.print("vvv");
    else if(nRC_Ch3 - 1520 > 0)Serial.print("^^^");
    else Serial.print("-+-");
    Serial.print(nRC_Ch3);
    
    Serial.print("   RC_Yaw:");
    if(nRC_Ch4 - 1480 < 0)Serial.print("<<<");
    else if(nRC_Ch4 - 1520 > 0)Serial.print(">>>");
    else Serial.print("-+-");
    Serial.print(nRC_Ch4);
    
    Serial.print("   RC_Gear:");
    if(nRC_Ch5 - 1480 < 0)Serial.print("<<<");
    else if(nRC_Ch5 - 1520 > 0)Serial.print(">>>");
    else Serial.print("-+-");
    Serial.print(nRC_Ch5);
}

void _print_Gyro_Signals()
{
    Serial.print("   //    Roll Gyro : ");
    Serial.print(nGyro[0]);
    Serial.print("   Pitch Gyro : ");
    Serial.print(nGyro[1]);
    Serial.print("   Yaw Gyro : ");
    Serial.print(nGyro[2]);
}

void _print_Throttle_Signals()
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


void _print_RPY_Signals()
{
    Serial.print("   //    Roll : ");
    Serial.print(nRPY[2]);
    Serial.print("   Pitch : ");
    Serial.print(nRPY[1]);
    Serial.print("   Yaw : ");
    Serial.print(nRPY[0]);    
}
#endif
#endif
