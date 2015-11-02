#ifndef DronixCopter(RC)
#define DronixCopter(RC)

#include <Servo.h>
#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <PinChangeInt.h>

#define DEBUG

// Arduino Pin configuration
#define ESC_1                               (11)
#define ESC_2                               (10)
#define ESC_3                               (9)
#define ESC_4                               (8)
#define REMOTECTRL_CH1                      (6)
#define REMOTECTRL_CH2                      (5)
#define REMOTECTRL_CH3                      (4)
#define REMOTECTRL_CH4                      (3)
#define REMOTECTRL_CH5                      (2)
#define REMOTECTRL_CHPWR                    (A0)

#define SAMPLING_TIME                       (0.01)              // seconds

// ESC configuration
#define ESC_MIN                             (800)
#define ESC_MAX (2200)
#define ESC_TAKEOFF_OFFSET                  (900)
#define ESC_ARM_DELAY                       (5000)

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

#define ROUNDING_BASE                       (50)

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

MPU6050                 mpu;                                    // mpu interface object
Quaternion              q;
VectorFloat             gravity;
Servo                   a,b,c,d;

bool                    dmpReady = false;                       // set true if DMP init was successful
uint8_t                 mpuIntStatus;                           // holds actual interrupt status byte from MPU
uint8_t                 devStatus;                              // return status after each device operation (0 = success, !0 = error)
uint16_t                packetSize;                             // expected DMP packet size (default is 42 bytes)
uint16_t                fifoCount;                              // count of all bytes currently in FIFO
uint8_t                 fifoBuffer[64];                         // FIFO storage buffer
float                   ypr[3];                                 // yaw pitch roll values
float                   yprLast[3];
int16_t                 gyro[3];                                // gyro values
volatile bool           mpuInterrupt = false;
boolean                 interruptLock = false;                  // Interrupt lock



void dmpDataReady()
{
    mpuInterrupt = true;
}


float                   ch1, ch2, ch3, ch4, ch5;                // RC channel inputs
unsigned long           rcLastChange1 = micros();
unsigned long           rcLastChange2 = micros();
unsigned long           rcLastChange3 = micros();
unsigned long           rcLastChange4 = micros();
unsigned long           rcLastChange5 = micros();

// Motor controll variables
int                     Throttle, ThrottleLast;                 // global throttle

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


AxisErrRate_T           nPitchErrRate = {0, };
AxisErrRate_T           nRollErrRate = {0, };
AxisErrRate_T           nYawErrRate = {0, };


/////////////////////////////
int                     Ta, Tb, Tc, Td;                         //throttles


// Filter variables
float                   ch1Last, ch2Last, ch4Last;


//ERR_I limit to prevent divergence
void Clip3_FLOAT(float *value, int MIN, int MAX)
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


void Clip3_INT(int *value, int MIN, int MAX)
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
    
    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    
    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
    
    // wait for ready
    //Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    //while (Serial.available() && Serial.read()); // empty buffer
    //while (!Serial.available());                                 // wait for data
    //while (Serial.available() && Serial.read()); // empty buffer again
    
    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();
    
    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
    
    // make sure it worked (returns 0 if so)
    if (devStatus == 0)
    {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);
        
        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        
        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;
        
        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    }
    else
    {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
    
    initRC();
    initESCs();
}

void loop()
{
    while(!mpuInterrupt && fifoCount < packetSize)
    {
        /* Do nothing while MPU is not working
         * This should be a VERY short period
         */
        Serial.println("Loop1");
    }
    
    ////////////////////////////////// Get angle & gyro ///////////////////////////////////////////////////////
    // if programming failed, don't try to do anything
    if(!dmpReady)
    {
        Serial.println("DMP Not Ready    return");
        return;
    }
    
    // wait for MPU interrupt or extra packet(s) available
    while(!mpuInterrupt && fifoCount < packetSize)
    {
        // other program behavior stuff here
        // .
        // .
        // .
        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data
        // .
        // .
        // .
        Serial.println("      Loop2");
    }
    
    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    
    // get current FIFO count
    fifoCount = mpu.getFIFOCount();
    
    // check for overflow (this should never happen unless our code is too inefficient)
    if((mpuIntStatus & 0x10) || fifoCount == 1024)
    {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        //Serial.println(F("FIFO overflow!"));
        
        // otherwise, check for DMP data ready interrupt (this should happen frequently)
    }
    else if (mpuIntStatus & 0x02)
    {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
        
        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;
        
        mpu.dmpGetGyro(gyro, fifoBuffer);
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    
        ////////////////////////////////// PID Computation ////////////////////////////////////////////////////////
        {
            acquireLock();
            
            ch1 = floor(ch1/ROUNDING_BASE)*ROUNDING_BASE;
            ch2 = floor(ch2/ROUNDING_BASE)*ROUNDING_BASE;
            ch4 = floor(ch4/ROUNDING_BASE)*ROUNDING_BASE;
            
            ch2 = map(ch2, REMOTECTRL_LOW_CH2, REMOTECTRL_HIGH_CH2, PITCH_ANG_MIN, PITCH_ANG_MAX) - 1;
            ch1 = map(ch1, REMOTECTRL_LOW_CH1, REMOTECTRL_HIGH_CH1, ROLL_ANG_MIN, ROLL_ANG_MAX) - 1;
            ch4 = map(ch4, REMOTECTRL_LOW_CH4, REMOTECTRL_HIGH_CH4, YAW_RATE_MIN, YAW_RATE_MAX);
            
            if((ch2 < PITCH_ANG_MIN) || (ch2 > PITCH_ANG_MAX)) ch2 = ch2Last;
            if((ch1 < ROLL_ANG_MIN) || (ch1 > ROLL_ANG_MAX)) ch1 = ch1Last;
            if((ch4 < YAW_RATE_MIN) || (ch4 > YAW_RATE_MAX)) ch4 = ch4Last;
            
            ch1Last = ch1;
            ch2Last = ch2;
            ch4Last = ch4;
            
            ypr[1] = ypr[1] * 180/M_PI + PITCH_ANG_OFFSET;
            ypr[2] = ypr[2] * 180/M_PI + ROLL_ANG_OFFSET;
            
            if(abs(ypr[1]-yprLast[1])>30)
                ypr[1] = yprLast[1];
            
            if(abs(ypr[2]-yprLast[2])>30)
                ypr[2] = yprLast[2];
            
            yprLast[1] = ypr[1];
            yprLast[2] = ypr[2];
            
            //ROLL control
            nRollErrRate.nAngleErr = ch1 - ypr[2];
            nRollErrRate.nCurrErrRate = nRollErrRate.nAngleErr * ROLL_OUTER_P_GAIN - gyro[0];
            nRollErrRate.nP_ErrRate = nRollErrRate.nCurrErrRate * ROLL_INNER_P_GAIN;
            nRollErrRate.nI_ErrRate = nRollErrRate.nI_ErrRate + (nRollErrRate.nCurrErrRate * ROLL_INNER_I_GAIN) * SAMPLING_TIME;
            Clip3_FLOAT(&nRollErrRate.nI_ErrRate, -100, 100);
            nRollErrRate.nD_ErrRate = (nRollErrRate.nCurrErrRate - nRollErrRate.nPrevErrRate) / SAMPLING_TIME * ROLL_INNER_D_GAIN;
            nRollErrRate.nPrevErrRate = nRollErrRate.nCurrErrRate;
            nRollErrRate.nBalance = nRollErrRate.nP_ErrRate + nRollErrRate.nI_ErrRate + nRollErrRate.nD_ErrRate;
            
            //PITCH control
            nPitchErrRate.nAngleErr = ch2 - ypr[1];
            nPitchErrRate.nCurrErrRate = nPitchErrRate.nAngleErr * PITCH_OUTER_P_GAIN + gyro[1];
            nPitchErrRate.nP_ErrRate = nPitchErrRate.nCurrErrRate * PITCH_INNER_P_GAIN;
            nPitchErrRate.nI_ErrRate = nPitchErrRate.nI_ErrRate + (nPitchErrRate.nCurrErrRate * PITCH_INNER_I_GAIN) * SAMPLING_TIME;
            Clip3_FLOAT(&nPitchErrRate.nI_ErrRate, -100, 100);
            nPitchErrRate.nD_ErrRate = (nPitchErrRate.nCurrErrRate - nPitchErrRate.nPrevErrRate) / SAMPLING_TIME * PITCH_INNER_D_GAIN;
            nPitchErrRate.nPrevErrRate = nPitchErrRate.nCurrErrRate;
            nPitchErrRate.nBalance = nPitchErrRate.nP_ErrRate + nPitchErrRate.nI_ErrRate + nPitchErrRate.nD_ErrRate;
            
            //YAW control
            nYawErrRate.nCurrErrRate = ch4 - gyro[2];
            nYawErrRate.nP_ErrRate = nYawErrRate.nCurrErrRate * YAW_P_GAIN;
            nYawErrRate.nI_ErrRate = nYawErrRate.nI_ErrRate + nYawErrRate.nCurrErrRate * YAW_I_GAIN * SAMPLING_TIME;
            Clip3_FLOAT(&nYawErrRate.nI_ErrRate, -50, 50);
            nYawErrRate.nTorque = nYawErrRate.nP_ErrRate + nYawErrRate.nI_ErrRate;
            
            releaseLock();
        }

        /////////////////////////////////////Throttle Calculation //////////////////////////////////////////////////
        {
            acquireLock();
            
            ch3 = floor(ch3/ROUNDING_BASE)*ROUNDING_BASE;
            Throttle = map(ch3, REMOTECTRL_LOW_CH3, REMOTECTRL_HIGH_CH3, ESC_MIN, ESC_MAX);
            if((Throttle < ESC_MIN) || (Throttle > ESC_MAX))
                Throttle = ThrottleLast;
            
            ThrottleLast = Throttle;
            
            releaseLock();
        }
        
        Ta = ( nPitchErrRate.nBalance - nRollErrRate.nBalance) * 0.5 + nYawErrRate.nTorque + (float)Throttle;
        Tb = (-nPitchErrRate.nBalance - nRollErrRate.nBalance) * 0.5 - nYawErrRate.nTorque + (float)Throttle;
        Tc = (-nPitchErrRate.nBalance + nRollErrRate.nBalance) * 0.5 + nYawErrRate.nTorque + (float)Throttle;
        Td = ( nPitchErrRate.nBalance + nRollErrRate.nBalance) * 0.5 - nYawErrRate.nTorque + (float)Throttle;
        
        Clip3_INT(&Ta, ESC_MIN, ESC_MAX);
        Clip3_INT(&Tb, ESC_MIN, ESC_MAX);
        Clip3_INT(&Tc, ESC_MIN, ESC_MAX);
        Clip3_INT(&Td, ESC_MIN, ESC_MAX);
        
        if(Throttle < ESC_TAKEOFF_OFFSET)
        {
            Ta = ESC_MIN;
            Tb = ESC_MIN;
            Tc = ESC_MIN;
            Td = ESC_MIN;
        }
        
        //////////////////////////////////Update BLDCs/////////////////////////////////////////////////////////////////
        //a.writeMicroseconds(Ta);
        //c.writeMicroseconds(Tc);
        //b.writeMicroseconds(Tb);
        //d.writeMicroseconds(Td);
        
        
        /*
         Serial.print("Ta : ");
         Serial.print(Ta);
         Serial.print("Tb : ");
         Serial.print(Tb);
         Serial.print("Tc : ");
         Serial.print(Tc);
         Serial.print("Td : ");
         Serial.println(Td);
         */
        
        /*
        Serial.print("Roll : ");
        Serial.print(ypr[2]);
        Serial.print("Pitch : ");
        Serial.println(ypr[1]);
        */
        
        Serial.print("Roll Gyro : ");
        Serial.print(gyro[0]);
        Serial.print("    Pitch Gyro : ");
        Serial.print(gyro[1]);
        Serial.print("    Yaw Gyro : ");
        Serial.println(gyro[2]);
        
        /*
         Serial.print("Roll cmd : ");
         Serial.print(ch1);
         Serial.print("Pitch cmd : ");
         Serial.print(ch2);
         Serial.print("Yaw cmd : ");
         Serial.println(ch4);
         */
    }
}


inline void arm()
{
    a.writeMicroseconds(ESC_MIN);
    b.writeMicroseconds(ESC_MIN);
    c.writeMicroseconds(ESC_MIN);
    d.writeMicroseconds(ESC_MIN);
    
    delay(ESC_ARM_DELAY);
}


inline void initESCs()
{
    a.attach(ESC_1);
    b.attach(ESC_2);
    c.attach(ESC_3);
    d.attach(ESC_4);
    
    delay(1000);
    
    arm();
}


inline void initRC()
{
    pinMode(REMOTECTRL_CHPWR, OUTPUT);
    digitalWrite(REMOTECTRL_CHPWR, HIGH);
    
    // FIVE FUCKING INTERRUPTS !!!
    PCintPort::attachInterrupt(REMOTECTRL_CH1, rcInterrupt1, CHANGE);
    PCintPort::attachInterrupt(REMOTECTRL_CH2, rcInterrupt2, CHANGE);
    PCintPort::attachInterrupt(REMOTECTRL_CH3, rcInterrupt3, CHANGE);
    PCintPort::attachInterrupt(REMOTECTRL_CH4, rcInterrupt4, CHANGE);
    PCintPort::attachInterrupt(REMOTECTRL_CH5, rcInterrupt5, CHANGE);   
}


inline void rcInterrupt1()
{
    if(!interruptLock)
        ch1 = micros() - rcLastChange1;
    
    rcLastChange1 = micros();
}


inline void rcInterrupt2()
{
    if(!interruptLock)
        ch2 = micros() - rcLastChange2;
    
    rcLastChange2 = micros();
}


inline void rcInterrupt3()
{
    if(!interruptLock)
        ch3 = micros() - rcLastChange3;
    
    rcLastChange3 = micros();
}


inline void rcInterrupt4()
{
    if(!interruptLock)
        ch4 = micros() - rcLastChange4;
    
    rcLastChange4 = micros();
}


inline void rcInterrupt5()
{
    if(!interruptLock)
        ch5 = micros() - rcLastChange5;
    
    rcLastChange5 = micros();
}


inline void acquireLock()
{
    interruptLock = true;
}


inline void releaseLock()
{
    interruptLock = false;
}

#endif
