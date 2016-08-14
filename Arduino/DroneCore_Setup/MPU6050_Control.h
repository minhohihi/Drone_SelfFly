//
//  MPU6050_Controller.h
//  SelfFly
//
//  Created by Maverick on 2016. 1. 21..
//

#ifndef __MPU6050_CONTROL__
#define __MPU6050_CONTROL__

#if USE_MPU6050_DMP
    volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
    bool dmpReady = false;                  // set true if DMP init was successful
    uint8_t mpuIntStatus;                   // holds actual interrupt status byte from MPU
    uint8_t devStatus;                      // return status after each device operation (0 = success, !0 = error)
    uint16_t packetSize;                    // expected DMP packet size (default is 42 bytes)
    uint16_t fifoCount;                     // count of all bytes currently in FIFO
    uint8_t fifoBuffer[64];                 // FIFO storage buffer
    Quaternion q;                           // [w, x, y, z]         quaternion container
    VectorFloat gravity;                    // [x, y, z]            gravity vector
    float euler[3];                         // [psi, theta, phi]    Euler angle container
    float ypr[3];                           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

    MPU6050 mpu;

    void dmpDataReady();
    void _AccelGyro_WaitDMPIntrrupt();
    void _AccelGyro_GetQuaternion();
    void _AccelGyro_GetEularAngle();
    void _AccelGyro_GetRPY();
#endif


void _AccelGyro_GetGyroData();
void _AccelGyro_GetAccelData();
void _AccelGyro_Calibration();

void _AccelGyro_Initialize()
{
    int                 i = 0;
    
    Serialprintln(F(" *      5. Start MPU6050 Module Initialization   "));
    
//    nAccelGyroHndl = MPU6050();
//
//    Serialprintln(F(" Initializing MPU..."));
//    nAccelGyroHndl.initialize();
//
//    delay(100);
//    
//    // Verify Vonnection
//    Serialprint(F("    Testing device connections..."));
//    Serialprintln(nAccelGyroHndl.testConnection() ? F("  MPU6050 connection successful") : F("  MPU6050 connection failed"));
//
//    nAccelGyroHndl.setI2CMasterModeEnabled(false);
//    nAccelGyroHndl.setI2CBypassEnabled(true);
//    nAccelGyroHndl.setSleepEnabled(false);
//
//    // Calibrate GyroAccel
//    //nAccelGyroHndl.doCalibration();
//    nAccelGyroHndl.setXGyroOffset(MPU6050_GYRO_OFFSET_X);
//    nAccelGyroHndl.setYGyroOffset(MPU6050_GYRO_OFFSET_Y);
//    nAccelGyroHndl.setZGyroOffset(MPU6050_GYRO_OFFSET_Z);
//    nAccelGyroHndl.setXAccelOffset(MPU6050_ACCEL_OFFSET_X);
//    nAccelGyroHndl.setYAccelOffset(MPU6050_ACCEL_OFFSET_Y);
//    //nAccelGyroHndl.setZAccelOffset(MPU6050_ACCEL_OFFSET_Z);
//
//    // supply your own gyro offsets here, scaled for min sensitivity
//    nAccelGyroHndl.setRate(1);                                            // Sample Rate (4000Hz = 8KHz Gyro SR / 1+1)
//    nAccelGyroHndl.setDLPFMode(MPU6050_DLP_PRECISION);
//    nAccelGyroHndl.setFullScaleGyroRange(GYRO_FS_PRECISIOM);
//    nAccelGyroHndl.setFullScaleAccelRange(ACCEL_FS_PRECISIOM);

    #if USE_MPU6050_DMP
        mpu.initialize();
    
        devStatus = mpu.dmpInitialize();
    
        if(0 == devStatus)
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
    #else
    Wire.beginTransmission(0x68);
    Wire.write(0x6B);                    
    Wire.write(0x00);                    
    Wire.endTransmission();              
    
    Wire.beginTransmission(0x68);
    Wire.write(0x1B);                    
    Wire.write(0x08);                    
    Wire.endTransmission();              
    
    Wire.beginTransmission(0x68);
    Wire.write(0x1B);                    
    Wire.endTransmission();              
    Wire.requestFrom(0x68, 1);
    while(Wire.available() < 1);         
    if(Wire.read() != 0x08)
    {
        _LED_Blink(1, 0, 0, 200000);
        while(1)
            delay(10);
    }
    
    Wire.beginTransmission(0x68);
    Wire.write(0x1A);                    
    Wire.write(0x03);                    
    Wire.endTransmission();

    delay(300);
    
    // Calibration
    _AccelGyro_Calibration();

    #if 0
    {
        uint8_t             *pOffset;
        int16_t             nSensorVal = 0;
        
        // Set Gyro Offset
        for(i=0 ; i<=Z_AXIS ; i++)
        {
            nSensorVal = (int16_t)(_gCalibMeanGyro[i]);
            
            pOffset = (uint8_t *)(&nSensorVal);
            Wire.beginTransmission(0x68);
            Wire.write(0x13 + (2 * i));
            Wire.write((pOffset[0]);
            Wire.write((pOffset[1]);
            Wire.endTransmission();
        }
                       
        // Set Accel Offset
        for(i=0 ; i<=Z_AXIS ; i++)
        {
            nSensorVal = (int16_t)(_gCalibMeanAccel[i]);
            
            pOffset = (uint8_t *)(&nSensorVal);
            Wire.beginTransmission(0x68);
            Wire.write(0x06 + (2 * i));
            Wire.write((pOffset[0]);
            Wire.write((pOffset[1]);
            Wire.endTransmission();
        }
    }
    #endif
    
    _gEstimatedRPY[0] = _gEstimatedRPY[1] = _gEstimatedRPY[2] = 0.0;
    #endif

    delay(300);
    
    Serialprintln(F(" *            => Done!!   "));
    
    return;
}


void _AccelGyro_GetGyroData()
{
    Wire.beginTransmission(0x68);
    Wire.write(0x43);                                               // starting with register 0x43 (GYRO_XOUT_H)
    Wire.endTransmission();
    Wire.requestFrom(0x68, 6);                                      // request a total of 6 registers

    _gRawGyro[X_AXIS] = (Wire.read()<<8 | Wire.read());             // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    _gRawGyro[Y_AXIS] = (Wire.read()<<8 | Wire.read());             // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    _gRawGyro[Z_AXIS] = (Wire.read()<<8 | Wire.read());             // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
    
    _gRawGyro[X_AXIS] -= _gCalibMeanGyro[X_AXIS];
    if(1 == _gAxisReverseFlag[X_AXIS]) _gRawGyro[X_AXIS] *= -1;
    _gRawGyro[Y_AXIS] -= _gCalibMeanGyro[Y_AXIS];
    if(1 == _gAxisReverseFlag[Y_AXIS]) _gRawGyro[Y_AXIS] *= -1;
    _gRawGyro[Z_AXIS] -= _gCalibMeanGyro[Z_AXIS];
    if(1 == _gAxisReverseFlag[Z_AXIS]) _gRawGyro[Z_AXIS] *= -1;
}

                   
void _AccelGyro_GetAccelData()
{
    Wire.beginTransmission(0x68);
    Wire.write(0x3B);                                               // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission();
    Wire.requestFrom(0x68, 6);                                      // request a total of 6 registers
    
    _gRawAccel[X_AXIS] = (Wire.read()<<8 | Wire.read());            // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
    _gRawAccel[Y_AXIS] = (Wire.read()<<8 | Wire.read());            // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    _gRawAccel[Z_AXIS] = (Wire.read()<<8 | Wire.read());            // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    
    _gRawAccel[X_AXIS] -= _gCalibMeanAccel[X_AXIS];
    if(1 == _gAxisReverseFlag[X_AXIS]) _gRawAccel[X_AXIS] *= -1;
    _gRawAccel[Y_AXIS] -= _gCalibMeanAccel[Y_AXIS];
    if(1 == _gAxisReverseFlag[Y_AXIS]) _gRawAccel[Y_AXIS] *= -1;
    _gRawAccel[Z_AXIS] -= _gCalibMeanAccel[Z_AXIS];
    if(1 == _gAxisReverseFlag[Z_AXIS]) _gRawAccel[Z_AXIS] *= -1;
}


void _AccelGyro_Calibration()
{
    int                     i = 0;

    Serialprint(F(" *          Calibrating "));
    
    for(i=0 ; i<=Z_AXIS ; i++)
        _gCalibMeanGyro[i] = 0.0;
    
    for(i=0 ; i<2000 ; i++)
    {
        // We don't want the esc's to be beeping annoyingly.
        // So let's give them a 1000us puls while calibrating the gyro.
        // Set Digital Port 8, 9, 10, and 11 as high.
        PORTB |= B00001111;
        delayMicroseconds(1000);
        
        //Set digital poort 8, 9, 10, and 11 low.
        PORTB &= B11110000;
        delayMicroseconds(3000);

        Wire.beginTransmission(0x68);
        Wire.write(0x43);                                               // starting with register 0x43 (GYRO_XOUT_H)
        Wire.endTransmission();
        Wire.requestFrom(0x68, 6);                                      // request a total of 6 registers
        
        _gCalibMeanGyro[X_AXIS] += (Wire.read()<<8 | Wire.read());      // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
        _gCalibMeanGyro[Y_AXIS] += (Wire.read()<<8 | Wire.read());      // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
        _gCalibMeanGyro[Z_AXIS] += (Wire.read()<<8 | Wire.read());      // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
        
        Wire.beginTransmission(0x68);
        Wire.write(0x3B);                                               // starting with register 0x3B (ACCEL_XOUT_H)
        Wire.endTransmission();
        Wire.requestFrom(0x68, 6);                                      // request a total of 6 registers
        
        _gCalibMeanAccel[X_AXIS] += (Wire.read()<<8 | Wire.read());     // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
        _gCalibMeanAccel[Y_AXIS] += (Wire.read()<<8 | Wire.read());     // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
        _gCalibMeanAccel[Z_AXIS] += (Wire.read()<<8 | Wire.read());     // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
        
        if(0 == (i % 20))
        {
            _LED_Blink(1, 0, 0, 100000);                                    // Blink LED with Period as 100ms
            Serialprint(F("."));
        }
    }
    
    for(i=0 ; i<=Z_AXIS ; i++)
    {
        _gCalibMeanGyro[i] /= 2000;
        _gCalibMeanAccel[i] /= 2000;
    }
    
    Serialprintln(F("."));

    Serialprintln(F(""));
    Serialprintln(F("        Calibrated Gyro & Accel Value"));
    Serialprint(F("             Gyro:"));
    Serialprint(F("  X:")); Serialprint(_gCalibMeanGyro[X_AXIS]);
    Serialprint(F("  Y:")); Serialprint(_gCalibMeanGyro[Y_AXIS]);
    Serialprint(F("  Z:")); Serialprintln(_gCalibMeanGyro[Z_AXIS]);
    Serialprint(F("             Accel:"));
    Serialprint(F("  X:")); Serialprint(_gCalibMeanAccel[X_AXIS]);
    Serialprint(F("  Y:")); Serialprint(_gCalibMeanAccel[Y_AXIS]);
    Serialprint(F("  Z:")); Serialprintln(_gCalibMeanAccel[Z_AXIS]);

    Serialprintln(F(" *          => Done!!   "));
}


#if USE_MPU6050_DMP
void dmpDataReady()
{
    mpuInterrupt = true;
}
            
                       
void _AccelGyro_WaitDMPIntrrupt()
{
    if (!dmpReady)
        return;
    
    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize)
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
        Serial.println(F("FIFO overflow!"));
        
        // otherwise, check for DMP data ready interrupt (this should happen frequently)
    }
    else if(mpuIntStatus & 0x02)
    {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
        
        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;
    }
}
                       

void _AccelGyro_GetQuaternion()
{
    _AccelGyro_WaitDMPIntrrupt();
    
    mpu.dmpGetQuaternion(&q, fifoBuffer);
}


void _AccelGyro_GetEularAngle()
{
    _AccelGyro_GetQuaternion();
    
    mpu.dmpGetEuler(euler, &q);
}


void _AccelGyro_GetRPY()
{
    _AccelGyro_GetQuaternion();
    
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
}
#endif
            
#endif /* MPU6050_Controller_h */



