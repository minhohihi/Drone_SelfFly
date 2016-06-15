//
//  MPU6050_Controller.h
//  SelfFly
//
//  Created by Maverick on 2016. 1. 21..
//

#ifndef __MPU6050_CONTROL__
#define __MPU6050_CONTROL__


void _AccelGyro_GetData();
void _AccelGyro_Calibration();

void _AccelGyro_Initialize()
{
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
//    nAccelGyroHndl.setRate(1);                                            // Sample Rate (500Hz = 1Hz Gyro SR / 1+1)
//    nAccelGyroHndl.setDLPFMode(MPU6050_DLP_PRECISION);
//    nAccelGyroHndl.setFullScaleGyroRange(GYRO_FS_PRECISIOM);
//    nAccelGyroHndl.setFullScaleAccelRange(ACCEL_FS_PRECISIOM);

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
    if(Wire.read() != 0x08){             
        digitalWrite(12,HIGH);           
        while(1)delay(10);               
    }
    
    Wire.beginTransmission(0x68);
    Wire.write(0x1A);                    
    Wire.write(0x03);                    
    Wire.endTransmission();
    
    nEstimatedRPY[0] = nEstimatedRPY[1] = nEstimatedRPY[2] = 0.0;

    delay(300);
    
    // Calibration
    _AccelGyro_Calibration();
    
    delay(300);
    
    Serialprintln(F(" *            => Done!!   "));
    
    return;
}


void _AccelGyro_GetData()
{
    Wire.beginTransmission(0x68);
    Wire.write(0x43);                                               // starting with register 0x43 (GYRO_XOUT_H)
    Wire.endTransmission();
    Wire.requestFrom(0x68, 6);                                      // request a total of 14 registers

    nRawGyro[X_AXIS] = (Wire.read()<<8 | Wire.read());              // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    nRawGyro[Y_AXIS] = (Wire.read()<<8 | Wire.read());              // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    nRawGyro[Z_AXIS] = (Wire.read()<<8 | Wire.read());              // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
    
    nRawGyro[X_AXIS] -= nCalibMeanGyro[X_AXIS];
    if(1 == nAxisReverseFlag[X_AXIS]) nRawGyro[X_AXIS] *= -1;
    nRawGyro[Y_AXIS] -= nCalibMeanGyro[Y_AXIS];
    if(1 == nAxisReverseFlag[Y_AXIS]) nRawGyro[Y_AXIS] *= -1;
    nRawGyro[Z_AXIS] -= nCalibMeanGyro[Z_AXIS];
    if(1 == nAxisReverseFlag[Z_AXIS]) nRawGyro[Z_AXIS] *= -1;
}


void _AccelGyro_Calibration()
{
    int                     i = 0;

    Serialprint(F(" *          Calibrating "));
    
    for(i=0 ; i<=Z_AXIS ; i++)
        nCalibMeanGyro[i] = 0.0;
    
    for(i=0 ; i<2000 ; i++)
    {
        _AccelGyro_GetData();
        
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
        
        nCalibMeanGyro[X_AXIS] += (Wire.read()<<8 | Wire.read());       // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
        nCalibMeanGyro[Y_AXIS] += (Wire.read()<<8 | Wire.read());       // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
        nCalibMeanGyro[Z_AXIS] += (Wire.read()<<8 | Wire.read());       // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
        
        if(0 == (i % 20))
        {
            _LED_Blink(1, 0, 0, 100000);                                    // Blink LED with Period as 100ms
            Serialprint(F("."));
        }
    }
    
    for(i=0 ; i<=Z_AXIS ; i++)
        nCalibMeanGyro[i] /= 2000;
    
    Serialprintln(F("."));

    Serialprintln(F(" *          => Done!!   "));
}

#endif /* MPU6050_Controller_h */

