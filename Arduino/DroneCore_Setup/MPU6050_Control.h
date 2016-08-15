//
//  MPU6050_Controller.h
//  SelfFly
//
//  Created by Maverick on 2016. 1. 21..
//

#ifndef __MPU6050_CONTROL__
#define __MPU6050_CONTROL__

void _AccelGyro_GetGyroData();
void _AccelGyro_GetAccelData();
void _AccelGyro_GetGyroAccelData();
void _AccelGyro_Calibration();

void _AccelGyro_Initialize()
{
    int                 i = 0;
    
    Serialprintln(F(" *      5. Start MPU6050 Module Initialization   "));

    Wire.beginTransmission(0x68);
    Wire.write(0x6B);                    
    Wire.write(0x00);                    
    Wire.endTransmission();              
    
    Wire.beginTransmission(0x68);
    Wire.write(0x1B);                    
    Wire.write(0x08);                    
    Wire.endTransmission();              
    
    Wire.beginTransmission(0x68);
    Wire.write(0x1C);
    Wire.write(0x10);
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
    
    delay(300);
    
    Serialprintln(F(" *            => Done!!   "));
    
    return;
}


void _AccelGyro_GetGyroData()
{
    int               nRawGyroX, nRawGyroY, nRawGyroZ;
  
    Wire.beginTransmission(0x68);
    Wire.write(0x43);                                               // starting with register 0x43 (GYRO_XOUT_H)
    Wire.endTransmission();
    Wire.requestFrom(0x68, 6);                                      // request a total of 6 registers

    nRawGyroX = (Wire.read()<<8 | Wire.read());                     // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    nRawGyroY = (Wire.read()<<8 | Wire.read());                     // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    nRawGyroZ = (Wire.read()<<8 | Wire.read());                     // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
    
    _gRawGyro[X_AXIS] = nRawGyroX - _gCalibMeanGyro[X_AXIS];
    if(1 == _gAxisReverseFlag[X_AXIS]) _gRawGyro[X_AXIS] *= -1;
    _gRawGyro[Y_AXIS] = nRawGyroY - _gCalibMeanGyro[Y_AXIS];
    if(1 == _gAxisReverseFlag[Y_AXIS]) _gRawGyro[Y_AXIS] *= -1;
    _gRawGyro[Z_AXIS] = nRawGyroZ - _gCalibMeanGyro[Z_AXIS];
    if(1 == _gAxisReverseFlag[Z_AXIS]) _gRawGyro[Z_AXIS] *= -1;
}

                   
void _AccelGyro_GetAccelData()
{
    int               nRawAccelX, nRawAccelY, nRawAccelZ;

    Wire.beginTransmission(0x68);
    Wire.write(0x3B);                                               // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission();
    Wire.requestFrom(0x68, 6);                                      // request a total of 6 registers
    
    nRawAccelX = (Wire.read()<<8 | Wire.read());                    // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
    nRawAccelY = (Wire.read()<<8 | Wire.read());                    // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    nRawAccelZ = (Wire.read()<<8 | Wire.read());                    // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    
    _gRawAccel[X_AXIS] = nRawAccelX;// - _gCalibMeanAccel[X_AXIS];
    if(1 == _gAxisReverseFlag[X_AXIS]) _gRawAccel[X_AXIS] *= -1;
    _gRawAccel[Y_AXIS] = nRawAccelY;// - _gCalibMeanAccel[Y_AXIS];
    if(1 == _gAxisReverseFlag[Y_AXIS]) _gRawAccel[Y_AXIS] *= -1;
    _gRawAccel[Z_AXIS] = nRawAccelZ;// - _gCalibMeanAccel[Z_AXIS];
    if(1 == _gAxisReverseFlag[Z_AXIS]) _gRawAccel[Z_AXIS] *= -1;
}

void _AccelGyro_GetGyroAccelData()
{
    int                     nRawGyroX, nRawGyroY, nRawGyroZ;
    int                     nRawAccelX, nRawAccelY, nRawAccelZ;

    Wire.beginTransmission(0x68);
    Wire.write(0x3B);                                               // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission();
    Wire.requestFrom(0x68, 14);                                     // request a total of 14 registers
    
    nRawAccelX = (Wire.read()<<8 | Wire.read());                    // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
    nRawAccelY = (Wire.read()<<8 | Wire.read());                    // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    nRawAccelZ = (Wire.read()<<8 | Wire.read());                    // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    Wire.read(); Wire.read();
    nRawGyroX = (Wire.read()<<8 | Wire.read());                     // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    nRawGyroY = (Wire.read()<<8 | Wire.read());                     // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    nRawGyroZ = (Wire.read()<<8 | Wire.read());                     // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  
    _gRawAccel[X_AXIS] = nRawAccelX;// - _gCalibMeanAccel[X_AXIS];
    if(1 == _gAxisReverseFlag[X_AXIS]) _gRawAccel[X_AXIS] *= -1;
    _gRawAccel[Y_AXIS] = nRawAccelY;// - _gCalibMeanAccel[Y_AXIS];
    if(1 == _gAxisReverseFlag[Y_AXIS]) _gRawAccel[Y_AXIS] *= -1;
    _gRawAccel[Z_AXIS] = nRawAccelZ;// - _gCalibMeanAccel[Z_AXIS];
    if(1 == _gAxisReverseFlag[Z_AXIS]) _gRawAccel[Z_AXIS] *= -1;

    _gRawGyro[X_AXIS] = nRawGyroX - _gCalibMeanGyro[X_AXIS];
    if(1 == _gAxisReverseFlag[X_AXIS]) _gRawGyro[X_AXIS] *= -1;
    _gRawGyro[Y_AXIS] = nRawGyroY - _gCalibMeanGyro[Y_AXIS];
    if(1 == _gAxisReverseFlag[Y_AXIS]) _gRawGyro[Y_AXIS] *= -1;
    _gRawGyro[Z_AXIS] = nRawGyroZ - _gCalibMeanGyro[Z_AXIS];
    if(1 == _gAxisReverseFlag[Z_AXIS]) _gRawGyro[Z_AXIS] *= -1;    
}

void _AccelGyro_Calibration()
{
    int                     i = 0, j = 0;
    int                     nRawGyro[3];
    int                     nRawAccel[3];

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
        Wire.write(0x3B);                                               // starting with register 0x3B (ACCEL_XOUT_H)
        Wire.endTransmission();
        Wire.requestFrom(0x68, 14);                                     // request a total of 14 registers

        for(j=0 ; j<3 ; j++)
            nRawAccel[j] = (Wire.read()<<8 | Wire.read());
        Wire.read(); Wire.read();
        for(j=0 ; j<3 ; j++)
            nRawGyro[j] = (Wire.read()<<8 | Wire.read());
        
        for(j=0 ; j<3 ; j++)
            _gCalibMeanAccel[j] += nRawAccel[j];
        
        for(j=0 ; j<3 ; j++)
            _gCalibMeanGyro[j] += nRawGyro[j];
        
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
#endif /* MPU6050_Controller_h */



