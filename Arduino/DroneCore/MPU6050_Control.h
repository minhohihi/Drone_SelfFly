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
void _AccelGyro_DispStatus(int nCase);

void _AccelGyro_Initialize()
{
    int                 i = 0;
    
    _AccelGyro_DispStatus(0);

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

    // For Magnetic Sensor
    I2Cdev::writeBit(0x68, 0x6A, 5, false);
    I2Cdev::writeBit(0x68, 0x37, 1, true);
    I2Cdev::writeBit(0x68, 0x6B, 6, false);

    delay(300);
    
    // Calibration
    //_AccelGyro_Calibration();
    
    delay(300);
    
    _AccelGyro_DispStatus(3);
    
    return;
}


void _AccelGyro_GetGyroData()
{
    int               nRawGyro[3];
  
    Wire.beginTransmission(0x68);
    Wire.write(0x43);                                                           // starting with register 0x43 (GYRO_XOUT_H)
    Wire.endTransmission();
    Wire.requestFrom(0x68, 6);                                                  // request a total of 6 registers

    nRawGyro[X_AXIS] = (Wire.read()<<8 | Wire.read());                          // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    nRawGyro[Y_AXIS] = (Wire.read()<<8 | Wire.read());                          // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    nRawGyro[Z_AXIS] = (Wire.read()<<8 | Wire.read());                          // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
    
    _gRawGyro[X_AXIS] = (float)nRawGyro[X_AXIS] - _gCalibMeanGyro[X_AXIS];
    _gRawGyro[Y_AXIS] = (float)nRawGyro[Y_AXIS] - _gCalibMeanGyro[Y_AXIS];
    _gRawGyro[Z_AXIS] = (float)nRawGyro[Z_AXIS] - _gCalibMeanGyro[Z_AXIS];
}

                   
void _AccelGyro_GetAccelData()
{
    int               nRawAccel[3];

    Wire.beginTransmission(0x68);
    Wire.write(0x3B);                                                           // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission();
    Wire.requestFrom(0x68, 6);                                                  // request a total of 6 registers
    
    nRawAccel[X_AXIS] = (Wire.read()<<8 | Wire.read());                         // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
    nRawAccel[Y_AXIS] = (Wire.read()<<8 | Wire.read());                         // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    nRawAccel[Z_AXIS] = (Wire.read()<<8 | Wire.read());                         // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    
    _gRawAccel[X_AXIS] = (float)nRawAccel[X_AXIS] - _gCalibMeanAccel[X_AXIS];
    _gRawAccel[Y_AXIS] = (float)nRawAccel[Y_AXIS] - _gCalibMeanAccel[Y_AXIS];
    _gRawAccel[Z_AXIS] = (float)nRawAccel[Z_AXIS];// - _gCalibMeanAccel[Z_AXIS];
}

void _AccelGyro_GetGyroAccelData()
{
    int                     nRawGyro[3];
    int                     nRawAccel[3];
    int                     nTemperature = 0;

    Wire.beginTransmission(0x68);
    Wire.write(0x3B);                                                           // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission();
    Wire.requestFrom(0x68, 14);                                                 // request a total of 14 registers
    
    nRawAccel[X_AXIS] = (Wire.read()<<8 | Wire.read());                         // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
    nRawAccel[Y_AXIS] = (Wire.read()<<8 | Wire.read());                         // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    nRawAccel[Z_AXIS] = (Wire.read()<<8 | Wire.read());                         // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    nTemperature = (Wire.read()<<8 | Wire.read());
    nRawGyro[X_AXIS] = (Wire.read()<<8 | Wire.read());                          // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    nRawGyro[Y_AXIS] = (Wire.read()<<8 | Wire.read());                          // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    nRawGyro[Z_AXIS] = (Wire.read()<<8 | Wire.read());                          // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  
    _gRawAccel[X_AXIS] = (float)nRawAccel[X_AXIS] - _gCalibMeanAccel[X_AXIS];
    _gRawAccel[Y_AXIS] = (float)nRawAccel[Y_AXIS] - _gCalibMeanAccel[Y_AXIS];
    _gRawAccel[Z_AXIS] = (float)nRawAccel[Z_AXIS];// - _gCalibMeanAccel[Z_AXIS];
    _gTemperature = (float)nTemperature;
    _gRawGyro[X_AXIS] = (float)nRawGyro[X_AXIS] - _gCalibMeanGyro[X_AXIS];
    _gRawGyro[Y_AXIS] = (float)nRawGyro[Y_AXIS] - _gCalibMeanGyro[Y_AXIS];
    _gRawGyro[Z_AXIS] = (float)nRawGyro[Z_AXIS] - _gCalibMeanGyro[Z_AXIS];
}


void _AccelGyro_Calibration()
{
    int                     i = 0, j = 0;
    int                     nRawGyro[3] = {0, };
    int                     nRawAccel[3] = {0, };
    int                     nTemperature = 0;

    Serialprintln(F(" * Please wait for 10 seconds"));
    Serialprintln(F(" "));
        
    _AccelGyro_DispStatus(1);
    
    for(i=0 ; i<=Z_AXIS ; i++)
        _gCalibMeanGyro[i] = 0;
    
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
        Wire.write(0x3B);                                                       // starting with register 0x3B (ACCEL_XOUT_H)
        Wire.endTransmission();
        Wire.requestFrom(0x68, 14);                                             // request a total of 14 registers

        // Read 14 Bytes (6Btye as Accel, 2Bytes as Temp, 6Bytes as Gyro)
        for(j=0 ; j<3 ; j++)
            nRawAccel[j] = (Wire.read()<<8 | Wire.read());
        nTemperature = (Wire.read()<<8 | Wire.read());
        for(j=0 ; j<3 ; j++)
            nRawGyro[j] = (Wire.read()<<8 | Wire.read());
        
        for(j=0 ; j<3 ; j++)
        {
            _gCalibMeanAccel[j] += (float)nRawAccel[j];
            _gCalibMeanGyro[j] += (float)nRawGyro[j];
        }
        _gCalibMeanTemp += (float)nTemperature;
        
        if(0 == (i % 20))
            _LED_Blink(1, 0, 0, 100000);                                        // Blink LED with Period as 100ms

        if(0 == (i % 400))
            _AccelGyro_DispStatus(1);
    }
    
    for(i=0 ; i<=Z_AXIS ; i++)
    {
        _gCalibMeanGyro[i] /= 2000.0;
        _gCalibMeanAccel[i] /= 2000.0;
    }
    _gCalibMeanTemp /= 2000.0;
    
    _AccelGyro_DispStatus(2);
}


void _AccelGyro_CheckAxis(int nAxisIdx)
{
    unsigned long           nCurrTime;
    byte                    nTmpAxis = B11111111, i = B11111111;
    float                   nAngle[3] = {0.0, 0.0, 0.0};
    
    if(0 == nAxisIdx)
    {
       // Check a Roll Axis
        Serialprintln(F("1. Please Lift the Left Wing to 30 degree for 10 seconds"));
        Serialprint(F("             Roll Axis is ["));
    }
    else if(1 == nAxisIdx)
    {
        // Check a Pitch Axis
        Serialprintln(F("2. Please Lift the Nose to 30 degree for 10 seconds"));
        Serialprint(F("             Picth Axis is ["));
    }
    else if(2 == nAxisIdx)
    {
        // Check a Yaw Axis
        Serialprintln(F("3. Please Rotate the Nose to the right to 30 degree for 10 seconds"));
        Serialprint(F("             Yaw Axis is ["));
    }
    
    nCurrTime = millis() + 10000;
    while(nCurrTime > millis() && (B11111111 == nTmpAxis))
    {
        // Get Sensor (Gyro / Accel / Megnetic / Baro / Temp)
        _AccelGyro_GetGyroAccelData();
        
        // Calculate Roll, Pitch, and Yaw by Quaternion
        nAngle[X_AXIS] += _gRawGyro[X_AXIS] * 0.0000611;
        nAngle[Y_AXIS] += _gRawGyro[Y_AXIS] * 0.0000611;
        nAngle[Z_AXIS] += _gRawGyro[Z_AXIS] * 0.0000611;

        for(i=0 ; i<3 ; i++)
        {
            if((nAngle[i] < -15) || (nAngle[i] > 15))
            {
                nTmpAxis = i;
                if(nAngle[i] < 0)
                    nTmpAxis |= B10000000;
                break;
            }
        }
        
        delayMicroseconds(3550);
    }
    
    _gGyroAccelAxis[nAxisIdx] = nTmpAxis;
    
    Serialprint(_gGyroAccelAxis[nAxisIdx] & B00000011);
    if(B10000000 & _gGyroAccelAxis[nAxisIdx])
        Serialprintln(F("] and Reversed"));
    else
        Serialprintln(F("] and Not Reversed"));

    Serialprintln(F("     Please Return to the Center"));
    Serialprintln(F(" "));

    nCurrTime = millis() + 10000;
    while(nCurrTime > millis())
    {
        // Get Sensor (Gyro / Accel / Megnetic / Baro / Temp)
        _AccelGyro_GetGyroAccelData();
        
        // Calculate Roll, Pitch, and Yaw by Quaternion
        nAngle[X_AXIS] += _gRawGyro[X_AXIS] * 0.0000611;
        nAngle[Y_AXIS] += _gRawGyro[Y_AXIS] * 0.0000611;
        nAngle[Z_AXIS] += _gRawGyro[Z_AXIS] * 0.0000611;
        
        if(((-5 <= nAngle[X_AXIS]) && (nAngle[X_AXIS] <= 5)) &&
           ((-5 <= nAngle[Y_AXIS]) && (nAngle[Y_AXIS] <= 5)) &&
           ((-5 <= nAngle[Z_AXIS]) && (nAngle[Z_AXIS] <= 5)))
            break;

        delayMicroseconds(3550);
    }
}


void _AccelGyro_DispStatus(int nCase)
{
    static int nCnt = 0;
    
    #if PRINT_SERIAL
        if(0 == nCase)
        {
            Serialprint(F(" *      "));
            Serialprint(_gDroneInitStep++);        
            Serialprintln(F(". Start MPU6050 Module Initialization   "));
        }
        else if(1 == nCase)
        {
            if(0 == nCnt++)
                Serialprint(F(" *          Calibrating "));
            else
                Serialprint(F("."));
        }
        else if(2 == nCase)
        {
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
        }
        else if(3 == nCase)
            Serialprintln(F(" *          => Done!!   "));
    #elif USE_LCD_DISPLAY
    {
        if(0 == nCase)
        {
            delay(500);
            _gLCDHndl.clear();

            _gLCDHndl.setCursor(0, 0);
            _gLCDHndl.print(_gDroneInitStep++);
            _gLCDHndl.setCursor(1, 0);
            _gLCDHndl.print(".Init MPU6050");
        }
        else if(1 == nCase)
        {
            _gLCDHndl.setCursor(nCnt++, 1);
            if(1 == nCnt)
            {
                delay(500);              
                _gLCDHndl.print("Calib:");
                nCnt += 5;
            }
            else
                _gLCDHndl.print(".");
        }
        else if(3 == nCase)
        {
            _gLCDHndl.setCursor(nCnt, 1);
            _gLCDHndl.print("Done!");
            delay(1000);
        }
    }
    #endif
}

#endif /* MPU6050_Controller_h */


