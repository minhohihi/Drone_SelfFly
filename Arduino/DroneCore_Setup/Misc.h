//
//  Misc.h
//  SelfFly
//
//  Created by Maverick on 2016. 1. 21..
//

#ifndef __MISC__
#define __MISC__


void _Check_Drone_Status()
{
    int32_t                 nLEDPeriod = 500000;
    
    if((1050 > _gCompensatedRCVal[CH_TYPE_THROTTLE]) && (1050 > _gCompensatedRCVal[CH_TYPE_YAW]))
        _gDroneStatus = DRONESTATUS_READY;
    
    //When yaw stick is back in the center position start the motors (step 2).
    if((DRONESTATUS_READY == _gDroneStatus) &&
       (1050 > _gCompensatedRCVal[CH_TYPE_THROTTLE]) && (1400 < _gCompensatedRCVal[CH_TYPE_YAW]))
    {
        int         i = 0;
        
        _gDroneStatus = DRONESTATUS_START;
        
        //Reset the pid controllers for a bumpless start.
        for(i=0 ; i<3 ; i++)
        {
            _gRPY_PID[i].nP_ErrRate = 0.0;
            _gRPY_PID[i].nI_ErrRate = 0.0;
            _gRPY_PID[i].nD_ErrRate = 0.0;
            _gRPY_PID[i].nPrevErrRate = 0.0;
            _gRPY_PID[i].nBalance = 0.0;
        }
    }
    
    //Stopping the motors: throttle low and yaw right.
    if((DRONESTATUS_START == _gDroneStatus) &&
       (1050 > _gCompensatedRCVal[CH_TYPE_THROTTLE]) && (1950 < _gCompensatedRCVal[CH_TYPE_YAW]))
        _gDroneStatus = DRONESTATUS_STOP;
    
    // Set LED Period as 200ms When Low Voltage
    //if((1030 > _gCurrBatteryVolt) && (600 < _gCurrBatteryVolt))
    //    nLEDPeriod = 200000;

    if(DRONESTATUS_STOP == _gDroneStatus)
        _LED_Blink(1, 0, 0, nLEDPeriod);                                 // Turn On a Red Light
    else if(DRONESTATUS_READY == _gDroneStatus)
        _LED_Blink(0, 1, 0, nLEDPeriod);                                 // Turn On a Green Light
    else if(DRONESTATUS_START == _gDroneStatus)
        _LED_Blink(0, 0, 1, nLEDPeriod);                                 // Turn On a Blue Light
    
    return;
}


void _GetRawSensorData()
{
    _gPrevSensorCapTime = _gCurrSensorCapTime;
    _gCurrSensorCapTime = micros();

    _gDiffTime = (_gCurrSensorCapTime - _gPrevSensorCapTime) / 1000000.0;
    
    // Get Gyro Raw Data
    _AccelGyro_GetGyroData();

    // Get Accel Raw Data
    _AccelGyro_GetAccelData();

    // Get Magnetic Raw Data
    //_Mag_GetData();

    // Get Barometer Raw Data
    //_Barometer_GetData();

    // Get Sonar Raw Data
    //_Sonar_GetData();
    //_Sonar_GetData_WithPeriod();
}


void _AccelGyro_CheckAxis(int nAxisIdx)
{
    unsigned long           nCurrTime;
    byte                    nTmpAxis;

    _gAngleRoll = 0;
    _gAnglePitch = 0;
    _gAngleYaw = 0;
    
    if(0 == nAxisIdx)
        Serialprintln(F("1. Please Lift the Left Wing to 30 degree for 10 seconds"));
    else if(1 == nAxisIdx)
        Serialprintln(F("2. Please Lift the Nose to 30 degree for 10 seconds"));
    else if(2 == nAxisIdx)
        Serialprintln(F("3. Please Rotate the Nose to the right to 30 degree for 10 seconds"));
    
    nCurrTime = millis() + 10000;
    
    while(nCurrTime > millis() &&
          (_gAngleRoll > -30)  && (_gAngleRoll < 30) &&
          (_gAnglePitch > -30) && (_gAnglePitch < 30) &&
          (_gAngleYaw > -30)   && (_gAngleYaw < 30))
    {
        // Get Sensor (Gyro / Accel / Megnetic / Baro / Temp)
        _GetRawSensorData();
        
        // Calculate Roll, Pitch, and Yaw by Quaternion
        _gAngleRoll  += _gRawGyro[X_AXIS] * 0.0000611;                   //Calculate the traveled roll angle and add
        _gAnglePitch += _gRawGyro[Y_AXIS] * 0.0000611;                  //Calculate the traveled pitch angle and add this to the angle_pitch variable.
        _gAngleYaw   += _gRawGyro[Z_AXIS] * 0.0000611;                   //Calculate the traveled roll angle and add
        
        delayMicroseconds(3600);
    }
    
    //Assign the moved axis to the orresponding function (pitch, roll, yaw)
    if(((_gAngleRoll < -30) || (_gAngleRoll > 30)) && (_gAnglePitch > -30) && (_gAnglePitch < 30) && (_gAngleYaw > -30) && (_gAngleYaw < 30))
    {
        nTmpAxis = B00000000;
        if(_gAngleRoll < 0)
            nTmpAxis |= B10000000;
    }
    
    if((_gAngleRoll > -30) && (_gAngleRoll < 30) && ((_gAnglePitch < -30) || (_gAnglePitch > 30)) && (_gAngleYaw > -30) && (_gAngleYaw < 30))
    {
        nTmpAxis = B00000001;
        if(_gAnglePitch < 0)
            nTmpAxis |= B10000000;
    }
    
    if((_gAngleRoll > -30) && (_gAngleRoll < 30) && (_gAnglePitch > -30) && (_gAnglePitch < 30) && ((_gAngleYaw < -30) || (_gAngleYaw > 30)))
    {
        nTmpAxis = B00000010;
        if(_gAngleYaw < 0)
            nTmpAxis |= B10000000;
    }
   
    if(0 == nAxisIdx)
        _gGyroAccelAxis[X_AXIS] = nTmpAxis;
    else if(1 == nAxisIdx)
        _gGyroAccelAxis[Y_AXIS] = nTmpAxis;
    else if(2 == nAxisIdx)
        _gGyroAccelAxis[Z_AXIS] = nTmpAxis;
    
    Serialprint(F("      Gyro "));
    if(0 == nAxisIdx)
    {
        Serialprint(F("Roll Axis is Detected...   "));
        Serialprint(_gAngleRoll);
    }
    else if(1 == nAxisIdx)
    {
        Serialprint(F("Pitch Axis is Detected...   "));
        Serialprint(_gAnglePitch);
    }
    else if(2 == nAxisIdx)
    {
         Serialprint(F("Yaw Axis is Detected...   "));
         Serialprint(_gAngleYaw);
    }

    if(B10000000 & nTmpAxis)
        Serialprint(F("(Reversed!!!)"));

    Serialprintln(F(" "));

    nCurrTime = millis() + 5000;
    while(nCurrTime > millis() &&
          (_gAngleRoll > -5)  && (_gAngleRoll < 5) &&
          (_gAnglePitch > -5) && (_gAnglePitch < 5) &&
          (_gAngleYaw > -5)   && (_gAngleYaw < 5))
    {
        // Get Sensor (Gyro / Accel / Megnetic / Baro / Temp)
        _GetRawSensorData();
        
        // Calculate Roll, Pitch, and Yaw by Quaternion
        _gAngleRoll  += _gRawGyro[X_AXIS] * 0.0000611;                   //Calculate the traveled roll angle and add
        _gAnglePitch += _gRawGyro[Y_AXIS] * 0.0000611;                  //Calculate the traveled pitch angle and add this to the angle_pitch variable.
        _gAngleYaw   += _gRawGyro[Z_AXIS] * 0.0000611;                   //Calculate the traveled roll angle and add
        
        delayMicroseconds(3600);
    }
}

       
void _Read_EEPROM()
{
    byte                nEEPRomData[EEPROM_SIZE];                // EEPROM Data
    int                 i = 0;
    
    Serialprintln(F(" *      1. Start Reading EEPROOM Data   "));
    
    for(i=0 ; i<EEPROM_SIZE ; i++)
        nEEPRomData[i] = EEPROM.read(i);

    for(i=0 ; i<CH_TYPE_MAX ; i++)
    {
        const int           nChannel = i + 1;
        
        _gRCSignal_L[i]    = (nEEPRomData[nChannel * 2 + 15] << 8) | nEEPRomData[nChannel * 2 + 14];
        _gRCSignal_M[i]    = (nEEPRomData[nChannel * 2 - 1] << 8) | nEEPRomData[nChannel * 2 - 2];
        _gRCSignal_H[i]    = (nEEPRomData[nChannel * 2 + 7] << 8) | nEEPRomData[nChannel * 2 + 6];
        _gRCReverseFlag[i] = !(!(nEEPRomData[nChannel + 23] & B10000000));
    }
    
    for(i=0 ; i<3 ; i++)
        _gAxisReverseFlag[i] = !(!(nEEPRomData[28 + i] & B10000000));
    
    delay(300);
    
    Serialprintln(F(" *            => Done!!   "));
}


void _Write_EEPROM(int nStartAddress)
{
    int                 i = 0;
    int                 nEEPRomAddress = 0;
    
    Serialprintln(F("   ")); Serialprintln(F("   ")); Serialprintln(F("   ")); Serialprintln(F("   "));
    Serialprintln(F("********************************************************************"));
    Serialprintln(F(" *         Writing Drone Setting to EEPROM          "));
    Serialprintln(F("********************************************************************"));

    // Write Range of Transmitter
    if(EEPROM_DATA_RC_CH0_LOW_H == nStartAddress)
    {
        Serialprintln(F(" *            => Write Transmitter Range   "));
        
        nEEPRomAddress = EEPROM_DATA_RC_CH0_LOW_H;
        
        for(i=0 ; i<4 ; i++)
        {
            EEPROM.write(nEEPRomAddress,   _gRCSignal_L[i] >> 8);
            EEPROM.write(nEEPRomAddress+1, _gRCSignal_L[i] & B11111111);
            EEPROM.write(nEEPRomAddress+2, _gRCSignal_M[i] >> 8);
            EEPROM.write(nEEPRomAddress+3, _gRCSignal_M[i] & B11111111);
            EEPROM.write(nEEPRomAddress+4, _gRCSignal_H[i] >> 8);
            EEPROM.write(nEEPRomAddress+5, _gRCSignal_H[i] & B11111111);
            
            nEEPRomAddress += 6;
        }
    }
    else if(EEPROM_DATA_GYRO_OFFSET_X == nStartAddress)
    {
        Serialprintln(F(" *            => Write Accel &Gyro Offset  "));
        
        nEEPRomAddress = EEPROM_DATA_GYRO_OFFSET_X;
        
        EEPROM.write(nEEPRomAddress,   _gRCSignal_L[i] >> 8);
        EEPROM.write(nEEPRomAddress+1, _gRCSignal_L[i] & B11111111);
        EEPROM.write(nEEPRomAddress+2, _gRCSignal_M[i] >> 8);
        EEPROM.write(nEEPRomAddress+3, _gRCSignal_M[i] & B11111111);
        EEPROM.write(nEEPRomAddress+4, _gRCSignal_H[i] >> 8);
        EEPROM.write(nEEPRomAddress+5, _gRCSignal_H[i] & B11111111);
    }
    
    
    delay(300);
    
    Serialprintln(F(" *            => Done!!   "));
}


void _Check_BatteryVolt()
{
    _gCurrBatteryVolt = (0.92 * _gCurrBatteryVolt) + (analogRead(PIN_CHECK_POWER_STAT) + 65) * 0.09853;

    //Serialprintln(_gCurrBatteryVolt);
}


float _Clip3Float(float nValue, float nMIN, float nMAX)
{
    float               nClipVal = nValue;

    if(nValue < nMIN)
        nClipVal = nMIN;
    else if(nValue > nMAX)
        nClipVal = nMAX;

    return nClipVal;
}


int _Clip3Int(int nValue, int nMIN, int nMAX)
{
    int                 nClipVal = nValue;

    if(nValue < nMIN)
        nClipVal = nMIN;
    else if(nValue > nMAX)
        nClipVal = nMAX;

    return nClipVal;
}


/**
 * Fast inverse square root implementation
 * @see http://en.wikipedia.org/wiki/Fast_inverse_square_root
 */
float _InvSqrt(float nNumber)
{
    long                i = 0;
    float               x = 0.0f, y = 0.0f;
    const float         f = 1.5F;

    x = nNumber * 0.5F;
    y = nNumber;
    i = * ( long * ) &y;
    i = 0x5f375a86 - ( i >> 1 );
    y = * ( float * ) &i;
    y = y * ( f - ( x * y * y ) );

    return y;
}

#endif /* Misc_h */

