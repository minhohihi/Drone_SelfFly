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
    
    if((1050 > _gCompensatedRCVal[_gRCChMap[CH_TYPE_THROTTLE]]) && (1050 > _gCompensatedRCVal[_gRCChMap[CH_TYPE_YAW]]))
        _gDroneStatus = DRONESTATUS_READY;
    
    //When yaw stick is back in the center position start the motors (step 2).
    if((DRONESTATUS_READY == _gDroneStatus) &&
       (1050 > _gCompensatedRCVal[_gRCChMap[CH_TYPE_THROTTLE]]) && (1400 < _gCompensatedRCVal[_gRCChMap[CH_TYPE_YAW]]))
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
       (1050 > _gCompensatedRCVal[_gRCChMap[CH_TYPE_THROTTLE]]) && (1950 < _gCompensatedRCVal[_gRCChMap[CH_TYPE_YAW]]))
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


void _Wait(unsigned long nMicroTime)
{
    while(micros() - _gESCLoopTimer < nMicroTime);
    _gESCLoopTimer = micros();
}

       
void _EEPROM_Read_Tmp()
{
    byte                nEEPRomData[EEPROM_SIZE];                // EEPROM Data
    int                 i = 0;
    
    Serialprintln(F(" *      1. Start Reading EEPROOM Data   "));
    
    for(i=0 ; i<EEPROM_SIZE ; i++)
        nEEPRomData[i] = EEPROM.read(i);

    for(i=0 ; i<CH_TYPE_MAX ; i++)
    {
        const int           nChannel = i + 1;
        
        _gRCSignal_L[i] = (nEEPRomData[nChannel * 2 + 15] << 8) | nEEPRomData[nChannel * 2 + 14];
        _gRCSignal_M[i] = (nEEPRomData[nChannel * 2 - 1] << 8) | nEEPRomData[nChannel * 2 - 2];
        _gRCSignal_H[i] = (nEEPRomData[nChannel * 2 + 7] << 8) | nEEPRomData[nChannel * 2 + 6];
        _gRCRvrsFlag[i] = !(!(nEEPRomData[nChannel + 23] & B10000000));
    }
    
    for(i=0 ; i<3 ; i++)
        _gMPUAxisRvrsFlag[i] = !(!(nEEPRomData[28 + i] & B10000000));
    
    delay(300);
    
    Serialprintln(F(" *            => Done!!   "));
}


int _EEPROM_Read(int nStartAddress, int nValidMode)
{
    byte                nEEPRomData[EEPROM_SIZE];                // EEPROM Data
    int                 i = 0;
    int                 nValidationChk = 0;
    int                 nEEPRomAddress = 0;
    
    // Write Range of Transmitter
    if(EEPROM_DATA_SIGN == nStartAddress)
    {
        Serialprintln(F(" *            => Read Drone Signiture   "));
        
        nEEPRomAddress = EEPROM_DATA_SIGN;
        
        if('M' != EEPROM.read(nEEPRomAddress++)) nValidationChk = -1;
        if('a' != EEPROM.read(nEEPRomAddress++)) nValidationChk = -1;
        if('v' != EEPROM.read(nEEPRomAddress++)) nValidationChk = -1;
        if('e' != EEPROM.read(nEEPRomAddress++)) nValidationChk = -1;
        if('r' != EEPROM.read(nEEPRomAddress++)) nValidationChk = -1;
        if('i' != EEPROM.read(nEEPRomAddress++)) nValidationChk = -1;
        if('c' != EEPROM.read(nEEPRomAddress++)) nValidationChk = -1;
        if('k' != EEPROM.read(nEEPRomAddress++)) nValidationChk = -1;
        
        if(0 == nValidationChk)
            Serialprint(F(" *                Verified!! You can Fly, Maverick!!     "));
    }
    else if(EEPROM_DATA_MPU_AXIS == nStartAddress)
    {
        Serialprint(F(" *            => Read MPU Type   "));
        
        nEEPRomAddress = EEPROM_DATA_MPU_AXIS0_TYPE;
        
        for(i=0 ; i<3 ; i++)
        {
            byte            nTmpAccelGyroAxis;

            nTmpAccelGyroAxis = EEPROM.read(nEEPRomAddress++);
            if((1 == nValidMode) && (_gGyroAccelAxis[i] != nTmpAccelGyroAxis))
                nValidationChk = -1;
            else
            {
                _gGyroAccelAxis[i] = nTmpAccelGyroAxis;
                _gMPUAxisRvrsFlag[i] = !(!(_gGyroAccelAxis[i] & B10000000));
                _gGyroAccelAxis[i] &= B01111111;
                
                if((0 > _gGyroAccelAxis[i]) || (3 <= _gGyroAccelAxis[i]))
                    nValidationChk = -1;
                
                Serialprintln(F(" "));
                Serialprint(F("                  "));
                Serialprint(_gGyroAccelAxis[i]);
                Serialprint(F(" / "));
                Serialprint(_gMPUAxisRvrsFlag[i]);
            }
        }
        Serialprintln(F(" ")); Serialprintln(F(" "));
    }
    else if(EEPROM_DATA_RC_TYPE == nStartAddress)
    {
        Serialprint(F(" *            => Read Transmitter Type   "));
        
        nEEPRomAddress = EEPROM_DATA_RC_CH0_TYPE;
        
        for(i=0 ; i<5 ; i++)
        {
            byte            nTmpRCChAxis;

            nTmpRCChAxis = EEPROM.read(nEEPRomAddress++);
            if((1 == nValidMode) && (_gRCChMap[i] != nTmpRCChAxis))
                nValidationChk = -1;
            else
            {
                _gRCChMap[i] = nTmpRCChAxis;
                _gRCRvrsFlag[i] = (!(!(_gRCChMap[i] & B10000000)));
                _gRCChMap[i] &= B01111111;
                
                if((0 > _gRCChMap[i]) || (5 <= _gRCChMap[i]))
                    nValidationChk = -1;

                Serialprintln(F(" "));
                Serialprint(F("                  "));
                Serialprint(_gRCChMap[i]);
                Serialprint(F(" / "));
                Serialprint(_gRCRvrsFlag[i]);
            }
        }
        Serialprintln(F(" ")); Serialprintln(F(" "));
    }
    else if(EEPROM_DATA_RC_RANGE == nStartAddress)
    {
        Serialprint(F(" *            => Read Transmitter Range   "));
        
        nEEPRomAddress = EEPROM_DATA_RC_CH0_LOW_H;
        
        for(i=0 ; i<4 ; i++)
        {
            int             nTmpL, nTmpM, nTmpH;
            
            nTmpL = (EEPROM.read(nEEPRomAddress++) << 8) | EEPROM.read(nEEPRomAddress++);
            nTmpM = (EEPROM.read(nEEPRomAddress++) << 8) | EEPROM.read(nEEPRomAddress++);
            nTmpH = (EEPROM.read(nEEPRomAddress++) << 8) | EEPROM.read(nEEPRomAddress++);
            if((1 == nValidMode) &&
               ((_gRCSignal_L[i] != nTmpL) || (_gRCSignal_M[i] != nTmpM) || (_gRCSignal_H[i] != nTmpH)))
                nValidationChk = -1;
            else
            {
                _gRCSignal_L[i] = nTmpL;
                _gRCSignal_M[i] = nTmpM;
                _gRCSignal_H[i] = nTmpH;

                if((500 > _gRCSignal_L[i]) || (2100 <= _gRCSignal_L[i])) nValidationChk = -1;
                if((500 > _gRCSignal_M[i]) || (2100 <= _gRCSignal_M[i])) nValidationChk = -1;
                if((500 > _gRCSignal_H[i]) || (2100 <= _gRCSignal_H[i])) nValidationChk = -1;

                Serialprintln(F(" "));
                Serialprint(F("                  "));
                Serialprint(_gRCSignal_L[i]);
                Serialprint(F(" / "));
                Serialprint(_gRCSignal_M[i]);
                Serialprint(F(" / "));
                Serialprint(_gRCSignal_H[i]);
            }
        }
        Serialprintln(F(" ")); Serialprintln(F(" "));
    }
    
    delay(300);
    
    if(0 != nValidationChk)
        Serialprintln(F(" *              => Error! Invalid ROM Data!!   "));
    
    return nValidationChk;
}


void _EEPROM_Write(int nStartAddress)
{
    int                 i = 0;
    int                 nEEPRomAddress = 0;
    
    // Write Range of Transmitter
    if(EEPROM_DATA_SIGN == nStartAddress)
    {
        Serialprintln(F(" *            => Write Drone Signiture   "));
        
        nEEPRomAddress = EEPROM_DATA_SIGN;
        EEPROM.write(nEEPRomAddress++, 'M');
        EEPROM.write(nEEPRomAddress++, 'a');
        EEPROM.write(nEEPRomAddress++, 'v');
        EEPROM.write(nEEPRomAddress++, 'e');
        EEPROM.write(nEEPRomAddress++, 'r');
        EEPROM.write(nEEPRomAddress++, 'i');
        EEPROM.write(nEEPRomAddress++, 'c');
        EEPROM.write(nEEPRomAddress++, 'k');
    }
    else if(EEPROM_DATA_MPU_AXIS == nStartAddress)
    {
        Serialprintln(F(" *            => Write MPU Type   "));
        
        nEEPRomAddress = EEPROM_DATA_MPU_AXIS0_TYPE;

        Serialprint(F("                  "));
        for(i=0 ; i<3 ; i++)
        {
            EEPROM.write(nEEPRomAddress++, _gGyroAccelAxis[i]);
            
            Serialprint(_gGyroAccelAxis[i]);
            Serialprint(F(" / "));
        }
        Serialprintln(F(" "));

    }
    else if(EEPROM_DATA_RC_TYPE == nStartAddress)
    {
        Serialprintln(F(" *            => Write Transmitter Type   "));
        
        nEEPRomAddress = EEPROM_DATA_RC_CH0_TYPE;
        
        Serialprint(F("                  "));
        for(i=0 ; i<5 ; i++)
        {
            EEPROM.write(nEEPRomAddress++, _gRCChMap[i]);
            
            Serialprint(_gRCChMap[i]);
            Serialprint(F(" / "));
        }
        Serialprintln(F(" "));
        
    }
    else if(EEPROM_DATA_RC_RANGE == nStartAddress)
    {
        Serialprintln(F(" *            => Write Transmitter Range   "));
        
        nEEPRomAddress = EEPROM_DATA_RC_CH0_LOW_H;
        
        for(i=0 ; i<4 ; i++)
        {
            EEPROM.write(nEEPRomAddress++, _gRCSignal_L[i] >> 8);
            EEPROM.write(nEEPRomAddress++, _gRCSignal_L[i] & B11111111);
            EEPROM.write(nEEPRomAddress++, _gRCSignal_M[i] >> 8);
            EEPROM.write(nEEPRomAddress++, _gRCSignal_M[i] & B11111111);
            EEPROM.write(nEEPRomAddress++, _gRCSignal_H[i] >> 8);
            EEPROM.write(nEEPRomAddress++, _gRCSignal_H[i] & B11111111);

            Serialprint(F("                  "));
            Serialprint(_gRCSignal_L[i]);
            Serialprint(F(" / "));
            Serialprint(_gRCSignal_M[i]);
            Serialprint(F(" / "));
            Serialprintln(_gRCSignal_H[i]);
        }
        Serialprintln(F(" "));
    }
    
    delay(300);
    
    Serialprintln(F(" *            => Done!!   "));
}


void _EEPROM_Clear()
{
    int                 i = 0;
    
    for(i=EEPROM_DATA_RESERVED ; i<EEPROM_DATA_MAX ; i++)
        EEPROM.write(i, 0);
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


