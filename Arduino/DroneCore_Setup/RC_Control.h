//
//  RC_Controller.h
//  SelfFly
//
//  Created by Maverick on 2016. 1. 21..
//

#ifndef __RC_CONTROL__
#define __RC_CONTROL__

void _RC_Wait_Signal();
void _RC_GetRCRange();

void _RC_Initialize()
{
    Serialprintln(F(" *      4. Start Receiver Module Initialization   "));

    PCICR |= (1 << PCIE2);                // Set PCIE2 to Enable Scan ISR
    PCMSK2 |= (1 << PCINT18);             // Set Digital Input 2 as RC Input (Roll)
    PCMSK2 |= (1 << PCINT19);             // Set Digital Input 3 as RC Input (Pitch)
    PCMSK2 |= (1 << PCINT20);             // Set Digital Input 4 as RC Input (Throttle)
    PCMSK2 |= (1 << PCINT21);             // Set Digital Input 5 as RC Input (Yaw)
    PCMSK2 |= (1 << PCINT22);             // Set Digital Input 6 as RC Input (Landing & TakeOff)

    _RC_Wait_Signal();
    
    delay(300);
    
    Serialprintln(F(" *            => Done!!   "));
}


void _Read_RCData_From_EEPROM()
{
    int                 i = 0;
    int                 nRCType = 0;
    int                 nEEPRomAddress = 0;
    
    Serialprintln(F("   ")); Serialprintln(F("   ")); Serialprintln(F("   ")); Serialprintln(F("   "));
    Serialprintln(F("********************************************************************"));
    Serialprintln(F(" *       Reading Drone Setting from EEPROM          "));
    Serialprintln(F("********************************************************************"));
    
    // Read Range of Transmitter
    for(i=EEPROM_DATA_RC_CH0_TYPE ; i<=EEPROM_DATA_RC_CH4_TYPE ; i++)
        _gEEPROMData[i] = EEPROM.read(i);
    
    for(i=EEPROM_DATA_RC_CH0_REVERSE ; i<=EEPROM_DATA_RC_CH4_REVERSE ; i++)
        _gEEPROMData[i] = EEPROM.read(i);
    
    for(i=EEPROM_DATA_RC_CH0_LOW_H ; i<=EEPROM_DATA_RC_CH4_HIG_L ; i++)
        _gEEPROMData[i] = EEPROM.read(i);
    
    delay(300);
    
    Serialprintln(F(" *            => Done!!   "));
}


//This part converts the actual receiver signals to a standardized 1000 – 1500 – 2000 microsecond value.
//The stored data in the EEPROM is used.
void _RC_Compensate(byte nRCCh)
{
    byte                    nReverse = _gRCReverseFlag[nRCCh];
    int                     nLow = _gRCSignal_L[nRCCh];
    int                     nCenter = _gRCSignal_M[nRCCh];
    int                     nHigh = _gRCSignal_H[nRCCh];
    int                     nActualRC = _gRCSignalVal[nRCCh];
    int                     nDiff = 0;
    int                     nCompensatedRC = 0;
  
    if(nActualRC < nCenter)
    {
        //The actual receiver value is lower than the center value
        // Limit the lowest value to the value that was detected during setup
        if(nActualRC < nLow)
            nActualRC = nLow;
        
        // Calculate and scale the actual value to a 1000 - 2000us value
        nDiff = ((long)(nCenter - nActualRC) * (long)500) / (nCenter - nLow);
        
        // If the channel is reversed
        if(nReverse == 1)
            nCompensatedRC = 1500 + nDiff;
        else
            nCompensatedRC = 1500 - nDiff;
    }
    else if(nActualRC > nCenter)
    {
        //The actual receiver value is higher than the center value
        //Limit the lowest value to the value that was detected during setup
        if(nActualRC > nHigh)
            nActualRC = nHigh;
        
        //Calculate and scale the actual value to a 1000 - 2000us value
        nDiff = ((long)(nActualRC - nCenter) * (long)500) / (nHigh - nCenter);
        
        //If the channel is reversed
        if(nReverse == 1)
            nCompensatedRC = 1500 - nDiff;
        else
            nCompensatedRC = 1500 + nDiff;
    }
    else
        nCompensatedRC = 1500;
    
    _gCompensatedRCVal[nRCCh] = nCompensatedRC;
}


void _RC_Wait_Signal()
{
    byte                nFlag = 0;
    
    while(nFlag < 15)
    {
        if(_gRCSignalVal[CH_TYPE_ROLL] < 2100 && _gRCSignalVal[CH_TYPE_ROLL] > 900)
            nFlag |= B00000001;
        
        if(_gRCSignalVal[CH_TYPE_PITCH] < 2100 && _gRCSignalVal[CH_TYPE_PITCH] > 900)
            nFlag |= B00000010;
        
        if(_gRCSignalVal[CH_TYPE_THROTTLE] < 2100 && _gRCSignalVal[CH_TYPE_THROTTLE] > 900)
            nFlag |= B00000100;
        
        if(_gRCSignalVal[CH_TYPE_YAW] < 2100 && _gRCSignalVal[CH_TYPE_YAW] > 900)
            nFlag |= B00001000;
        
        delay(500);
    }
}


void _RC_Wait_CenterPos()
{
    byte                nFlag = 0;
    const int           nCneterOffset = 100;
    const int           nCneterOffsetH = 1500 + nCneterOffset;
    const int           nCneterOffsetL = 1500 - nCneterOffset;
    
    while(nFlag < 15)
    {
        if(_gRCSignalVal[CH_TYPE_ROLL] < nCneterOffsetH && _gRCSignalVal[CH_TYPE_ROLL] > nCneterOffsetL)
            nFlag |= B00000001;
        
        if(_gRCSignalVal[CH_TYPE_PITCH] < nCneterOffsetH && _gRCSignalVal[CH_TYPE_PITCH] > nCneterOffsetL)
            nFlag |= B00000010;
        
        if(_gRCSignalVal[CH_TYPE_THROTTLE] < nCneterOffsetH && _gRCSignalVal[CH_TYPE_THROTTLE] > nCneterOffsetL)
            nFlag |= B00000100;
        
        if(_gRCSignalVal[CH_TYPE_YAW] < nCneterOffsetH && _gRCSignalVal[CH_TYPE_YAW] > nCneterOffsetL)
            nFlag |= B00001000;
        
        delay(500);
    }
}


void _RC_CheckAxis(int nAxisIdx)
{
    int                     nChNum = 0;
    
    if(0 == nAxisIdx)
        Serialprintln(F("1. Please Throttle Up and Back to Center"));
    else if(1 == nAxisIdx)
        Serialprintln(F("2. Please Roll to Left Wing Up and Back to Center"));
    else if(2 == nAxisIdx)
        Serialprintln(F("3. Please Pitch to Nose Up and Back to Center"));
    else if(3 == nAxisIdx)
        Serialprintln(F("4. Please Yaw to Nose Right and Back to Center"));
    
    if((_gRCSignalVal[0] < 1200) || (_gRCSignalVal[0] > 1700)) nChNum = 0;
    if((_gRCSignalVal[1] < 1200) || (_gRCSignalVal[1] > 1700)) nChNum = 1;
    if((_gRCSignalVal[2] < 1200) || (_gRCSignalVal[2] > 1700)) nChNum = 2;
    if((_gRCSignalVal[3] < 1200) || (_gRCSignalVal[3] > 1700)) nChNum = 3;

    if(0 == nAxisIdx)
        _gRCChAxis[nChNum] = CH_TYPE_THROTTLE;
    else if(1 == nAxisIdx)
        _gRCChAxis[nChNum] = CH_TYPE_ROLL;
    else if(2 == nAxisIdx)
        _gRCChAxis[nChNum] = CH_TYPE_PITCH;
    else if(3 == nAxisIdx)
        _gRCChAxis[nChNum] = CH_TYPE_YAW;
    
    // Check RC Signal is Inverse of Not
    Serialprint(F("       This Channel ["));
    Serialprint(nChNum);
    if((_gRCSignalVal[0] < 1200) || (_gRCSignalVal[1] < 1200) || (_gRCSignalVal[2] < 1200) || (_gRCSignalVal[3] < 1200))
    {
        _gRCChAxis[nChNum] |= B10000000;
        Serialprint(F("] is Inverted"));
    }
    else
        Serialprint(F("] is Not Inverted"));
    
    Serialprintln(F(" "));
    _RC_Wait_CenterPos();
}


void _RC_GetRCRange()
{
    int                     i = 0;
    int                     nOffset = 100;
    byte                    nFlag = 0;
    
    for(i=0 ; i<5 ; i++)
    {
        _gRCSignal_L[i] = 9999;
        _gRCSignal_M[i] = 1500;
        _gRCSignal_H[i] = 0;
    }
    
    Serialprintln(F(" *      Start Receiver Range Check   "));
    Serialprintln(F(" *      Please Keep Moving Remote Controller   "));
    Serialprint(F(" *      "));
    
    while(nFlag < 15)
    {
        // Set Min & Max Range
        if(_gRCSignalVal[0] < _gRCSignal_L[0])  _gRCSignal_L[0]  = _gRCSignalVal[0];
        if(_gRCSignalVal[0] > _gRCSignal_H[0]) _gRCSignal_H[0] = _gRCSignalVal[0];
        if(_gRCSignalVal[1] < _gRCSignal_L[1])  _gRCSignal_L[1]  = _gRCSignalVal[1];
        if(_gRCSignalVal[1] > _gRCSignal_H[1]) _gRCSignal_H[1] = _gRCSignalVal[1];
        if(_gRCSignalVal[2] < _gRCSignal_L[2])  _gRCSignal_L[2]  = _gRCSignalVal[2];
        if(_gRCSignalVal[2] > _gRCSignal_H[2]) _gRCSignal_H[2] = _gRCSignalVal[2];
        if(_gRCSignalVal[3] < _gRCSignal_L[3])  _gRCSignal_L[3]  = _gRCSignalVal[3];
        if(_gRCSignalVal[3] > _gRCSignal_H[3]) _gRCSignal_H[3] = _gRCSignalVal[3];
        
        // Check Center Position
        if((_gRCSignalVal[0] > (_gRCSignal_M[0] - nOffset)) && (_gRCSignalVal[0] < (_gRCSignal_M[0] + nOffset)))  nFlag |= B00000001;
        if((_gRCSignalVal[1] > (_gRCSignal_M[1] - nOffset)) && (_gRCSignalVal[1] < (_gRCSignal_M[1] + nOffset)))  nFlag |= B00000010;
        if((_gRCSignalVal[2] > (_gRCSignal_M[2] - nOffset)) && (_gRCSignalVal[2] < (_gRCSignal_M[2] + nOffset)))  nFlag |= B00000100;
        if((_gRCSignalVal[3] > (_gRCSignal_M[3] - nOffset)) && (_gRCSignalVal[3] < (_gRCSignal_M[3] + nOffset)))  nFlag |= B00001000;
        
        Serialprint(F("."));
        
        delay(50);
    }
    
    Serialprintln(F(" *            => Done!!   "));
    
    for(i=0 ; i<4 ; i++)
        _gRCSignal_M[i] = (_gRCSignal_H[i] + _gRCSignal_L[i]) / 2;
    
    for(i=0 ; i<4 ; i++)
    {
        Serialprint(F(" *              Ch"));
        Serialprint(i);
        Serialprint(F(" Range: "));
        Serialprint(_gRCSignal_L[i]);
        Serialprint(F(" ~ "));
        Serialprint(_gRCSignal_M[i]);
        Serialprint(F(" ~ "));
        Serialprintln(_gRCSignal_H[i]);
    }
    
    Serialprintln(F(" "));
    
    _RC_Wait_CenterPos();
}


// Get Transmitter Signal From Digital Pin 2, 3, 4, 5, and 6 by HW Interrupt
ISR(PCINT2_vect)
{
    _gCurrTime = micros();
    
    // Check Status of Digital Pin 2
    if(PIND & B00000100)
    {
        if(0 == (_gRCRisingFlag & B00000001))
        {
            _gRCRisingFlag |= B00000001;
            _gRCChRisingTime[0] = _gCurrTime;
        }
    }
    else if(_gRCRisingFlag & B00000001)
    {
        _gRCRisingFlag &= B11111110;
        _gRCSignalVal[0] = _gCurrTime - _gRCChRisingTime[0];
    }

    // Check Status of Digital Pin 3
    if(PIND & B00001000)
    {
        if(0 == (_gRCRisingFlag & B00000010))
        {
            _gRCRisingFlag |= B00000010;
            _gRCChRisingTime[1] = _gCurrTime;
        }
    }
    else if(_gRCRisingFlag & B00000010)
    {
        _gRCRisingFlag &= B11111101;
        _gRCSignalVal[1] = _gCurrTime - _gRCChRisingTime[1];
    }

    // Check Status of Digital Pin 4
    if(PIND & B00010000)
    {
        if(0 == (_gRCRisingFlag & B00000100))
        {
            _gRCRisingFlag |= B00000100;
            _gRCChRisingTime[2] = _gCurrTime;
        }
    }
    else if(_gRCRisingFlag & B00000100)
    {
        _gRCRisingFlag &= B11111011;
        _gRCSignalVal[2] = _gCurrTime - _gRCChRisingTime[2];
    }

    // Check Status of Digital Pin 5
    if(PIND & B00100000)
    {
        if(0 == (_gRCRisingFlag & B00001000))
        {
            _gRCRisingFlag |= B00001000;
            _gRCChRisingTime[3] = _gCurrTime;
        }
    }
    else if(_gRCRisingFlag & B00001000)
    {
        _gRCRisingFlag &= B11110111;
        _gRCSignalVal[3] = _gCurrTime - _gRCChRisingTime[3];
    }
    
    // Check Status of Digital Pin 6
    if(PIND & B01000000)
    {
        if(0 == (_gRCRisingFlag & B00010000))
        {
            _gRCRisingFlag |= B00010000;
            _gRCChRisingTime[4] = _gCurrTime;
        }
    }
    else if(_gRCRisingFlag & B00010000)
    {
        _gRCRisingFlag &= B11101111;
        _gRCSignalVal[4] = _gCurrTime - _gRCChRisingTime[4];
    }
}

#endif /* RC_Controller_h */

