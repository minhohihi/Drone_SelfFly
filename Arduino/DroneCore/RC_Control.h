//
//  RC_Controller.h
//  SelfFly
//
//  Created by Maverick on 2016. 1. 21..
//

#ifndef __RC_CONTROL__
#define __RC_CONTROL__

void _Wait_Receiver();
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

    _Wait_Receiver();
    
    delay(300);
    
    Serialprintln(F(" *            => Done!!   "));
}


void _RC_GetRCRange()
{
    int                     i = 0;
    int                     nOffset = 100;
    byte                    nFlag = 0;
    
    for(i=0 ; i<5 ; i++)
    {
        nLowRC[i] = 9999;
        nCenterRC[i] = 1500;
        nHighRC[i] = 0;
    }

    Serialprintln(F(" *      Start Receiver Range Check   "));
    Serialprintln(F(" *      Please Keep Moving Remote Controller   "));
    Serialprint(F(" *      "));
    
    while(nFlag < 15)
    {
        // Set Min & Max Range
        if(nRcvChVal[0] < nLowRC[0])  nLowRC[0]  = nRcvChVal[0];
        if(nRcvChVal[0] > nHighRC[0]) nHighRC[0] = nRcvChVal[0];
        if(nRcvChVal[1] < nLowRC[1])  nLowRC[1]  = nRcvChVal[1];
        if(nRcvChVal[1] > nHighRC[1]) nHighRC[1] = nRcvChVal[1];
        if(nRcvChVal[2] < nLowRC[2])  nLowRC[2]  = nRcvChVal[2];
        if(nRcvChVal[2] > nHighRC[2]) nHighRC[2] = nRcvChVal[2];
        if(nRcvChVal[3] < nLowRC[3])  nLowRC[3]  = nRcvChVal[3];
        if(nRcvChVal[3] > nHighRC[3]) nHighRC[3] = nRcvChVal[3];

        // Check Center Position
        if((nRcvChVal[0] > (nCenterRC[0] - nOffset)) && (nRcvChVal[0] < (nCenterRC[0] + nOffset)))  nFlag |= 0b00000001;
        if((nRcvChVal[1] > (nCenterRC[1] - nOffset)) && (nRcvChVal[1] < (nCenterRC[1] + nOffset)))  nFlag |= 0b00000010;
        if((nRcvChVal[2] > (nCenterRC[2] - nOffset)) && (nRcvChVal[2] < (nCenterRC[2] + nOffset)))  nFlag |= 0b00000100;
        if((nRcvChVal[3] > (nCenterRC[3] - nOffset)) && (nRcvChVal[3] < (nCenterRC[3] + nOffset)))  nFlag |= 0b00001000;

        Serialprint(F("."));
        
        delay(50);
    }
    
    Serialprintln(F(" *            => Done!!   "));
    
    for(i=0 ; i<4 ; i++)
        nCenterRC[i] = (nHighRC[i] + nLowRC[i]) / 2;
        
    for(i=0 ; i<4 ; i++)
    {
        Serialprint(F(" *              Ch"));
        Serialprint(i);
        Serialprint(F(" Range: "));
        Serialprint(nLowRC[i]);
        Serialprint(F(" ~ "));
        Serialprint(nCenterRC[i]);
        Serialprint(F(" ~ "));
        Serialprintln(nHighRC[i]);
    }
    
    Serialprintln(F(" "));
}

//This part converts the actual receiver signals to a standardized 1000 – 1500 – 2000 microsecond value.
//The stored data in the EEPROM is used.
void _RC_Compensate(unsigned char nRCCh)
{
    unsigned char           nReverse = nRCReverseFlag[nRCCh];
    int                     nLow = nLowRC[nRCCh];
    int                     nCenter = nCenterRC[nRCCh];
    int                     nHigh = nHighRC[nRCCh];
    int                     nActualRC = nRcvChVal[nRCCh];
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
    
    nCompensatedRCVal[nRCCh] = nCompensatedRC;
}


void _Wait_Receiver()
{
    byte                nFlag = 0;
    
    while(nFlag < 15)
    {
        if(nRcvChVal[CH_TYPE_ROLL] < 2100 && nRcvChVal[CH_TYPE_ROLL] > 900)
            nFlag |= 0b00000001;
        
        if(nRcvChVal[CH_TYPE_PITCH] < 2100 && nRcvChVal[CH_TYPE_PITCH] > 900)
            nFlag |= 0b00000010;
        
        if(nRcvChVal[CH_TYPE_THROTTLE] < 2100 && nRcvChVal[CH_TYPE_THROTTLE] > 900)
            nFlag |= 0b00000100;
        
        if(nRcvChVal[CH_TYPE_YAW] < 2100 && nRcvChVal[CH_TYPE_YAW] > 900)
            nFlag |= 0b00001000;
        
        delay(500);
    }
}


void _Read_RCData_From_EEPROM()
{
    int                 i = 0;
    int                 nRCType = 0;
    int                 nEEPRomAddress = 0;
    
    Serialprintln(F("   ")); Serialprintln(F("   ")); Serialprintln(F("   ")); Serialprintln(F("   "));
    Serialprintln(F("   **********************************************   "));
    Serialprintln(F(" *       Reading Drone Setting from EEPROM          "));
    Serialprintln(F("   **********************************************   "));
    
    // Read Range of Transmitter
    for(i=EEPROM_DATA_RC_CH0_TYPE ; i<=EEPROM_DATA_RC_CH4_TYPE ; i++)
        nEEPROMData[i] = EEPROM.read(i);

    for(i=EEPROM_DATA_RC_CH0_REVERSE ; i<=EEPROM_DATA_RC_CH4_REVERSE ; i++)
        nEEPROMData[i] = EEPROM.read(i);
    
    for(i=EEPROM_DATA_RC_CH0_LOW_H ; i<=EEPROM_DATA_RC_CH4_HIG_L ; i++)
        nEEPROMData[i] = EEPROM.read(i);
    
    delay(300);
    
    Serialprintln(F(" *            => Done!!   "));
}


#if 1
ISR(PCINT2_vect)
{
    nCurrTime = micros();
    
    if(PIND & B00000100)
    {
        if(0 == nRcvChFlag[CH_TYPE_ROLL])
        {
            nRcvChFlag[CH_TYPE_ROLL] = 1;
            nRcvChHighTime[CH_TYPE_ROLL] = nCurrTime;
        }
    }
    else if(1 == nRcvChFlag[CH_TYPE_ROLL])
    {
        nRcvChFlag[CH_TYPE_ROLL] = 0;
        nRcvChVal[CH_TYPE_ROLL] = nCurrTime - nRcvChHighTime[CH_TYPE_ROLL];
    }

    if(PIND & B00001000)
    {
        if(0 == nRcvChFlag[CH_TYPE_PITCH])
        {
            nRcvChFlag[CH_TYPE_PITCH] = 1;
            nRcvChHighTime[CH_TYPE_PITCH] = nCurrTime;
        }
    }
    else if(1 == nRcvChFlag[CH_TYPE_PITCH])
    {
        nRcvChFlag[CH_TYPE_PITCH] = 0;
        nRcvChVal[CH_TYPE_PITCH] = nCurrTime - nRcvChHighTime[CH_TYPE_PITCH];
    }

    if(PIND & B00010000)
    {
        if(0 == nRcvChFlag[CH_TYPE_THROTTLE])
        {
            nRcvChFlag[CH_TYPE_THROTTLE] = 1;
            nRcvChHighTime[CH_TYPE_THROTTLE] = nCurrTime;
        }
    }
    else if(1 == nRcvChFlag[CH_TYPE_THROTTLE])
    {
        nRcvChFlag[CH_TYPE_THROTTLE] = 0;
        nRcvChVal[CH_TYPE_THROTTLE] = nCurrTime - nRcvChHighTime[CH_TYPE_THROTTLE];
    }

    if(PIND & B00100000)
    {
        if(0 == nRcvChFlag[CH_TYPE_YAW])
        {
            nRcvChFlag[CH_TYPE_YAW] = 1;
            nRcvChHighTime[CH_TYPE_YAW] = nCurrTime;
        }
    }
    else if(1 == nRcvChFlag[CH_TYPE_YAW])
    {
        nRcvChFlag[CH_TYPE_YAW] = 0;
        nRcvChVal[CH_TYPE_YAW] = nCurrTime - nRcvChHighTime[CH_TYPE_YAW];
    }
    
    if(PIND & B01000000)
    {
        if(0 == nRcvChFlag[CH_TYPE_TAKE_LAND])
        {
            nRcvChFlag[CH_TYPE_TAKE_LAND] = 1;
            nRcvChHighTime[CH_TYPE_TAKE_LAND] = nCurrTime;
        }
    }
    else if(1 == nRcvChFlag[CH_TYPE_TAKE_LAND])
    {
        nRcvChFlag[CH_TYPE_TAKE_LAND] = 0;
        nRcvChVal[CH_TYPE_TAKE_LAND] = nCurrTime - nRcvChHighTime[CH_TYPE_TAKE_LAND];
    }
}
#else
ISR(PCINT2_vect)
{
    nCurrTime = micros();

    if(PIND & B00000100)
    {
        if(0 == nRCPrevChangeTime[CH_TYPE_ROLL])
            nRCPrevChangeTime[CH_TYPE_ROLL] = nCurrTime;
    }
    else if(0 != nRCPrevChangeTime[CH_TYPE_ROLL])
    {
        nRcvChVal[CH_TYPE_ROLL] = nCurrTime - nRCPrevChangeTime[CH_TYPE_ROLL];
        nRCPrevChangeTime[CH_TYPE_ROLL] = 0;
    }

    if(PIND & B00001000)
    {
        if(0 == nRCPrevChangeTime[CH_TYPE_PITCH])
            nRCPrevChangeTime[CH_TYPE_PITCH] = nCurrTime;
    }
    else if(0 != nRCPrevChangeTime[CH_TYPE_PITCH])
    {
        nRcvChVal[CH_TYPE_PITCH] = nCurrTime - nRCPrevChangeTime[CH_TYPE_PITCH];
        nRCPrevChangeTime[CH_TYPE_PITCH] = 0;
    }

    if(PIND & B00010000)
    {
        if(0 == nRCPrevChangeTime[CH_TYPE_THROTTLE])
            nRCPrevChangeTime[CH_TYPE_THROTTLE] = nCurrTime;
    }
    else if(0 != nRCPrevChangeTime[CH_TYPE_THROTTLE])
    {
        nRcvChVal[CH_TYPE_THROTTLE] = nCurrTime - nRCPrevChangeTime[CH_TYPE_THROTTLE];
        nRCPrevChangeTime[CH_TYPE_THROTTLE] = 0;
    }

    if(PIND & B00100000)
    {
        if(0 == nRCPrevChangeTime[CH_TYPE_YAW])
            nRCPrevChangeTime[CH_TYPE_YAW] = nCurrTime;
    }
    else if(0 != nRCPrevChangeTime[CH_TYPE_YAW])
    {
        nRcvChVal[CH_TYPE_YAW] = nCurrTime - nRCPrevChangeTime[CH_TYPE_YAW];
        nRCPrevChangeTime[CH_TYPE_YAW] = 0;
    }

    if(PIND & B01000000)
    {
        if(0 == nRCPrevChangeTime[CH_TYPE_TAKE_LAND])
            nRCPrevChangeTime[CH_TYPE_TAKE_LAND] = nCurrTime;
    }
    else if(0 != nRCPrevChangeTime[CH_TYPE_TAKE_LAND])
    {
        nRcvChVal[CH_TYPE_TAKE_LAND] = nCurrTime - nRCPrevChangeTime[CH_TYPE_TAKE_LAND];
        nRCPrevChangeTime[CH_TYPE_TAKE_LAND] = 0;
    }
}
#endif

#endif /* RC_Controller_h */

