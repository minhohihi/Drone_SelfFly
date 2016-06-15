
#include <EEPROM.h>             //Include the EEPROM.h library so we can store information onto the EEPROM


#define Serialprint(...)                Serial.print(__VA_ARGS__)
#define Serialprintln(...)              Serial.println(__VA_ARGS__)

unsigned long       nCurrTime;
int                 nLowRC[5];
int                 nCenterRC[5];
int                 nHighRC[5];
int                 nRCReverseFlag[5];
byte                nRcvChFlag[5];
unsigned long       nRcvChHighTime[5];
int                 nRcvChVal[5];
int                 nCompensatedRCVal[5];

void _RC_Initialize();
void _RC_GetRCRange();
void _RC_Compensate(unsigned char nRCCh);
void _Wait_Receiver();
void _print_CaturedRC_Signals();
void _print_CompensatedRC_Signals();


void setup() 
{
    Serialprintln(F("   ")); Serialprintln(F("   ")); Serialprintln(F("   ")); Serialprintln(F("   "));
    Serialprintln(F("   **********************************************   "));
    Serialprintln(F("   **********************************************   "));

    Serial.begin(115200);
    
    _RC_Initialize();

    _RC_GetRCRange();
    
    Serialprintln(F("   **********************************************   "));
    Serialprintln(F("   **********************************************   "));
    Serialprintln(F("   ")); Serialprintln(F("   ")); Serialprintln(F("   ")); Serialprintln(F("   "));    
}

void loop()
{
    int                     i = 0;

    // Get Receiver Input
    // Then Mapping Actual Reciever Value to 1000 ~ 2000
    for(i=0 ; i<5 ; i++)
        _RC_Compensate(i);

    _print_CaturedRC_Signals();
 
    _print_CompensatedRC_Signals();

}


void _RC_Initialize()
{
    Serialprintln(F(" *      1. Start Receiver Module Initialize   "));

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
        if(nRcvChVal[0] < 2100 && nRcvChVal[0] > 900)
            nFlag |= 0b00000001;
        
        if(nRcvChVal[1] < 2100 && nRcvChVal[1] > 900)
            nFlag |= 0b00000010;
        
        if(nRcvChVal[2] < 2100 && nRcvChVal[2] > 900)
            nFlag |= 0b00000100;
        
        if(nRcvChVal[3] < 2100 && nRcvChVal[3] > 900)
            nFlag |= 0b00001000;
        
        delay(500);
    }
}


void _print_CaturedRC_Signals()
{
    Serialprint(F("   //   RC_Roll:"));
    Serialprint(nRcvChVal[0]);
    Serialprint(F("   RC_Pitch:"));
    Serialprint(nRcvChVal[1]);
    Serialprint(F("   RC_Throttle:"));
    Serialprint(nRcvChVal[2]);
    Serialprint(F("   RC_Yaw:"));
    Serialprint(nRcvChVal[3]);
    Serialprint(F("   RC_Gear:"));
    Serialprint(nRcvChVal[4]);
}


void _print_CompensatedRC_Signals()
{
    Serialprint(F("   //   RC_Roll:"));
    Serialprint(nCompensatedRCVal[0]);
    Serialprint(F("   RC_Pitch:"));
    Serialprint(nCompensatedRCVal[1]);
    Serialprint(F("   RC_Throttle:"));
    Serialprint(nCompensatedRCVal[2]);
    Serialprint(F("   RC_Yaw:"));
    Serialprint(nCompensatedRCVal[3]);
    Serialprint(F("   RC_Gear:"));
    Serialprint(nCompensatedRCVal[4]);
}


ISR(PCINT2_vect)
{
    nCurrTime = micros();
    
    if(PIND & B00000100)
    {
        if(0 == nRcvChFlag[0])
        {
            nRcvChFlag[0] = 1;
            nRcvChHighTime[0] = nCurrTime;
        }
    }
    else if(1 == nRcvChFlag[0])
    {
        nRcvChFlag[0] = 0;
        nRcvChVal[0] = nCurrTime - nRcvChHighTime[0];
    }

    if(PIND & B00001000)
    {
        if(0 == nRcvChFlag[1])
        {
            nRcvChFlag[1] = 1;
            nRcvChHighTime[1] = nCurrTime;
        }
    }
    else if(1 == nRcvChFlag[1])
    {
        nRcvChFlag[1] = 0;
        nRcvChVal[1] = nCurrTime - nRcvChHighTime[1];
    }

    if(PIND & B00010000)
    {
        if(0 == nRcvChFlag[2])
        {
            nRcvChFlag[2] = 1;
            nRcvChHighTime[2] = nCurrTime;
        }
    }
    else if(1 == nRcvChFlag[2])
    {
        nRcvChFlag[2] = 0;
        nRcvChVal[2] = nCurrTime - nRcvChHighTime[2];
    }

    if(PIND & B00100000)
    {
        if(0 == nRcvChFlag[3])
        {
            nRcvChFlag[3] = 1;
            nRcvChHighTime[3] = nCurrTime;
        }
    }
    else if(1 == nRcvChFlag[3])
    {
        nRcvChFlag[3] = 0;
        nRcvChVal[3] = nCurrTime - nRcvChHighTime[3];
    }
    
    if(PIND & B01000000)
    {
        if(0 == nRcvChFlag[4])
        {
            nRcvChFlag[4] = 1;
            nRcvChHighTime[4] = nCurrTime;
        }
    }
    else if(1 == nRcvChFlag[4])
    {
        nRcvChFlag[4] = 0;
        nRcvChVal[4] = nCurrTime - nRcvChHighTime[4];
    }
}

