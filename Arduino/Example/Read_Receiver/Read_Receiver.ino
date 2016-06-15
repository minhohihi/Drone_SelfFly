
#include <EEPROM.h>
#include "CommHeader.h"

#define Serialprint(...)                Serial.print(__VA_ARGS__)
#define Serialprintln(...)              Serial.println(__VA_ARGS__)

typedef enum _RC_CH_Type
{
    CH_TYPE_ROLL                = 0,
    CH_TYPE_PITCH,
    CH_TYPE_THROTTLE,
    CH_TYPE_YAW,
    CH_TYPE_TAKE_LAND,
    CH_TYPE_MAX,
}RC_CH_Type;

byte                nEEPROMData[EEPROM_DATA_MAX];
unsigned long       nCurrTime;
int                 nLowRC[CH_TYPE_MAX] = {0, };
int                 nCenterRC[CH_TYPE_MAX] = {0, };
int                 nHighRC[CH_TYPE_MAX] = {0, };
int                 nRCReverseFlag[CH_TYPE_MAX] = {0, };
byte                nRcvChFlag[CH_TYPE_MAX] = {0, };
unsigned long       nRcvChHighTime[CH_TYPE_MAX] = {0, };
int                 nRcvChVal[CH_TYPE_MAX] = {0, };
int                 nCompensatedRCVal[CH_TYPE_MAX] = {0, };
int                 nRCChType[CH_TYPE_MAX] = {0, };
byte                nRCChReverse[CH_TYPE_MAX] = {0, };
int                 nCheckProcessDone = 0;

void _RC_Initialize();
void _RC_EstimateRCRange();
void _RC_Compensate(unsigned char nRCCh);
void _RC_CheckStickType(int nRCType);
void _Wait_Receiver();
void _print_CaturedRC_Signals();
void _print_CompensatedRC_Signals();


void setup() 
{
    Serial.begin(115200);
    Serial.flush();

    Serialprintln(F("   ")); Serialprintln(F("   ")); Serialprintln(F("   ")); Serialprintln(F("   "));
    Serialprintln(F("   **********************************************   "));
    Serialprintln(F("   **********************************************   "));

    _RC_Initialize();

    Serialprintln(F("   **********************************************   "));
    Serialprintln(F("   **********************************************   "));
    Serialprintln(F("   ")); Serialprintln(F("   ")); Serialprintln(F("   ")); Serialprintln(F("   "));    
}

void loop()
{
    int                     i = 0;

    if(1 == nCheckProcessDone)
    {
        int                 nError = 0;

        Serialprintln(F("   ")); Serialprintln(F("   ")); Serialprintln(F("   ")); Serialprintln(F("   "));
        Serialprintln(F("   **********************************************   "));
        Serialprintln(F(" *        Mapping Each Stick to RC Channel          "));
        Serialprintln(F("   **********************************************   "));
        Serialprintln(F("  "));
        Serialprintln(F(" *      Please Roll Stick to Right, then Return to Center  "));
        _RC_CheckStickType(CH_TYPE_ROLL);
        Serialprint(F(" *        => Roll Stick is Mapped to Ch"));Serialprintln(nRCChType[CH_TYPE_ROLL]);
        Serialprintln(F("  "));
        Serialprintln(F(" *      Please Pitch Stick to Up, then Return to Center  "));
        _RC_CheckStickType(CH_TYPE_PITCH);
        Serialprint(F(" *        => Pitch Stick is Mapped to Ch"));Serialprintln(nRCChType[CH_TYPE_PITCH]);
        Serialprintln(F("  "));
        Serialprintln(F(" *      Please Throttle Stick to Up, then Return to Center  "));
        _RC_CheckStickType(CH_TYPE_THROTTLE);
        Serialprint(F(" *        => Throttle Stick is Mapped to Ch"));Serialprintln(nRCChType[CH_TYPE_THROTTLE]);
        Serialprintln(F("  "));
        Serialprintln(F(" *      Please YAW Stick to Right, then Return to Center  "));
        _RC_CheckStickType(CH_TYPE_YAW);
        Serialprint(F(" *        => Yaw Stick is Mapped to Ch"));Serialprintln(nRCChType[CH_TYPE_YAW]);
        Serialprintln(F("  "));
        Serialprintln(F(" *      Please Take & Land Stick to Up, then Down  "));
        _RC_CheckStickType(CH_TYPE_TAKE_LAND);
        Serialprint(F(" *        => Yaw Stick is Mapped to Ch"));Serialprintln(nRCChType[CH_TYPE_TAKE_LAND]);


        Serialprintln(F("   ")); Serialprintln(F("   ")); Serialprintln(F("   ")); Serialprintln(F("   "));
        Serialprintln(F("   **********************************************   "));
        Serialprintln(F(" *       Get Min & Max Range of Transmitter         "));
        Serialprintln(F("   **********************************************   "));
        Serialprintln(F("  "));
        Serialprintln(F(" *       Please Keep Moving Sticks End to End   "));
        _RC_EstimateRCRange();

        // Write RC Data to EEPROM
        _Write_RCData_To_EEPROM();

        // Read RC Data from EEPROM
        _Read_RCData_From_EEPROM();

        // Verify EEPROM Data
        {
            int                 nRCType = 0;
            int                 nEEPRomAddress = 0;

            nEEPRomAddress = EEPROM_DATA_RC_CH0_LOW_H;
            for(i=0 ; i<CH_TYPE_MAX ; i++, nEEPRomAddress+=6)
            {
                nRCType = nRCChType[i];
                if((nEEPROMData[nEEPRomAddress] != (nLowRC[nRCType] >> 8)) ||
                    (nEEPROMData[nEEPRomAddress+1] != (nLowRC[nRCType] & 0b11111111)) ||
                    (nEEPROMData[nEEPRomAddress+2] != (nCenterRC[nRCType] >> 8)) ||
                    (nEEPROMData[nEEPRomAddress+3] != (nCenterRC[nRCType] & 0b11111111)) ||
                    (nEEPROMData[nEEPRomAddress+4] != (nHighRC[nRCType] >> 8)) ||
                    (nEEPROMData[nEEPRomAddress+5] != (nHighRC[nRCType] & 0b11111111)))
                    nError = 1;
            }

            nEEPRomAddress = EEPROM_DATA_RC_CH0_TYPE;
            for(i=0 ; i<CH_TYPE_MAX ; i++, nEEPRomAddress++)
                if(nEEPROMData[nEEPRomAddress] != nRCChType[i])
                    nError = 1;

            nEEPRomAddress = EEPROM_DATA_RC_CH0_REVERSE;
            for(i=0 ; i<CH_TYPE_MAX ; i++, nEEPRomAddress++)
                if(nEEPROMData[nEEPRomAddress] != nRCChReverse[i])
                    nError = 1;
        }

        if(0 == nError)
        {
            Serialprintln(F("   **********************************************   "));
            Serialprintln(F(" *           Writing Data is Succeed!!!             "));
            Serialprintln(F(" *         Next Step is Calibrating Gyro            "));
            Serialprintln(F("   **********************************************   "));

            nCheckProcessDone = 1;
        }
        else
        {
            Serialprintln(F("   **********************************************   "));
            Serialprintln(F(" *    There is Something Wrong!!! Try Again!!!      "));
            Serialprintln(F("   **********************************************   "));
        }
    }
    else
    {
        // Get Receiver Input
        // Then Mapping Actual Reciever Value to 1000 ~ 2000
        for(i=0 ; i<CH_TYPE_MAX ; i++)
            _RC_Compensate(i);

        _print_CaturedRC_Signals();
     
        _print_CompensatedRC_Signals();

        Serialprintln(F("   "));
    }
}


void _RC_Initialize()
{
    Serialprintln(F(" *      Start Receiver Module Initialize   "));

    PCICR |= (1 << PCIE2);                // Set PCIE2 to Enable Scan ISR
    PCMSK2 |= (1 << PCINT18);             // Set Digital Input 2 as RC Input (Roll)
    PCMSK2 |= (1 << PCINT19);             // Set Digital Input 3 as RC Input (Pitch)
    PCMSK2 |= (1 << PCINT20);             // Set Digital Input 4 as RC Input (Throttle)
    PCMSK2 |= (1 << PCINT21);             // Set Digital Input 5 as RC Input (Yaw)
    PCMSK2 |= (1 << PCINT22);             // Set Digital Input 6 as RC Input (Landing & TakeOff)

    Serialprintln(F(" *        Pease Move All Sticks to Center"));
    _Wait_Receiver();
    
    delay(300);
    
    Serialprintln(F(" *            => Done!!   "));
}


void _RC_EstimateRCRange()
{
    int                     i = 0;
    int                     nRCType = 0;
    int                     nOffset = 100;
    byte                    nFlag = 0;
    
    for(i=0 ; i<CH_TYPE_MAX ; i++)
    {
        nLowRC[i] = 9999;
        nCenterRC[i] = 1500;
        nHighRC[i] = 0;
    }

    while(nFlag < 31)
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
        if(nRcvChVal[4] < nLowRC[4])  nLowRC[4]  = nRcvChVal[4];
        if(nRcvChVal[4] > nHighRC[4]) nHighRC[4] = nRcvChVal[4];

        // Check Center Position
        if((nRcvChVal[0] > (nCenterRC[0] - nOffset)) && (nRcvChVal[0] < (nCenterRC[0] + nOffset)))  nFlag |= 0b00000001;
        if((nRcvChVal[1] > (nCenterRC[1] - nOffset)) && (nRcvChVal[1] < (nCenterRC[1] + nOffset)))  nFlag |= 0b00000010;
        if((nRcvChVal[2] > (nCenterRC[2] - nOffset)) && (nRcvChVal[2] < (nCenterRC[2] + nOffset)))  nFlag |= 0b00000100;
        if((nRcvChVal[3] > (nCenterRC[3] - nOffset)) && (nRcvChVal[3] < (nCenterRC[3] + nOffset)))  nFlag |= 0b00001000;
        if((nRcvChVal[4] > (nCenterRC[4] - nOffset)) && (nRcvChVal[4] < (nCenterRC[4] + nOffset)))  nFlag |= 0b00010000;

        Serialprint(F("."));
        
        delay(50);
    }

    nRCType = nRCChType[0];
    nCenterRC[nRCType] = (nLowRC[nRCType] + nHighRC[nRCType]) / 2;
    Serialprint(F(" *         Roll Ch Range: "));Serialprint(nLowRC[nRCType]);Serialprint(F(" ~ "));Serialprint(nCenterRC[nRCType]);Serialprint(F(" ~ "));Serialprintln(nHighRC[nRCType]);

    nRCType = nRCChType[1];
    nCenterRC[nRCType] = (nLowRC[nRCType] + nHighRC[nRCType]) / 2;
    Serialprint(F(" *        Pitch Ch Range: "));Serialprint(nLowRC[nRCType]);Serialprint(F(" ~ "));Serialprint(nCenterRC[nRCType]);Serialprint(F(" ~ "));Serialprintln(nHighRC[nRCType]);

    nRCType = nRCChType[2];
    nCenterRC[nRCType] = (nLowRC[nRCType] + nHighRC[nRCType]) / 2;
    Serialprint(F(" *     Throttle Ch Range: "));Serialprint(nLowRC[nRCType]);Serialprint(F(" ~ "));Serialprint(nCenterRC[nRCType]);Serialprint(F(" ~ "));Serialprintln(nHighRC[nRCType]);

    nRCType = nRCChType[3];
    nCenterRC[nRCType] = (nLowRC[nRCType] + nHighRC[nRCType]) / 2;
    Serialprint(F(" *          Yaw Ch Range: "));Serialprint(nLowRC[nRCType]);Serialprint(F(" ~ "));Serialprint(nCenterRC[nRCType]);Serialprint(F(" ~ "));Serialprintln(nHighRC[nRCType]);

    nRCType = nRCChType[4];
    nCenterRC[nRCType] = (nLowRC[nRCType] + nHighRC[nRCType]) / 2;
    Serialprint(F(" *  Take & Land Ch Range: "));Serialprint(nLowRC[nRCType]);Serialprint(F(" ~ "));Serialprint(nCenterRC[nRCType]);Serialprint(F(" ~ "));Serialprintln(nHighRC[nRCType]);

    Serialprintln(F(" *            => Done!!   "));
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


void _RC_CheckStickType(int nRCType)
{
    int                 nMin = 1300;
    int                 nMax = 1700;
    int                 nFlag = -1;
    int                 nReveserFlag = 0;

    if(nRcvChVal[0] > nMax && nRcvChVal[0] < nMin)
        nFlag = 0;

    if(nRcvChVal[1] > nMax && nRcvChVal[1] < nMin)
        nFlag = 1;

    if(nRcvChVal[2] > nMax && nRcvChVal[2] < nMin)
        nFlag = 2;

    if(nRcvChVal[3] > nMax && nRcvChVal[3] < nMin)
        nFlag = 3;

    nRCChType[nRCType] = nFlag;
    nRCChReverse[nRCType] = 0;
    if(nRcvChVal[nFlag] < nMin)
        nRCChReverse[nRCType] = 1;

    _Wait_Receiver();
}


void _Wait_Receiver()
{
    byte                nFlag = 0;
    int                 nMin = 1300;
    int                 nMax = 1700;

    while(nFlag < 15)
    {
        if(nRcvChVal[0] < nMax && nRcvChVal[0] > nMin)
            nFlag |= 0b00000001;
        
        if(nRcvChVal[1] < nMax && nRcvChVal[1] > nMin)
            nFlag |= 0b00000010;
        
        if(nRcvChVal[2] < nMax && nRcvChVal[2] > nMin)
            nFlag |= 0b00000100;
        
        if(nRcvChVal[3] < nMax && nRcvChVal[3] > nMin)
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


void _Write_RCData_To_EEPROM()
{
    int                 i = 0;
    int                 nRCType = 0;
    int                 nEEPRomAddress = 0;

    Serialprintln(F("   ")); Serialprintln(F("   ")); Serialprintln(F("   ")); Serialprintln(F("   "));
    Serialprintln(F("   **********************************************   "));
    Serialprintln(F(" *         Writing Drone Setting to EEPROM          "));
    Serialprintln(F("   **********************************************   "));

    // Write Range of Transmitter
    Serialprintln(F(" *            => Write Transmitter Range   "));

    nEEPRomAddress = EEPROM_DATA_RC_CH0_LOW_H;
    for(i=0 ; i<CH_TYPE_MAX ; i++, nEEPRomAddress+=6)
    {
        nRCType = nRCChType[i];
        EEPROM.write(nEEPRomAddress,   nLowRC[nRCType] >> 8);
        EEPROM.write(nEEPRomAddress+1, nLowRC[nRCType] & 0b11111111);
        EEPROM.write(nEEPRomAddress+2, nCenterRC[nRCType] >> 8);
        EEPROM.write(nEEPRomAddress+3, nCenterRC[nRCType] & 0b11111111);
        EEPROM.write(nEEPRomAddress+4, nHighRC[nRCType] >> 8);
        EEPROM.write(nEEPRomAddress+5, nHighRC[nRCType] & 0b11111111);
    }

    nEEPRomAddress = EEPROM_DATA_RC_CH0_TYPE;
    for(i=0 ; i<CH_TYPE_MAX ; i++, nEEPRomAddress++)
        EEPROM.write(nEEPRomAddress, nRCChType[i]);

    nEEPRomAddress = EEPROM_DATA_RC_CH0_REVERSE;
    for(i=0 ; i<CH_TYPE_MAX ; i++, nEEPRomAddress++)
        EEPROM.write(nEEPRomAddress, nRCChReverse[i]);

    delay(300);

    Serialprintln(F(" *            => Done!!   "));
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

