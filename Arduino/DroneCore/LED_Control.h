//
//  LED_Controller.h
//  SelfFly
//
//  Created by Maverick on 2016. 1. 21..
//

#ifndef __LED_CONTROL__
#define __LED_CONTROL__

void _LED_SetColor(int nRed, int nGreen, int nBlue);
void _LED_Blink(int nRed, int nGreen, int nBlue, int32_t nLinkPeriod);
void _LED_DispStatus(int nCase);

void _LED_Initialize()
{
    int             i = 0;

    _LED_DispStatus(0);
    
    // Set Digital Port 7, 12, and 13 as Output
    DDRB |= B00110000;
    DDRD |= B10000000;

    // Set Value of Digital Port 7, 12, and 13 as Low to Initialize LED
    PORTB &= B11001111;
    PORTD &= B01111111;

    _LED_Blink(1, 1, 1, 0);
    
    delay(1000);

    for(i=0 ; i<5 ; i++)
    {
        _LED_Blink(1, 0, 0, 0);
        delay(120);
        _LED_Blink(0, 1, 0, 0);
        delay(120);
        _LED_Blink(0, 0, 1, 0);
        delay(120);
        
        _LED_DispStatus(1);
    }

    // Set RED Led as Init Color
    _LED_Blink(1, 0, 0, 0);

    _gPrevBlinkTime = micros();
    
    delay(300);
    
    _LED_DispStatus(2);
}


void _LED_SetColor(int nRed, int nGreen, int nBlue)
{
    // Digital Port 7
    if(0 != nRed)
        PORTD |= B10000000;
    else
        PORTD &= B01111111;

    // Digital Port 13
    if(0 != nGreen)
        PORTB |= B00100000;
    else
        PORTB &= B11011111;

    // Digital Port 12
    if(0 != nBlue)
        PORTB |= B00010000;
    else
        PORTB &= B11101111;
}


void _LED_Blink(int nRed, int nGreen, int nBlue, int32_t nLinkPeriod)
{
    _gCurrTime = micros();

    if(0 == nLinkPeriod)
        _LED_SetColor(nRed, nGreen, nBlue);
    else if((_gCurrTime - _gPrevBlinkTime) > nLinkPeriod)
    {
        if(0 == _gLED_Status)
            _LED_SetColor(nRed, nGreen, nBlue);
        else
            _LED_SetColor(0, 0, 0);

        _gLED_Status = !(_gLED_Status);

        _gPrevBlinkTime = _gCurrTime;
    }
}


void _LED_DispStatus(int nCase)
{
    #if PRINT_SERIAL
        if(0 == nCase)
        {
            Serialprint(F(" *      "));
            Serialprint(_gDroneInitStep++);
            Serialprintln(F(". Start LED Module Initialization   "));
        }
        else if(1 == nCase)
            Serialprint(F("."));
        else if(2 == nCase)
            Serialprintln(F(" Done!!"));
    #elif USE_LCD_DISPLAY
        {
            static int nCnt = 0;
            
            if(0 == nCase)
            {
                delay(500);
                _gLCDHndl.clear();
                
                _gLCDHndl.setCursor(0, 0);
                _gLCDHndl.print(_gDroneInitStep++);
                _gLCDHndl.setCursor(1, 0);
                _gLCDHndl.print(".Init LED");
            }
            else if(1 == nCase)
            {
                _gLCDHndl.setCursor(nCnt++, 1);
                if(1 == nCnt)
                {
                    delay(500);                  
                    _gLCDHndl.print("Test:");
                    nCnt += 4;
                }
                else
                    _gLCDHndl.print(".");
            }
            else if(2 == nCase)
            {
                _gLCDHndl.setCursor(nCnt, 1);
                _gLCDHndl.print("Done!");
                delay(1000);
            }
        }
    #endif
}

#endif /* LED_Controller_h */




